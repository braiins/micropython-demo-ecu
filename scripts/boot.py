# -*- coding: utf-8 -*-
import hal
import bsp
import uio
import app
import sys

mount_path = "/mnt"
counter_path = "%s/counter" % mount_path

def write_csv(out_file, list):
    line = ','.join(map(str, list)) + '\n'
    count = sys.stdout.write(line)

    if out_file is not None:
        count = out_file.write(line)

def pipe__init(a):
    conf_path = '%s/pipe.cfg' % mount_path
    with uio.open(conf_path, 'r') as conf:
        content = conf.readlines()

    pipe_ctl_channel = int(content[0])
    full_throttle_threshold = int(content[1])
    print('Setting tuned pipe control channel %s, full throttle @ %s' % (pipe_ctl_channel, full_throttle_threshold))
    a.pipe_ctl_channel(pipe_ctl_channel)
    a.full_throttle_threshold(full_throttle_threshold)

    print('Reading tuned pipe configuration')
    i = 0
    for line in content[2:]:
        pipe_setting = [i] + [int(x) for x in line.split()]
        print('idx: %s rpm: %s length: %s' % (pipe_setting[0], pipe_setting[1], pipe_setting[2]))

        a.pipe_table_entry(pipe_setting)
        i += 1

print('XFFBAZOQBooting ECU logger')
print(bsp.sd_config)
# Temporary workaround - receiver configuration - RX polarity pin has to be
# activated and set to 0 (so that the XOR gate on apogee doesn't invert the
# serial signal
print('Configuring RX polarity pin')
rx_pol = hal.GPIO_PIN(bsp.rx_polarity)
rx_pol.write(0)


sd = hal.SD(1, bsp.sd_config)
hal.mount(sd, mount_path)
print('File system mounted: %s' % mount_path)


log_id = 0
try:
    f = uio.open(counter_path, 'r')
except OSError:
    print("Log ID file doesn't exist, starting from 0")
else:
    try:
        log_id_str = f.readall()
        log_id = int(log_id_str)
    except ValueError as e:
        print("Cannot read counter '%s' (%s), starting from 0" %
              (log_id_str, e))
    else:
        print('Closing counter file')
        f.close()


new_log_id = log_id + 1
print('Writing next ID (%s) into counter file' % new_log_id)
f = uio.open(counter_path, 'w')
f.write('%s' % new_log_id)
print('Closing counter file')
f.close()

print('Starting application')
a = app.get()
pipe__init(a)

# Starting logging telemetry data
log_file = None
try:
    log_file_path = '%s/log-%d.csv' % (mount_path, log_id)
    log_file = uio.open(log_file_path, 'a')
except Exception as e:
    print('Failed to open log file: %s, %s' % (log_file_path, e))

print(log_file)
write_csv(log_file,
          ['timestamp[ms]',
           'rpm',
           'tc1 status',
           'tc1[C]',
           'tc1_cold_junction[C]',
           'rudder(1)',
           'mixture(2)',
           'throttle(3)',
           'ctrim(4)',
           'boost(5)',
           'aux(6)',
           'auto_pipe(7)',
           'aux(8)',
           'pipe-idx',
           'last-rpm',
           'dec-rpm',
           'gps_fix',
           'gps_sig',
           'gps-speed',
           'lat',
           'lon',
           'elv',
           'pdop',
           'hdop',
           'vdop',
           'direction'
           ])

lines_written = 0
#for i in range(1, 10):
while True:
    time_stamp = hal.millis()
    (fix, sig, speed, lat, lon, elv, direction, pdop, hdop, vdop) = \
        a.gps()
    data = [time_stamp, a.rpm()]
    temp1 = list(a.tc_temp())
    data.extend(temp1[0:3])
    rc_channels = list(a.rc_channels())
    data.extend(rc_channels[0:8])
    pipe_length_telem_data = list(a.pipe_length_telem_data())
    data.extend(pipe_length_telem_data)
    data.extend([fix, sig, speed, lat, lon, elv, direction, pdop, hdop, vdop])
    write_csv(log_file, data)

    lines_written += 1
    # flush every 100 lines
    if lines_written == 100:
        log_file.flush()
        lines_written = 0

    a.jeti_ex_sensors((a.rpm(), int(speed*10), int(temp1[1]*10),
                      pipe_length_telem_data[0], rc_channels[1]))
    # Sleep for the rest of this 20ms slot
    hal.sleep_until_ms(time_stamp, 20)


# This will never be reached
log_file.close()

# Unmount the SD card
hal.mount(None, mount_path)

