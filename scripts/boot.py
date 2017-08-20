# -*- coding: utf-8 -*-
import hal
import bsp
import uio
import app
import sys
import ujson

mount_path = "/mnt"
counter_path = "%s/counter" % mount_path

def write_csv(out_file, list):
    line = ','.join(map(str, list)) + '\n'
    count = sys.stdout.write(line)

    if out_file is not None:
        count = out_file.write(line)

def mount_sd_card():
    sd = hal.SD(1, bsp.sd_config)
    hal.mount(sd, mount_path)
    print('File system mounted: %s' % mount_path)

def read_and_archive_config(log_id):
    """Reads ECU configuration and creates an archive of the config.

    The archive is useful for determining what was the actual
    configuration for a particular logging session.
    """
    conf_path_fmt = '%s/ecu%s.cfg'
    conf_path = conf_path_fmt % (mount_path, '')
    # ECU configuration archive contains log id in the name
    conf_path_archive = conf_path_fmt % (mount_path, log_id)
    with uio.open(conf_path, 'r') as conf_file:
        conf_data = conf_file.readall()
        conf = ujson.loads(conf_data)
    # copy the data into a new file
    with uio.open(conf_path_archive, 'w') as conf_archive:
        conf_archive.write(conf_data)

    return conf


def get_new_log_id():
    """Helper that creates a new unique ID on the SD card

    Reads the basic counter value from the counter stored on the SD
    card

    @return 4 digit unique log ID string
    """
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
    with uio.open(counter_path, 'w') as f:
        f.write('%s' % new_log_id)
    print('Closing counter file')

    return '%04d' % log_id

def pipe__init(a):
    pipe_ctl_channel = conf['channels']['pipe_ctl']
    full_throttle_threshold = conf['pipe']['full_throttle_threshold']
    print('Setting tuned pipe control channel %s, full throttle @ %s' % (pipe_ctl_channel, full_throttle_threshold))
    a.pipe_ctl_channel(pipe_ctl_channel)
    a.full_throttle_threshold(full_throttle_threshold)

    print('Reading tuned pipe configuration')
    for idx, entry in enumerate(conf['pipe']['table']):
        pipe_setting = [idx] + entry
        print('idx: %s rpm: %s length: %s' % (pipe_setting[0], pipe_setting[1], pipe_setting[2]))

        a.pipe_table_entry(pipe_setting)

class LinearMapping(object):
    def __init__(self, x2, x1, y2, y1):
        self.k = (y2 - y1) / (x2 - x1)
        self.q = y1 - (self.k * x1)

    def map(self, x):
        return self.k * x + self.q

def mixture__init_controller():
    #    mixture_pid = Pid(sample_time_ms=conf['mixture']['sample_time_ms'])
    mixture_pid = Pid(sample_time_ms=50)
    lean_srv_limit = conf['mixture']['lean']
    rich_srv_limit = conf['mixture']['rich']

    # Mixture control is a reverse process -> when the current
    # temperature is higher than desired, we have to make the mixture
    # richer. This is true only iff lean_srv_limit < rich_srv_limit
    # TODO: this has to be called before set_pid_params -> fix PID
    # implementation accordingly
    if lean_srv_limit < rich_srv_limit:
        mixture_pid.set_output_direction(reverse_direction=True)

    mixture_pid.set_pid_params(conf['mixture']['pid']['kp'],
                               conf['mixture']['pid']['ki'],
                               conf['mixture']['pid']['kd'])
    # Set PID limits based on physical servo limits
    mixture_pid.set_output_limits(min(lean_srv_limit, rich_srv_limit),
                                  max(lean_srv_limit, rich_srv_limit))
    mixture_pid.setpoint = conf['mixture']['temp_setpoint']
    setpoint_range = conf['mixture']['temp_range']
    mixture_ctl_2_setpoint = LinearMapping(rich_srv_limit, lean_srv_limit,
                                           mixture_pid.setpoint + setpoint_range,
                                           mixture_pid.setpoint - setpoint_range)

    mixture_ctl_2_mixture = LinearMapping(rich_srv_limit, lean_srv_limit, 100, 0)

    return (mixture_pid, mixture_ctl_2_setpoint, mixture_ctl_2_mixture)

def mixture__recalculate_setpoint(a, rc_channels):
    """Recalculates temperature setpoint based on user setting
    """
    # fetch setting of the potentionmeter on the remote controller
    mixture_ctl = rc_channels[conf['channels']['mixture_ctl']]
    mixture_pid.setpoint = mixture_ctl_2_setpoint.map(mixture_ctl)

def mixture__control_temp(a, now, rc_channels):
    (discard1, exhaust_temp, cold_junction, discard2, discard3) = a.tc_temp(0)
    mixture_pid.curr_input = exhaust_temp
    print('MIX: autoswitch:%s currlevel: %s, conflevel: %s' % (
        conf['channels']['mixture_auto_switch'],
        rc_channels[conf['channels']['mixture_auto_switch']],
        conf['ctl_switch_levels'][1]
    ))
    # Enable/disable automatic mode of the PID controller
    if rc_channels[conf['channels']['mixture_auto_switch']] > \
       conf['ctl_switch_levels'][1]:
        mixture__recalculate_setpoint(a, rc_channels)
        mixture_pid.enable_auto_mode()
        mixture_pid.compute(now)
    else:
        # In manual mode copy user output value to the PID, so that it
        # is always ready for automatic mode
        mixture_pid.disable_auto_mode()
        mixture_pid.output = rc_channels[conf['channels']['mixture_ctl']]

    a.set_mixture_valve(int(mixture_pid.output))

print('XFFBAZOQBooting ECU logger')
print(bsp.sd_config)
# Temporary workaround - receiver configuration - RX polarity pin has to be
# activated and set to 0 (so that the XOR gate on apogee doesn't invert the
# serial signal
print('Configuring RX polarity pin')
rx_pol = hal.GPIO_PIN(bsp.rx_polarity)
rx_pol.write(0)

# Initialization
mount_sd_card()
log_id = get_new_log_id()
conf = read_and_archive_config(log_id)

print('Starting application')
a = app.get()
pipe__init(a)
(mixture_pid, mixture_ctl_2_setpoint, mixture_ctl_2_mixture) \
    = mixture__init_controller()

# Starting logging telemetry data
log_file = None
try:
    log_file_path = '%s/log-%s.csv' % (mount_path, log_id)
    log_file = uio.open(log_file_path, 'a')
except Exception as e:
    print('Failed to open log file: %s, %s' % (log_file_path, e))

print(log_file)

# populate thermocouple labels
tc_columns_fmt = ['tc%s status', 'tc%s[C]', 'tc%s_cold_junction[C]']
tc_columns = []
for i in range(0, 2):
    tc_columns += [l % i for l in tc_columns_fmt]

write_csv(log_file,
          ['timestamp[ms]',
           'rpm',
          ] +
          tc_columns +
          [
           'exh temp. setpoint',
           'mixture output',
           'mixture_auto'] +
          list(conf['channels'].keys()) +
          ['pipe-idx',
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
    temp = []
    for i in range(0, 2):
        temp.append(a.tc_temp(i))
        data.extend(list(temp[i])[0:3])

    data.extend([mixture_pid.setpoint, mixture_pid.output, mixture_pid.auto_mode])
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

    a.jeti_ex_sensors((a.rpm(), int(speed*10), int(temp[0][1]*10),
                       int(temp[1][1]*10),
                       int(mixture_pid.setpoint*10),
                       int(mixture_ctl_2_mixture.map(mixture_pid.output) * 10),
                       pipe_length_telem_data[0]))

    mixture__control_temp(a, hal.millis(), rc_channels)

    # Sleep for the rest of this 20ms slot
    hal.sleep_until_ms(time_stamp, 20)


# This will never be reached
log_file.close()

# Unmount the SD card
hal.mount(None, mount_path)

