# Engine telemetry and control demo application


This demo shows the use of micropython to read engine telemetry data and 
store it as CSV on the attached SD card.
The realtime part of the application takes care of controlling the tuned pipe
length based on measured RPM.


# Setup

It is necessary to select the target board for which this demo has to
be built.
```
$ cp board/apogee/config.def ./.config
$ scons
```

This results in building the configuration python module. Follow the
instructions and rerun the build:

```
$ scons
```
