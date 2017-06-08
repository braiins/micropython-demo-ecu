#!/bin/bash

# Make sure the image has been built:

docker build -t scons-pila-builder .

# This runs the scons container while allowing the user access the X11
# session. The display access is needed if you want to run the
# configuration target. The build runs with user credentials to
# prevent any intermediate build output files in ../build/ directory
# from being owned by root.
GID=`id -g`
docker run --rm -v ${PWD}/../../../:/src -e DISPLAY \
       -v /tmp/.X11-unix:/tmp/.X11-unix scons-pila-builder /bin/sh -c " \
	 addgroup --system $USER --gid=$GID; \
	 adduser --system $USER --uid=$UID --ingroup=$USER; \
	 cd src/demos/demo-ecu; \
	 sudo -u $USER scons $@
       "
