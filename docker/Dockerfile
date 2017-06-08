FROM ubuntu:16.04

RUN apt-get update && apt-get install -y \
    build-essential \
    autoconf \
    flex \
    bison \
    scons \
    qtbase5-dev \
    libgtk-3-dev \
    libncurses5-dev \
    gcc-arm-none-eabi

RUN apt-get update && apt-get install -y \
    git \
    libtool \
    gperf \
    python-mako \
    sudo

WORKDIR /
RUN git clone http://ymorin.is-a-geek.org/git/kconfig-frontends && \
    cd /kconfig-frontends && \
    git checkout tags/v4.11.0.0 && \
    autoreconf -fi && \
    ./configure --prefix=/usr && make install
