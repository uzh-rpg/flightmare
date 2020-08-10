FROM ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive 

RUN apt-get update && apt-get install -y --no-install-recommends \
   build-essential \
   python3 python3-dev python3-pip \
   cmake \
   git \
   ca-certificates \
   libzmqpp-dev \
   libopencv-dev \
   && rm -rf /var/lib/apt/lists/*

RUN echo "export FLIGHTMARE_DIR=/__w/flightmare/flightmare" >> ~/.bashrc 
RUN /bin/bash -c "source ~/.bashrc" 
RUN echo $FLIGHTMARE_DIR

