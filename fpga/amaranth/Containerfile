FROM ubuntu:mantic

ARG DEBIAN_FRONTEND=noninteractive

# Install Amaranth Prerequsites
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y python3 python3-pip gtkwave yosys

RUN pip3 install --user --upgrade amaranth

