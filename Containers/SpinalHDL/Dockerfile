ARG IMAGE="ubuntu:focal"

FROM $IMAGE AS common

WORKDIR /work

RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
      apt-utils \
      apt-transport-https \
      ca-certificates \
      git \
      make \
  && apt-get update -qq && apt-get autoclean && apt-get clean && apt-get -y autoremove \
  && update-ca-certificates


FROM common AS build

RUN apt-get install -y --no-install-recommends \
      autoconf \
      bison \
      clang \
      flex libfl-dev \
      g++ \
      gcc \
      gnat-9 \
      gperf \
      llvm-dev \
      readline-common \
      zlib1g-dev


  # Build GHDL
# RUN git clone https://github.com/ghdl/ghdl && cd ghdl \
#  && git reset --hard "0316f95368837dc163173e7ca52f37ecd8d3591d" \
#  && ./dist/ci-run.sh -b llvm -p ghdl-llvm build \
#  && mv ghdl-llvm.tgz /tmp \
#  && cd ..

  # Build Verilator
# RUN git clone https://github.com/verilator/verilator && cd verilator \
#  && git checkout v4.028 \
#  && unset VERILATOR_ROOT \
#  && autoconf \
#  && ./configure --prefix="/usr/local/"\
#  && make -j$(nproc) \
#  && make install DESTDIR="$(pwd)/install-verilator" \
#  && mv install-verilator/usr/local /tmp/verilator \
#  && cd ..

  # Build iverilog
RUN git clone https://github.com/steveicarus/iverilog --depth=1 --branch v11_0 && cd iverilog \
 && autoconf \
 && ./configure \
 && make -j$(nproc) \
 && make install DESTDIR="/tmp/iverilog"

#
# Add deps to base image: GHDL, Verilator, iverilog, VUnit and cocotb
#

FROM common AS deps

RUN apt-get install -y --no-install-recommends \
      libgnat-9 \
      swig \
      zlib1g-dev \
      libboost-dev \
      build-essential \
  && apt-get autoclean && apt-get clean && apt-get -y autoremove

# COPY --from=build /tmp/ghdl-llvm.tgz /tmp/ghdl.tgz
# COPY --from=build /tmp/verilator/ /usr/local/
COPY --from=build /tmp/iverilog/ /

# RUN tar -xzf /tmp/ghdl.tgz -C /usr/local \
#  && rm -f /tmp/*

# RUN pip3 install --upgrade pip

# RUN git clone --recurse-submodule https://github.com/vunit/vunit /opt/vunit \
#  && pip3 install -r /opt/vunit/requirements.txt
# ENV PYTHONPATH=/opt/vunit

# RUN pip3 install cocotb

#
# Add scala
#

FROM deps AS base

## Set frontend required for docker
ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update -qq && apt-get install -y --no-install-recommends gnupg2 curl && \
    echo "deb https://repo.scala-sbt.org/scalasbt/debian all main" | tee /etc/apt/sources.list.d/sbt.list && \
    echo "deb https://repo.scala-sbt.org/scalasbt/debian /" | tee /etc/apt/sources.list.d/sbt_old.list && \
    curl -sL "https://keyserver.ubuntu.com/pks/lookup?op=get&search=0x2EE0EA64E40A89B84B2DF73499E82A75642AC823" | apt-key add && \
    apt-get update -qq && apt-get install -y --no-install-recommends \
    g++ \
    llvm \
    curl \
    sbt \
    scala \
    sudo \
 && apt-get autoclean && apt-get clean && apt-get -y autoremove

RUN apt-get update -qq && apt-get install -y --no-install-recommends \
      openjdk-11-jdk \
  && apt-get autoclean && apt-get clean && apt-get -y autoremove

RUN curl -s -L https://github.com/com-lihaoyi/mill/releases/download/0.10.11/0.10.11 > /usr/local/bin/mill && \
    chmod +x /usr/local/bin/mill

RUN mill --version

RUN cd /tmp && git clone https://github.com/SpinalHDL/SpinalTemplateSbt && \
    cd /tmp/SpinalTemplateSbt && mill projectname.runMain projectname.MyTopLevelVerilog \
     && cd ../ && rm -rf /tmp/SpinalTemplateSbt

ENV USER_NAME=spinalhdl
ENV USER_UID=1000
ENV USER_GID=1000

ADD entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]