FROM ubuntu:impish

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC \
    apt-get install -y build-essential clang bison flex libreadline-dev \
                     gawk tcl-dev libffi-dev git mercurial graphviz   \
                     xdot pkg-config python python3 libftdi-dev gperf \
                     libboost-program-options-dev autoconf libgmp-dev \
                     cmake curl

# install yosys
RUN git clone https://github.com/YosysHQ/yosys.git yosys && \
    cd yosys && \
    make -j$(nproc) && \
    make install

# install SymbiYosys
RUN git clone https://github.com/YosysHQ/SymbiYosys.git SymbiYosys && \
    cd SymbiYosys && \
    make install

# install Yices 2
RUN git clone https://github.com/SRI-CSL/yices2.git yices2 && \
    cd yices2 && \
    autoconf && \
    ./configure && \
    make -j$(nproc) && \
    make install

# install Z3
RUN git clone https://github.com/Z3Prover/z3.git z3 && \
    cd z3 && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -G "Unix Makefiles" ../ && \
    make -j$(nproc) && \
    make install

SHELL ["/bin/bash", "-c"]

# install bootlector
RUN git clone https://github.com/boolector/boolector && \
    cd boolector && \
    ./contrib/setup-btor2tools.sh && \
    ./contrib/setup-lingeling.sh && \
    ./configure.sh && \
    make -C build -j$(nproc) && \
    cp build/bin/{boolector,btor*} /usr/local/bin/ && \
    cp deps/btor2tools/bin/btorsim /usr/local/bin/

ADD copy_to_output.sh /

WORKDIR /
ENTRYPOINT ["/copy_to_output.sh"]