FROM i386/ubuntu:xenial as base

RUN apt-get update && apt-get install -y libc6 \
    libpng12-0 libfreetype6 libsm6 libxrender1 libfontconfig1 libxtst6 \
    libxft2 libxext6 libxi6 libncurses5 libgtk2.0-0

FROM base as install
# ADD http://download.altera.com/akdlm/software/acdsinst/13.0sp1/232/ib_tar/Quartus-web-13.0.1.232-linux.tar /installer
# ADD Quartus-web-13.0.1.232-linux.tar /installer
RUN mkdir -p /opt/quartus && /installer/setup.sh --mode unattended --installdir /opt/quartus

FROM base
COPY --from=install /opt/quartus /opt/quartus

RUN apt-get update && apt-get install -y sudo

ENV INPUT_PATH=/input
ENV OUTPUT_PATH=/output

ENV USER_NAME=quartus
ENV USER_UID=1000
ENV USER_GID=1000

ENV LM_LICENSE_FILE=/quartus_license.dat
# ADD license.dat ${LM_LICENSE_FILE}

ENV PATH="/opt/quartus/quartus/bin:/opt/quartus/modelsim_ase/linuxaloem:$PATH"

ADD entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
