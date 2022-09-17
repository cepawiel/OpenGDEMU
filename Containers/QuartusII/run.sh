#!/bin/bash

set -x

ENTRY_FLAGS="--entrypoint /bin/bash"
LICENSE_FLAGS="-v /home/coltonp/Git/OpenGDEMU/Containers/QuartusII/license.dat:/quartus_license.dat -e LM_LICENSE_FILE=/quartus_license.dat"
ETHERNET_MAC="46:CD:8F:3C:A6:06"
# ETHERNET_MAC="00:00:00:00:00:00"

xhost +local:docker

docker run --rm -it --privileged \
    -e DISPLAY=:0 --ipc=host -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v /home/coltonp/Git/OpenGDEMU/Containers/QuartusII/home:/home/coltonp \
    -v /home/coltonp/Git/OpenGDEMU/FPGA:/home/coltonp/Git/OpenGDEMU/FPGA \
    --mac-address ${ETHERNET_MAC} \
    ${LICENSE_FLAGS} \
    ${ENTRY_FLAGS} \
    quartus13

xhost -local:docker

set +x