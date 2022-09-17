#!/bin/bash

set -x

docker run --rm -it \
    -v /home/coltonp/Git/OpenGDEMU/Containers/SymbiYosys/data:/data \
    symbiyosys

set +x