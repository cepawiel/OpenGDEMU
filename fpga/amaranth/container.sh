#!/bin/bash

# Wrapper script to build & start container
# passes all args from script to container

# Exit script on error
set -e

# Build container
docker build -t amaranth .

# Run container
docker run -it --rm \
    -v $(pwd):$(pwd) \
    -w $(pwd) \
    amaranth \
    $@