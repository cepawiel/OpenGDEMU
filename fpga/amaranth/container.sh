#!/bin/bash

# Wrapper script to build & start container
# passes all args from script to container

# Exit script on error
set -e

# Build container. Podman finds the Containerfile on its own; `-f` is kept
# explicit so this still works if docker is substituted, since docker/buildx
# only ever auto-detects a file named `Dockerfile`.
podman build -t amaranth -f Containerfile .

# Run container
podman run -it --rm \
    -v $(pwd):$(pwd) \
    -w $(pwd) \
    amaranth \
    $@