#!/bin/bash

groupadd -g $USER_GID $USER_NAME
useradd -s /bin/bash -g $USER_GID -u $USER_UID $USER_NAME

sudo -E -u $USER_NAME cp -R $INPUT_PATH/* $OUTPUT_PATH
cd $OUTPUT_PATH
sudo -E -u $USER_NAME env "PATH=$PATH" $@