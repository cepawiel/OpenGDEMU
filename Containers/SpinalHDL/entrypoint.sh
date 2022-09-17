#!/bin/bash

groupadd -g $USER_GID $USER_NAME
useradd -m -d /userhome -s /bin/bash -g $USER_GID -u $USER_UID $USER_NAME

chown -R $USER_UID:$USER_GID out/

echo $@
# sudo -u $USER_NAME mill opengdemu.runMain opengdemu.OpenGDEMUVerilog
sudo -u $USER_NAME mill $@