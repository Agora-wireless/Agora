#!/bin/bash
mkdir ~/.vnc
curl https://raw.githubusercontent.com/Agora-wireless/Agora/develop/scripts/vnc_xstartup.sh > ~/.vnc/xstartup
chmod +x ~/.vnc/xstartup
vncserver