#!/bin/bash

# NOTE: To delete interface: sudo ip link delete <tunInterface>
#                            e.g., sudo ip link delete tun45

THIS_USER=$USER
THIS_GROUP=$GROUPS
echo $THIS_USER $THIS_GROUP
sudo ip tuntap add mode tun user $THIS_USER group $THIS_GROUP dev tun45 pi 
echo "Set up ip link (tun45 interface)"
sudo ip link set tun45 up
echo "Add IP address to interface"
sudo ip addr add 10.0.0.2/24 dev tun45
