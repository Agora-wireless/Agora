#!/bin/bash

# NOTE: To delete interface: sudo ip link delete <tunInterface>
#                            e.g., sudo ip link delete tun45

THIS_USER=$USER
THIS_GROUP=$GROUPS
echo $THIS_USER $THIS_GROUP
sudo ip tuntap add mode tun user $THIS_USER group $THIS_GROUP dev tun44 pi 
echo "Set up ip link (tun44 interface)"
sudo ip link set tun44 up
echo "Add IP address to interface"
sudo ip addr add 10.0.0.1/24 dev tun44
echo "Ping 10.0.0.2"
ping 10.0.0.2

