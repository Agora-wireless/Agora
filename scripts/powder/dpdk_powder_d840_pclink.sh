#!/bin/bash
# Helper file to configure powder d740 node (pclink, per agora profile) for dpdk
# assumes intel nic.  Make sure kernel options (intel_iommu=on iommu=pt) are set
sudo dpdk-devbind.py --status
#D840 (not required to take ens2f0 down)
#D840 -- setup dpdk on 1 pclink
sudo ip link set dev ens2f1 down
sudo dpdk-devbind.py -u 0000:25:00.1
sudo modprobe vfio-pci
sudo dpdk-devbind.py --bind=vfio-pci 0000:25:00.1
#display status
sudo dpdk-devbind.py --status