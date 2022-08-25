#!/bin/bash
# Helper file to configure powder d740 node (pclink, per agora profile) for dpdk
# assumes intel nic.  Make sure kernel options (intel_iommu=on iommu=pt) are set
sudo dpdk-devbind.py --status
#D740 (not required to take ens7f0 down)
#sudo ip link set dev ens7f0 down
#sudo dpdk-devbind.py -u 0000:86:00.0
#sudo modprobe vfio-pci
#sudo dpdk-devbind.py --bind=vfio-pci 0000:86:00.0

sudo ip link set dev ens7f1 down
sudo dpdk-devbind.py -u 0000:86:00.1
sudo modprobe vfio-pci
sudo dpdk-devbind.py --bind=vfio-pci 0000:86:00.1
#done with ens7f1
sudo dpdk-devbind.py --status