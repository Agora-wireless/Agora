#! /bin/bash

sudo apt-get update
sudo apt-get install -y byobu
cd ~
mkdir project
sudo mkfs -t ext3 /dev/nvme0n1p4 <<< y
sudo mount -t auto /dev/nvme0n1p4 ./project
sudo chown junzhig ./project
sudo chgrp opensketch-PG0 ./project
cd project
git clone https://github.com/jianding17/Agora.git
cd Agora
git checkout merge_subcarrier_doer_single_queue
