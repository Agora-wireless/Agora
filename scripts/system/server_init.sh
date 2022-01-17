#! /bin/bash

sudo apt-get update
sudo apt-get install -y byobu
cd ~
mkdir project
sudo mkfs -t ext3 /dev/sda4 <<< y
sudo mount -t auto /dev/sda4 ./project
sudo chown junzhig ./project
sudo chgrp opensketch-PG0 ./project
