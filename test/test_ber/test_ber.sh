#!/bin/sh

SERVER=$1
NUC=$2

# copy logs
scp ${SERVER}:/tmp/mac_log /tmp/macbs.log
scp ${NUC}:/tmp/mac_log /tmp/maclient.log

# compute BER
python3 ber.py -u /tmp/maclient.log -b /tmp/macbs.log

