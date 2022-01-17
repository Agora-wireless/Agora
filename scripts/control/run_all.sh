#! /bin/bash

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )
ROOT_DIR=$( cd ${DIR}/../.. >/dev/null 2>&1 && pwd )

source ${ROOT_DIR}/scripts/utils/utils.sh

# Running states
mode=0
slot_us=1000
data_gen=1

while getopts "h?:frs:d" opt; do
    case "$opt" in
        h|\?)
            echo "Help"
            echo -e "\t-h\tShow this infomation"
            echo -e "\t-f\tRun application without generating new data"
            echo -e "\t-r\tRun RRU only"
            echo -e "\t-s\tSet slot size (unit: us)"
            echo -e "\t-d\tGenerate data only"
            exit 0
            ;;
        f)
            data_gen=0
            ;;
        r)
            mode=1
            ;;
        s)  
            slot_us=${OPTARG}
            if [ "${slot_us}" -lt 500 ]; then
                echored "Slot size should be no smaller than 500us"
                exit
            fi
            if [ "${slot_us}" -gt 10000 ]; then
                echored "Slot size should be no larger than 10000us"
                exit
            fi
            ;;
        d)
            mode=2
            ;;
    esac
done

# Initialize the info of the platform:
# app_name, servers, NIC info
source ${ROOT_DIR}/scripts/control/init_platform.sh

# Check the validity of the deployment config files
source ${ROOT_DIR}/scripts/control/check_deploy.sh

# Create config for servers
source ${ROOT_DIR}/scripts/control/create_config.sh

if [ "${data_gen}" == 1 ]; then
    # Generate control data and traffic data
    source ${ROOT_DIR}/scripts/control/gen_data.sh
fi

mkdir -p /tmp/Hydra

if [ "${mode}" == 0 ]; then
    # Run the Hydra application
    source ${ROOT_DIR}/scripts/control/run_hydra.sh

    # Give Hydra servers 5 seconds to initialize
    sleep 5
fi

if [ "${mode}" == 0 ] || [ "${mode}" == 1 ]; then
    # Run the RRU application
    source ${ROOT_DIR}/scripts/control/run_rru.sh
fi

wait