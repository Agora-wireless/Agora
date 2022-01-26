#! /bin/bash

# Run this script on your laptop
# Run the entire Hydra application across all remote servers
# Record the results and logs to /tmp/Hydra on your laptop
# * Options
#   * -f Run Hydra without generating new input traffic data
#   * -r Run Hydra RRU traffic generator only
#   * -s [slot size] Set the slot size used in Hydra (unit: us, default: 1000 us)
#   * -d Generate new input traffic data
#   * -m Use MATLAB to generate input traffic data for Rayleigh channel (require MATLAB to be installed)

set -e

script_dir=$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )
hydra_root_dir=$( cd ${script_dir}/../.. >/dev/null 2>&1 && pwd )

source ${hydra_root_dir}/scripts/utils/utils.sh

# Running mode
# 0 for running the entire Hydra app (including RRU, Hydra)
# 1 for running the RRU traffic generator only
# 2 for not running Hydra and RRU
HYDRA_RUN_MODE=0

# The slot size for running Hydra
SLOT_US=1000

# 1 for generating new input traffic data, 0 for not
GEN_NEW_TRAFFIC_DATA=1

# 1 for using MATLAB to generate Rayleigh channel input traffic data
# 0 for not generating Rayleigh channel input traffic data, generate normal random data
# TODO: integrate this option
USE_MATLAB_GEN_RAYLEIGH=0

while getopts "h?:frs:dm" opt; do
  case "$opt" in
    h|\?)
      echo "Help"
      echo -e "\t-h\tShow this infomation"
      echo -e "\t-f\tRun Hydra without generating new input traffic data"
      echo -e "\t-r\tRun Hydra RRU traffic generator only"
      echo -e "\t-s [slot size]\tSet the slot size used in Hydra (unit: us, default: 1000 us)"
      echo -e "\t-d\tenerate new input traffic data"
      echo -e "\t-m\tUse MATLAB to generate input traffic data for Rayleigh channel (require MATLAB to be installed)"
      exit 0
      ;;
    f)
      GEN_NEW_TRAFFIC_DATA=0
      ;;
    r)
      HYDRA_RUN_MODE=1
      ;;
    s)  
      SLOT_US=${OPTARG}
      if [ "${SLOT_US}" -lt 500 ]; then
        echored "Slot size should be no smaller than 500us"
        exit
      fi
      if [ "${SLOT_US}" -gt 10000 ]; then
        echored "Slot size should be no larger than 10000us"
        exit
      fi
      ;;
    d)
      HYDRA_RUN_MODE=2
      ;;
    m)
      USE_MATLAB_GEN_RAYLEIGH=1
      ;;
  esac
done

checkpkg jq
if [ ${checkpkg_res} == "0" ]; then
  exit
fi

# Initialize the info of the platform:
# app_name, servers, NIC info
source ${hydra_root_dir}/scripts/control/init_platform.sh

# Check the validity of the deployment config files
echocyan "Checking the validity of configuration files"
source ${hydra_root_dir}/scripts/control/check_deploy.sh

# Create config for servers
echocyan "Create and deploy configuration file for each remote server"
source ${hydra_root_dir}/scripts/control/create_config.sh

mkdir -p /tmp/Hydra

if [ "${GEN_NEW_TRAFFIC_DATA}" == 1 ]; then
  # Generate control data and traffic data
  echocyan "Generate RRU traffic data"
  source ${hydra_root_dir}/scripts/control/gen_data.sh
fi

if [ "${HYDRA_RUN_MODE}" == 0 ]; then
  # Run the Hydra application
  echocyan "Run Hydra applications and wait for 5 seconds"
  source ${hydra_root_dir}/scripts/control/run_hydra.sh

  # Give Hydra servers 5 seconds to initialize
  sleep 5
fi

if [ "${HYDRA_RUN_MODE}" == 0 ] || [ "${HYDRA_RUN_MODE}" == 1 ]; then
  # Run the RRU application
  echocyan "Run RRU traffic generators"
  source ${hydra_root_dir}/scripts/control/run_rru.sh
fi

wait