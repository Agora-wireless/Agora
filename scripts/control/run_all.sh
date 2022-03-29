#! /bin/bash

# Run this script on your laptop
# Run the entire Hydra application across all remote servers
# Record the results and logs to /tmp/hydra on your laptop
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

# 1 for generating new input traffic data, 0 for not
GEN_NEW_TRAFFIC_DATA=1

# 1 for using MATLAB to generate Rayleigh channel input traffic data
# 0 for not generating Rayleigh channel input traffic data, generate normal random data
# TODO: integrate this option
USE_MATLAB_GEN_RAYLEIGH=0

# 1 for print diagnosis digest for each Hydra server
# 0 for not print
PRINT_DIGEST_ALL=1

# 1 for diagnosis
# 0 for do not diagnosis
ERROR_DIAGNOSIS=1

while getopts "h?:frdmsx" opt; do
  case "$opt" in
    h|\?)
      echo "Help"
      echo -e "\t-h\tShow this infomation"
      echo -e "\t-f\tRun Hydra without generating new input traffic data"
      echo -e "\t-r\tRun Hydra RRU traffic generator only"
      echo -e "\t-d\tenerate new input traffic data"
      echo -e "\t-m\tUse MATLAB to generate input traffic data for Rayleigh channel (require MATLAB to be installed)"
      echo -e "\t-s\tDo not print diagnosis digest for each Hydra server"
      echo -e "\t-x\tDo not diagnosis"
      exit 0
      ;;
    f)
      GEN_NEW_TRAFFIC_DATA=0
      ;;
    r)
      HYDRA_RUN_MODE=1
      ;;
    d)
      HYDRA_RUN_MODE=2
      ;;
    m)
      USE_MATLAB_GEN_RAYLEIGH=1
      ;;
    s)
      PRINT_DIGEST_ALL=1
      ;;
    x)
      ERROR_DIAGNOSIS=0
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

echocyan "Checking hugepage size on remote servers"
source ${hydra_root_dir}/scripts/control/check_hugepage.sh

# Check the validity of the deployment config files
echocyan "Checking the validity of configuration files"
source ${hydra_root_dir}/scripts/control/check_deploy.sh

# Create config for servers
echocyan "Create and deploy configuration file for each remote server"
source ${hydra_root_dir}/scripts/control/create_config.sh

mkdir -p /tmp/hydra
rm -f /tmp/hydra/install.log

if [ "${GEN_NEW_TRAFFIC_DATA}" == 1 ]; then
  # Generate control data and traffic data
  echocyan "Generate RRU traffic data"
  source ${hydra_root_dir}/scripts/control/gen_data.sh || \
    { echored "Failed to generate RRU traffic data. Please check /tmp/hydra/install.log for details"; exit 1; }
fi

if [ "${HYDRA_RUN_MODE}" == 2 ]; then
  exit
fi

if [ "${HYDRA_RUN_MODE}" == 0 ]; then
  # Run the Hydra application
  echocyan "Run Hydra applications and wait the initialization to finish"
  source ${hydra_root_dir}/scripts/control/run_hydra.sh
fi

if [ "${HYDRA_RUN_MODE}" == 0 ] || [ "${HYDRA_RUN_MODE}" == 1 ]; then
  # Run the RRU application
  echocyan "Run RRU traffic generators (slot size: ${slot_us} us)"
  source ${hydra_root_dir}/scripts/control/run_rru.sh
fi

time_to_run=$(( slot_us*slots_to_test/1000000+25 ))
echocyan "Wait ${time_to_run} seconds for the test to finish"
sleep ${time_to_run}

set +e

source ${hydra_root_dir}/scripts/control/check_running_all.sh

if [ "${is_running}" == "1" ]; then
  echored "Hydra is still running (timeout), please check the log files. Now kill all Hydra processes"
  source ${hydra_root_dir}/scripts/control/stop_all.sh
  exit 1
fi

echocyan "All running logs will be stored in /tmp/hydra/log_[server].txt"

if [ "${HYDRA_RUN_MODE}" == 0 ] && [ "${bigstation_mode}" != "true" ]; then
  if [ "${ERROR_DIAGNOSIS}" == 1 ]; then
    source ${hydra_root_dir}/scripts/control/error_diagnosis.sh
  fi
fi