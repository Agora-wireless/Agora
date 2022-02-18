#! /bin/bash

# Find the root directory of Hydra app
script_dir=$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )
hydra_root_dir=$( cd ${script_dir}/../.. >/dev/null 2>&1 && pwd )

installed_armadillo="none"
installed_json="none"
installed_mkl="none"
installed_rdma="none"
installed_dpdk="none"

# Find the progress file
if [ -f ${hydra_root_dir}/progress/progress.txt ]; then
  echocyan "Found progress file: ${hydra_root_dir}/progress/progress.txt. Load the installation progress."
  while IFS='' read -r line || [[ -n "$line" ]]; do
    token=$(echo "$line" | cut -d'=' -f1)
    value=$(echo "$line" | cut -d'=' -f2)
    case ${token} in
      "armadillo")
        installed_armadillo=${value}
        ;;
      "json")
        installed_json=${value}
        ;;
      "mkl")
        installed_mkl=${value}
        ;;
      "rdma")
        installed_rdma=${value}
        ;;
      "dpdk")
        installed_dpdk=${value}
        ;;
      *)
        echored "ERROR: unknown token ${token} in progress file"
        exit
        ;;
    esac
  done < ${hydra_root_dir}/progress/progress.txt
else
  echocyan "Create progress file: ${hydra_root_dir}/progress/progress.txt. Check if any package is installed."
  mkdir -p ${hydra_root_dir}/progress
  source ${hydra_root_dir}/scripts/install/check_global_package.sh
  check_global_armadillo
  check_global_json
  check_global_mkl
  check_global_dpdk
  echo "armadillo=${installed_armadillo}" >> ${hydra_root_dir}/progress/progress.txt
  echo "json=${installed_json}" >> ${hydra_root_dir}/progress/progress.txt
  echo "mkl=${installed_mkl}" >> ${hydra_root_dir}/progress/progress.txt
  echo "rdma=${installed_rdma}" >> ${hydra_root_dir}/progress/progress.txt
  echo "dpdk=${installed_dpdk}" >> ${hydra_root_dir}/progress/progress.txt
fi