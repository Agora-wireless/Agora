#! /bin/bash

script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
hydra_root_dir=$( cd ${script_dir}/../.. >/dev/null 2>&1 && pwd )

source ${hydra_root_dir}/scripts/utils/utils.sh
source ${hydra_root_dir}/scripts/control/init_platform.sh

server_list=("roce91" "roce92" "roce83" "roce95" "roce94")
# server_list=("node1" "node2" "node3" "node4" "node5" "node6" "node7" "node8" \
#     "node25" "node9" "node11" "node12" "node13" "node14" "node15" "node16")
ant_list=(64 64 64 128 128)
ue_list=(8 16 32 16 32)

phy_core_num=$(cat ${HYDRA_SYSTEM_CONFIG_JSON} | jq '.phy_core_num')
core_offset=$(cat ${HYDRA_SYSTEM_CONFIG_JSON} | jq '.core_offset')
rx_thread_num=$(cat ${HYDRA_SYSTEM_CONFIG_JSON} | jq '.rx_thread_num')
tx_thread_num=$(cat ${HYDRA_SYSTEM_CONFIG_JSON} | jq '.tx_thread_num')
core_left=$(( ${phy_core_num}-${core_offset}-1 ))
# total_server_num=${#server_list[@]}
total_server_num=$1

function abort_detect()
{
    res=$(cat /tmp/hydra/log_*.txt | grep "Aborted")
    if [ "$?" != "0" ]; then
        abort=0
    else
        abort=1
    fi
}

function error_detect()
{
    res=$(cat /tmp/hydra/log_*.txt | grep "error traceback")
    if [ "$?" != "0" ]; then
        error=0
    else
        error=1
    fi
}

function get_error_slot()
{
    error_slot=100000
    # for i in ${!server_list[@]}; do
    for (( i=0; i<${total_server_num}; i++ )) do
        server=${server_list[$i]}
        res=$(cat /tmp/hydra/log_${server}.txt | grep "frame window" | head -n 1)
        res=${res##*(}
        res=${res%% +*}
        if [ ${res} -lt ${error_slot} ]; then
            error_slot=${res}
        fi
    done
}

function detect_bottleneck()
{
    # Detect FFT bottleneck
    IFS=$'\n'
    bar=$(( ${error_slot}+40 ))
    bottleneck_reason=0
    res=$(cat /tmp/hydra/log_*.txt | grep "FFT Thread [0-9]* error traceback")
    for line in ${res}; do
        frameStr=${line##*frame }
        frameStr=${frameStr%%, symbol*}
        if [ ${frameStr} -le ${bar} ]; then
            echo "FFT bottlenecked!"
            echo ${line}
            bottleneck_reason=3
            recvStr=${line##*recv }
            recvStr=${recvStr%%)*}
            if [ "${recvStr}" == "0" ]; then
                echo "But Time IQ packet lost!"
                bottleneck_reason=1
            fi
            return
        fi
    done
    # Detect FFT TX bottleneck
    res=$(cat /tmp/hydra/log_*.txt | grep "TX error traceback")
    for line in ${res}; do
        frameStr=${line##*fft (frame }
        frameStr=${frameStr%% symbol*}
        if [ ${frameStr} -le ${bar} ]; then
            echo "FFT TX bottlenecked!"
            echo ${line}
            bottleneck_reason=2
            return
        fi
    done
    # Detect ZF bottleneck
    res=$(cat /tmp/hydra/log_*.txt | grep "ZF Thread [0-9]* error traceback")
    for line in ${res}; do
        frameStr=${line##*frame }
        frameStr=${frameStr%%, recv*}
        if [ ${frameStr} -le ${bar} ]; then
            echo "ZF bottlenecked!"
            echo ${line}
            bottleneck_reason=4
            recvStr=${line##*recv }
            recvStr=${recvStr%% (*}
            if [ "${recvStr}" == "0" ]; then
                echo "But Freq IQ packet lost!"
                bottleneck_reason=1
            fi
            return
        fi
    done
    # Detect ZF TX bottleneck
    res=$(cat /tmp/hydra/log_*.txt | grep "TX error traceback")
    for line in ${res}; do
        frameStr=${line##*zf (frame }
        frameStr=${frameStr%%), demul*}
        if [ ${frameStr} -le ${bar} ]; then
            echo "ZF TX bottlenecked!"
            echo ${line}
            bottleneck_reason=2
            return
        fi
    done
    # Detect Demul bottleneck
    res=$(cat /tmp/hydra/log_*.txt | grep "Demul Thread [0-9]* error traceback")
    for line in ${res}; do
        frameStr=${line##*frame }
        frameStr=${frameStr%%, symbol*}
        if [ ${frameStr} -le ${bar} ]; then
            echo "Demul bottlenecked!"
            echo ${line}
            bottleneck_reason=5
            recvStr=${line##*recv }
            recvStr=${recvStr%% and*}
            if [ "${recvStr}" == "0" ]; then
                echo "But ZF packet lost!"
                bottleneck_reason=1
            fi
            recvStr=${line##*and }
            recvStr=${recvStr%%)*}
            if [ "${recvStr}" == "0" ]; then
                echo "But Freq IQ packet lost!"
                bottleneck_reason=1
            fi
            return
        fi
    done
    # Detect Demul TX bottleneck
    res=$(cat /tmp/hydra/log_*.txt | grep "TX error traceback")
    for line in ${res}; do
        frameStr=${line##*demul (frame }
        frameStr=${frameStr%% symbol*}
        if [ ${frameStr} -le ${bar} ]; then
            echo "Demul TX bottlenecked!"
            echo ${line}
            bottleneck_reason=2
            return
        fi
    done
    # Detect Decode bottleneck
    res=$(cat /tmp/hydra/log_*.txt | grep "Decode Thread [0-9]* error traceback")
    for line in ${res}; do
        frameStr=${line##*frame }
        frameStr=${frameStr%%, idx*}
        if [ ${frameStr} -le ${bar} ]; then
            echo "Decode bottlenecked!"
            echo ${line}
            bottleneck_reason=6
            recvStr=${line##*recv }
            recvStr=${recvStr%%)*}
            if [ "${recvStr}" == "0" ]; then
                echo "But Demul packet lost!"
                bottleneck_reason=1
            fi
            return
        fi
    done
}

function verify_core_num()
{
    fft_thread_num_total=$1
    zf_thread_num_total=$2
    demul_thread_num_total=$3
    decode_thread_per_server=$4
    rx_thread_num=$5
    tx_thread_num=$6
    ant_num=$7
    valid=1
    if [ ${fft_thread_num_total} -lt ${total_server_num} ]; then
        valid=3
        return
    fi
    if [ ${zf_thread_num_total} -lt ${total_server_num} ]; then
        valid=4
        return
    fi
    if [ ${demul_thread_num_total} -lt ${total_server_num} ]; then
        valid=5
        return
    fi
    if [ ${decode_thread_per_server} -eq 0 ]; then
        valid=6
        return
    fi
    if [ ${rx_thread_num} -eq 0 ]; then
        valid=7
        return
    fi
    if [ ${tx_thread_num} -eq 0 ]; then
        valid=8
        return
    fi
    thread_num_total=$(( ${fft_thread_num_total}+${zf_thread_num_total}+${demul_thread_num_total}+${decode_thread_per_server}*${total_server_num} ))
    core_left_exclude_txrx=$(( ${core_left}-${rx_thread_num}-${tx_thread_num} ))
    core_left_exclude_txrx=$(( ${core_left_exclude_txrx}*${total_server_num}*2 ))
    if [ "${core_left_exclude_txrx}" -lt ${thread_num_total} ]; then
        valid=0
        return
    fi
}

function create_template_json()
{
    ant_num=$1
    ue_num=$2
    rx_thread_num=$3
    tx_thread_num=$4
    ofdm_data_num=1200
    if [ "${ue_num}" == "32" ]; then
        ofdm_data_num=1216
    fi
    cat ${HYDRA_SYSTEM_CONFIG_JSON} | jq --argjson ant ${ant_num} '.antenna_num=$ant' | \
        jq --argjson ue ${ue_num} '.ue_num=$ue' | \
        jq --argjson ue ${ue_num} '.user_level_list[0]=$ue' | \
        jq --argjson data ${ofdm_data_num} '.ofdm_data_num=$data' | \
        jq --argjson rx ${rx_thread_num} '.rx_thread_num=$rx' | \
        jq --argjson tx ${tx_thread_num} '.tx_thread_num=$tx' > tmp.json
    cp tmp.json ${HYDRA_SYSTEM_CONFIG_JSON}
}

function create_deploy_json()
{
    fft_thread_num_total=$1
    zf_thread_num_total=$2
    demul_thread_num_total=$3
    decode_thread_per_server=$4
    ant_num=$5
    for (( i=0; i<${total_server_num}; i++ )) do
        fft_thread_num_list[$i]=0
        zf_thread_num_list[$i]=0
        demul_thread_num_list[$i]=0
    done
    server_iter=0
    for (( i=0; i<${fft_thread_num_total}; i++ )) do
        fft_thread_num_list[${server_iter}]=$(( ${fft_thread_num_list[${server_iter}]}+1 ))
        server_iter=$(( ${server_iter}+1 ))
        if [ "${server_iter}" == "${total_server_num}" ]; then
            server_iter=0
        fi
    done
    for (( i=0; i<${zf_thread_num_total}; i++ )) do
        zf_thread_num_list[${server_iter}]=$(( ${zf_thread_num_list[${server_iter}]}+1 ))
        server_iter=$(( ${server_iter}+1 ))
        if [ "${server_iter}" == "${total_server_num}" ]; then
            server_iter=0
        fi
    done
    for (( i=0; i<${demul_thread_num_total}; i++ )) do
        demul_thread_num_list[${server_iter}]=$(( ${demul_thread_num_list[${server_iter}]}+1 ))
        server_iter=$(( ${server_iter}+1 ))
        if [ "${server_iter}" == "${total_server_num}" ]; then
            server_iter=0
        fi
    done
    for (( i=0; i<${total_server_num}; i++ )) do
        cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i $i --argjson num ${fft_thread_num_list[$i]} \
            '.num_fft_workers[$i]=$num' > tmp.json
        cp tmp.json ${HYDRA_SERVER_DEPLOY_JSON}
    done
    for (( i=0; i<${total_server_num}; i++ )) do
        cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i $i --argjson num ${zf_thread_num_list[$i]} \
            '.num_zf_workers[$i]=$num' > tmp.json
        cp tmp.json ${HYDRA_SERVER_DEPLOY_JSON}
    done
    for (( i=0; i<${total_server_num}; i++ )) do
        cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i $i --argjson num ${demul_thread_num_list[$i]} \
            '.num_demul_workers[$i]=$num' > tmp.json
        cp tmp.json ${HYDRA_SERVER_DEPLOY_JSON}
    done
    for (( i=0; i<${total_server_num}; i++ )) do
        cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i $i --argjson num ${decode_thread_per_server} \
            '.num_decode_workers[$i]=$num' > tmp.json
        cp tmp.json ${HYDRA_SERVER_DEPLOY_JSON}
    done
}

# bigstation_template_template=${hydra_root_dir}/config/template_cloudlab/template_cloudlab_bigstation_ul.json
# bigstation_deploy_template=${hydra_root_dir}/config/deploy_cloudlab/deploy_cloudlab_bigstation_ul.json
bigstation_template_template=${hydra_root_dir}/config/template_msr_bigstation/template_msr_${total_server_num}.json
bigstation_deploy_template=${hydra_root_dir}/config/deploy_msr_bigstation/deploy_msr_${total_server_num}.json

for i in ${!ant_list[@]}; do
    cur_ant=${ant_list[$i]}
    cur_ue=${ue_list[$i]}
    echo "Test setup ant ${cur_ant} ue ${cur_ue}"
    # cp ${bigstation_template_template} ${hydra_root_dir}/config/template_cloudlab/template_cloudlab_bigstation_${cur_ant}_${cur_ue}_ul.json
    # HYDRA_SYSTEM_CONFIG_JSON=config/template_cloudlab/template_cloudlab_bigstation_${cur_ant}_${cur_ue}_ul.json
    # cp ${bigstation_deploy_template} ${hydra_root_dir}/config/deploy_cloudlab/deploy_cloudlab_bigstation_${cur_ant}_${cur_ue}_ul.json
    # HYDRA_SERVER_DEPLOY_JSON=config/deploy_cloudlab/deploy_cloudlab_bigstation_${cur_ant}_${cur_ue}_ul.json
    cp ${bigstation_template_template} ${hydra_root_dir}/config/template_msr_bigstation/template_msr_${cur_ant}_${cur_ue}_${total_server_num}.json
    HYDRA_SYSTEM_CONFIG_JSON=config/template_msr_bigstation/template_msr_${cur_ant}_${cur_ue}_${total_server_num}.json
    cp ${bigstation_deploy_template} ${hydra_root_dir}/config/deploy_msr_bigstation/deploy_msr_${cur_ant}_${cur_ue}_${total_server_num}.json
    HYDRA_SERVER_DEPLOY_JSON=config/deploy_msr_bigstation/deploy_msr_${cur_ant}_${cur_ue}_${total_server_num}.json
    cat ${hydra_master_config_json} | jq --arg template ${HYDRA_SYSTEM_CONFIG_JSON} '.hydra_system_config_json=$template' | \
        jq --arg deploy ${HYDRA_SERVER_DEPLOY_JSON} '.hydra_server_deploy_json=$deploy' > tmp.json
    cp tmp.json ${hydra_master_config_json}
    cur_rx_thread_num=4
    cur_tx_thread_num=2
    cur_fft_thread_num_total=$(( ${total_server_num}*6 ))
    cur_zf_thread_num_total=$(( ${total_server_num}*2 ))
    cur_demul_thread_num_total=$(( ${total_server_num}*10 ))
    cur_decode_thread_num_per_server=10
    consecutive_error_num=0
    fft_done=0
    zf_done=0
    demul_done=0
    decode_done=0
    work=0
    while [ ${consecutive_error_num} -lt 5 ] && [ ${work} -eq 0 ]; do
        echo "Verify ant ${cur_ant} ue ${cur_ue} rx ${cur_rx_thread_num}" \
            "tx ${cur_tx_thread_num} fft ${cur_fft_thread_num_total}" \
            "zf ${cur_zf_thread_num_total} demul ${cur_demul_thread_num_total}" \
            "decode ${cur_decode_thread_num_per_server}"
        verify_core_num ${cur_fft_thread_num_total} ${cur_zf_thread_num_total} \
            ${cur_demul_thread_num_total} ${cur_decode_thread_num_per_server} \
            ${cur_rx_thread_num} ${cur_tx_thread_num} ${cur_ant}
        echo "${valid} ${total_server_num}"
        if [ "${valid}" == "0" ]; then
            echo "Verify unsuccessful"
            if [ "${fft_done}" == "0" ]; then
                if [ ${cur_fft_thread_num_total} -gt $(( ${total_server_num}*2 )) ]; then
                    cur_fft_thread_num_total=$(( ${cur_fft_thread_num_total}-${total_server_num}*2 ))
                    continue
                fi
            fi
            if [ "${zf_done}" == "0" ]; then
                if [ ${cur_zf_thread_num_total} -gt $(( ${total_server_num}*2 )) ]; then
                    cur_zf_thread_num_total=$(( ${cur_zf_thread_num_total}-${total_server_num}*2 ))
                    continue
                fi
            fi
            if [ "${demul_done}" == "0" ]; then
                if [ ${cur_demul_thread_num_total} -gt $(( ${total_server_num}*2 )) ]; then
                    cur_demul_thread_num_total=$(( ${cur_demul_thread_num_total}-${total_server_num}*2 ))
                    continue
                fi
            fi
            if [ "${decode_done}" == "0" ]; then
                if [ ${cur_decode_thread_num_per_server} -gt 2 ]; then
                    cur_decode_thread_num_per_server=$(( ${cur_decode_thread_num_per_server}-2 ))
                    continue
                fi
            fi
            # Exit
            work=2
        elif [ "${valid}" == "1" ]; then
            echo "Verify successful"
            runtime_error=0
            create_template_json ${cur_ant} ${cur_ue} ${cur_rx_thread_num} ${cur_tx_thread_num}
            create_deploy_json ${cur_fft_thread_num_total} ${cur_zf_thread_num_total} \
                ${cur_demul_thread_num_total} ${cur_decode_thread_num_per_server} ${cur_ant}
            echo "Testing ant ${cur_ant} ue ${cur_ue} rx ${cur_rx_thread_num}" \
                "tx ${cur_tx_thread_num} fft ${cur_fft_thread_num_total}" \
                "zf ${cur_zf_thread_num_total} demul ${cur_demul_thread_num_total}" \
                "decode ${cur_decode_thread_num_per_server}"
            ${hydra_root_dir}/scripts/control/run_all.sh -x || runtime_error=1
            if [ "${runtime_error}" == "1" ]; then
                consecutive_error_num=$(( ${consecutive_error_num}+1 ))
                echo "Runtime error for ant ${cur_ant} ue ${cur_ue} rx ${cur_rx_thread_num}" \
                    "tx ${cur_tx_thread_num} fft ${cur_fft_thread_num_total}" \
                    "zf ${cur_zf_thread_num_total} demul ${cur_demul_thread_num_total}" \
                    "decode ${cur_decode_thread_num_per_server}"
                continue
            fi
            abort_detect
            if [ "${abort}" == "1" ]; then
                echo "Runtime abort for ant ${cur_ant} ue ${cur_ue} rx ${cur_rx_thread_num}" \
                    "tx ${cur_tx_thread_num} fft ${cur_fft_thread_num_total}" \
                    "zf ${cur_zf_thread_num_total} demul ${cur_demul_thread_num_total}" \
                    "decode ${cur_decode_thread_num_per_server}"
                consecutive_error_num=$(( ${consecutive_error_num}+1 ))
                continue
            fi
            consecutive_error_num=0
            error_detect
            if [ "${error}" == "1" ]; then
                get_error_slot
                detect_bottleneck
                if [ "${bottleneck_reason}" == "1" ]; then
                    cur_rx_thread_num=$(( ${cur_rx_thread_num}+1 ))
                    continue
                elif [ "${bottleneck_reason}" == "2" ]; then
                    cur_tx_thread_num=$(( ${cur_tx_thread_num}+1 ))
                    continue
                elif [ "${bottleneck_reason}" == "3" ]; then
                    cur_fft_thread_num_total=$(( ${cur_fft_thread_num_total}+${total_server_num}*2 ))
                    fft_done=1
                    continue
                elif [ "${bottleneck_reason}" == "4" ]; then
                    cur_zf_thread_num_total=$(( ${cur_zf_thread_num_total}+${total_server_num}*2 ))
                    zf_done=1
                    continue
                elif [ "${bottleneck_reason}" == "5" ]; then
                    cur_demul_thread_num_total=$(( ${cur_demul_thread_num_total}+${total_server_num}*2 ))
                    demul_done=1
                    continue
                elif [ "${bottleneck_reason}" == "6" ]; then
                    cur_decode_thread_num_per_server=$(( ${cur_decode_thread_num_per_server}+2 ))
                    decode_done=1
                    continue
                fi
            else
                echo "Run successful"
                work=1
            fi
        fi
    done
    if [ "${work}" == "2" ]; then
        echo "Failed to run ant ${cur_ant} ue ${cur_ue}"
        create_template_json ${cur_ant} ${cur_ue} 0 0
        create_deploy_json ${cur_fft_thread_num_total} ${cur_zf_thread_num_total} \
            ${cur_demul_thread_num_total} ${cur_decode_thread_num_per_server} ${cur_ant}
        continue
    fi

    # Reducing the number of FFT cores
    work=1
    consecutive_error_num=0
    while [ "${work}" == "1" ] && [ ${consecutive_error_num} -lt 5 ]; do
        try_fft_thread_num_total=$(( ${cur_fft_thread_num_total}-${total_server_num}*2 ))
        if [ ${try_fft_thread_num_total} -ge $(( ${total_server_num}*2 )) ]; then
            runtime_error=0
            create_template_json ${cur_ant} ${cur_ue} ${cur_rx_thread_num} ${cur_tx_thread_num}
            create_deploy_json ${try_fft_thread_num_total} ${cur_zf_thread_num_total} \
                ${cur_demul_thread_num_total} ${cur_decode_thread_num_per_server} ${cur_ant}
            echo "Testing ant ${cur_ant} ue ${cur_ue} rx ${cur_rx_thread_num}" \
                "tx ${cur_tx_thread_num} fft ${try_fft_thread_num_total}" \
                "zf ${cur_zf_thread_num_total} demul ${cur_demul_thread_num_total}" \
                "decode ${cur_decode_thread_num_per_server}"
            ${hydra_root_dir}/scripts/control/run_all.sh -x || runtime_error=1
            if [ "${runtime_error}" == "1" ]; then
                consecutive_error_num=$(( ${consecutive_error_num}+1 ))
                echo "Runtime error for ant ${cur_ant} ue ${cur_ue} rx ${cur_rx_thread_num}" \
                    "tx ${cur_tx_thread_num} fft ${try_fft_thread_num_total}" \
                    "zf ${cur_zf_thread_num_total} demul ${cur_demul_thread_num_total}" \
                    "decode ${cur_decode_thread_num_per_server}"
                continue
            fi
            abort_detect
            if [ "${abort}" == "1" ]; then
                echo "Runtime abort for ant ${cur_ant} ue ${cur_ue} rx ${cur_rx_thread_num}" \
                    "tx ${cur_tx_thread_num} fft ${try_fft_thread_num_total}" \
                    "zf ${cur_zf_thread_num_total} demul ${cur_demul_thread_num_total}" \
                    "decode ${cur_decode_thread_num_per_server}"
                consecutive_error_num=$(( ${consecutive_error_num}+1 ))
                continue
            fi
            consecutive_error_num=0
            error_detect
            if [ "${error}" == "1" ]; then
                work=0
            else
                cur_fft_thread_num_total=${try_fft_thread_num_total}
                echo "Run successful"
            fi
        else
            work=0
        fi
    done

    # Reduce the zf worker number
    work=1
    consecutive_error_num=0
    while [ "${work}" == "1" ] && [ ${consecutive_error_num} -lt 5 ]; do
        try_zf_thread_num_total=$(( ${cur_zf_thread_num_total}-${total_server_num}*2 ))
        if [ ${try_zf_thread_num_total} -ge $(( ${total_server_num}*2 )) ]; then
            runtime_error=0
            create_template_json ${cur_ant} ${cur_ue} ${cur_rx_thread_num} ${cur_tx_thread_num}
            create_deploy_json ${cur_fft_thread_num_total} ${try_zf_thread_num_total} \
                ${cur_demul_thread_num_total} ${cur_decode_thread_num_per_server} ${cur_ant}
            echo "Testing ant ${cur_ant} ue ${cur_ue} rx ${cur_rx_thread_num}" \
                "tx ${cur_tx_thread_num} fft ${cur_fft_thread_num_total}" \
                "zf ${try_zf_thread_num_total} demul ${cur_demul_thread_num_total}" \
                "decode ${cur_decode_thread_num_per_server}"
            ${hydra_root_dir}/scripts/control/run_all.sh -x || runtime_error=1
            if [ "${runtime_error}" == "1" ]; then
                consecutive_error_num=$(( ${consecutive_error_num}+1 ))
                echo "Runtime error for ant ${cur_ant} ue ${cur_ue} rx ${cur_rx_thread_num}" \
                    "tx ${cur_tx_thread_num} fft ${cur_fft_thread_num_total}" \
                    "zf ${try_zf_thread_num_total} demul ${cur_demul_thread_num_total}" \
                    "decode ${cur_decode_thread_num_per_server}"
                continue
            fi
            abort_detect
            if [ "${abort}" == "1" ]; then
                echo "Runtime abort for ant ${cur_ant} ue ${cur_ue} rx ${cur_rx_thread_num}" \
                    "tx ${cur_tx_thread_num} fft ${cur_fft_thread_num_total}" \
                    "zf ${try_zf_thread_num_total} demul ${cur_demul_thread_num_total}" \
                    "decode ${cur_decode_thread_num_per_server}"
                consecutive_error_num=$(( ${consecutive_error_num}+1 ))
                continue
            fi
            consecutive_error_num=0
            error_detect
            if [ "${error}" == "1" ]; then
                work=0
            else
                cur_zf_thread_num_total=${try_zf_thread_num_total}
                echo "Run successful"
            fi
        else
            work=0
        fi
    done

    # Reduce the demul worker number
    work=1
    consecutive_error_num=0
    while [ "${work}" == "1" ] && [ ${consecutive_error_num} -lt 5 ]; do
        try_demul_thread_num_total=$(( ${cur_demul_thread_num_total}-${total_server_num}*2 ))
        if [ ${try_demul_thread_num_total} -ge $(( ${total_server_num}*2 )) ]; then
            runtime_error=0
            create_template_json ${cur_ant} ${cur_ue} ${cur_rx_thread_num} ${cur_tx_thread_num}
            create_deploy_json ${cur_fft_thread_num_total} ${cur_zf_thread_num_total} \
                ${try_demul_thread_num_total} ${cur_decode_thread_num_per_server} ${cur_ant}
            echo "Testing ant ${cur_ant} ue ${cur_ue} rx ${cur_rx_thread_num}" \
                "tx ${cur_tx_thread_num} fft ${cur_fft_thread_num_total}" \
                "zf ${cur_zf_thread_num_total} demul ${try_demul_thread_num_total}" \
                "decode ${cur_decode_thread_num_per_server}"
            ${hydra_root_dir}/scripts/control/run_all.sh -x || runtime_error=1
            if [ "${runtime_error}" == "1" ]; then
                consecutive_error_num=$(( ${consecutive_error_num}+1 ))
                echo "Runtime error for ant ${cur_ant} ue ${cur_ue} rx ${cur_rx_thread_num}" \
                    "tx ${cur_tx_thread_num} fft ${cur_fft_thread_num_total}" \
                    "zf ${cur_zf_thread_num_total} demul ${try_demul_thread_num_total}" \
                    "decode ${cur_decode_thread_num_per_server}"
                continue
            fi
            abort_detect
            if [ "${abort}" == "1" ]; then
                echo "Runtime abort for ant ${cur_ant} ue ${cur_ue} rx ${cur_rx_thread_num}" \
                    "tx ${cur_tx_thread_num} fft ${cur_fft_thread_num_total}" \
                    "zf ${cur_zf_thread_num_total} demul ${try_demul_thread_num_total}" \
                    "decode ${cur_decode_thread_num_per_server}"
                consecutive_error_num=$(( ${consecutive_error_num}+1 ))
                continue
            fi
            consecutive_error_num=0
            error_detect
            if [ "${error}" == "1" ]; then
                work=0
            else
                cur_demul_thread_num_total=${try_demul_thread_num_total}
                echo "Run successful"
            fi
        else
            work=0
        fi
    done

    # Reduce the decode worker number
    work=1
    consecutive_error_num=0
    while [ "${work}" == "1" ] && [ ${consecutive_error_num} -lt 5 ]; do
        try_decode_thread_num_per_server=$(( ${cur_decode_thread_num_per_server}-2 ))
        if [ ${try_decode_thread_num_per_server} -ge 2 ]; then
            runtime_error=0
            create_template_json ${cur_ant} ${cur_ue} ${cur_rx_thread_num} ${cur_tx_thread_num}
            create_deploy_json ${cur_fft_thread_num_total} ${cur_zf_thread_num_total} \
                ${cur_demul_thread_num_total} ${try_decode_thread_num_per_server} ${cur_ant}
            echo "Testing ant ${cur_ant} ue ${cur_ue} rx ${cur_rx_thread_num}" \
                "tx ${cur_tx_thread_num} fft ${cur_fft_thread_num_total}" \
                "zf ${cur_zf_thread_num_total} demul ${cur_demul_thread_num_total}" \
                "decode ${try_decode_thread_num_per_server}"
            ${hydra_root_dir}/scripts/control/run_all.sh -x || runtime_error=1
            if [ "${runtime_error}" == "1" ]; then
                consecutive_error_num=$(( ${consecutive_error_num}+1 ))
                echo "Runtime error for ant ${cur_ant} ue ${cur_ue} rx ${cur_rx_thread_num}" \
                    "tx ${cur_tx_thread_num} fft ${cur_fft_thread_num_total}" \
                    "zf ${cur_zf_thread_num_total} demul ${cur_demul_thread_num_total}" \
                    "decode ${try_decode_thread_num_per_server}"
                continue
            fi
            abort_detect
            if [ "${abort}" == "1" ]; then
                echo "Runtime abort for ant ${cur_ant} ue ${cur_ue} rx ${cur_rx_thread_num}" \
                    "tx ${cur_tx_thread_num} fft ${cur_fft_thread_num_total}" \
                    "zf ${cur_zf_thread_num_total} demul ${cur_demul_thread_num_total}" \
                    "decode ${try_decode_thread_num_per_server}"
                consecutive_error_num=$(( ${consecutive_error_num}+1 ))
                continue
            fi
            consecutive_error_num=0
            error_detect
            if [ "${error}" == "1" ]; then
                work=0
            else
                cur_decode_thread_num_per_server=${try_decode_thread_num_per_server}
                echo "Run successful"
            fi
        else
            work=0
        fi
    done

    # Reduce the rx thread number
    work=1
    consecutive_error_num=0
    while [ "${work}" == "1" ] && [ ${consecutive_error_num} -lt 5 ]; do
        try_rx_thread_num=$(( ${cur_rx_thread_num}-1 ))
        if [ ${try_rx_thread_num} -gt 0 ]; then
            runtime_error=0
            create_template_json ${cur_ant} ${cur_ue} ${try_rx_thread_num} ${cur_tx_thread_num}
            create_deploy_json ${cur_fft_thread_num_total} ${cur_zf_thread_num_total} \
                ${cur_demul_thread_num_total} ${cur_decode_thread_num_per_server} ${cur_ant}
            echo "Testing ant ${cur_ant} ue ${cur_ue} rx ${try_rx_thread_num}" \
                "tx ${cur_tx_thread_num} fft ${cur_fft_thread_num_total}" \
                "zf ${cur_zf_thread_num_total} demul ${cur_demul_thread_num_total}" \
                "decode ${cur_decode_thread_num_per_server}"
            ${hydra_root_dir}/scripts/control/run_all.sh -x || runtime_error=1
            if [ "${runtime_error}" == "1" ]; then
                consecutive_error_num=$(( ${consecutive_error_num}+1 ))
                echo "Runtime error for ant ${cur_ant} ue ${cur_ue} rx ${try_rx_thread_num}" \
                    "tx ${cur_tx_thread_num} fft ${cur_fft_thread_num_total}" \
                    "zf ${cur_zf_thread_num_total} demul ${cur_demul_thread_num_total}" \
                    "decode ${cur_decode_thread_num_per_server}"
                continue
            fi
            abort_detect
            if [ "${abort}" == "1" ]; then
                echo "Runtime abort for ant ${cur_ant} ue ${cur_ue} rx ${try_rx_thread_num}" \
                    "tx ${cur_tx_thread_num} fft ${cur_fft_thread_num_total}" \
                    "zf ${cur_zf_thread_num_total} demul ${cur_demul_thread_num_total}" \
                    "decode ${cur_decode_thread_num_per_server}"
                consecutive_error_num=$(( ${consecutive_error_num}+1 ))
                continue
            fi
            consecutive_error_num=0
            error_detect
            if [ "${error}" == "1" ]; then
                work=0
            else
                cur_rx_thread_num=${try_rx_thread_num}
                echo "Run successful"
            fi
        else
            work=0
        fi
    done

    # Reduce the tx thread number
    work=1
    consecutive_error_num=0
    while [ "${work}" == "1" ] && [ ${consecutive_error_num} -lt 5 ]; do
        try_tx_thread_num=$(( ${cur_tx_thread_num}-1 ))
        if [ ${try_tx_thread_num} -gt 0 ]; then
            runtime_error=0
            create_template_json ${cur_ant} ${cur_ue} ${cur_rx_thread_num} ${try_tx_thread_num}
            create_deploy_json ${cur_fft_thread_num_total} ${cur_zf_thread_num_total} \
                ${cur_demul_thread_num_total} ${cur_decode_thread_num_per_server} ${cur_ant}
            echo "Testing ant ${cur_ant} ue ${cur_ue} rx ${cur_rx_thread_num}" \
                "tx ${try_tx_thread_num} fft ${cur_fft_thread_num_total}" \
                "zf ${cur_zf_thread_num_total} demul ${cur_demul_thread_num_total}" \
                "decode ${cur_decode_thread_num_per_server}"
            ${hydra_root_dir}/scripts/control/run_all.sh -x || runtime_error=1
            if [ "${runtime_error}" == "1" ]; then
                consecutive_error_num=$(( ${consecutive_error_num}+1 ))
                echo "Runtime error for ant ${cur_ant} ue ${cur_ue} rx ${cur_rx_thread_num}" \
                    "tx ${try_tx_thread_num} fft ${cur_fft_thread_num_total}" \
                    "zf ${cur_zf_thread_num_total} demul ${cur_demul_thread_num_total}" \
                    "decode ${cur_decode_thread_num_per_server}"
                continue
            fi
            abort_detect
            if [ "${abort}" == "1" ]; then
                echo "Runtime abort for ant ${cur_ant} ue ${cur_ue} rx ${cur_rx_thread_num}" \
                    "tx ${try_tx_thread_num} fft ${cur_fft_thread_num_total}" \
                    "zf ${cur_zf_thread_num_total} demul ${cur_demul_thread_num_total}" \
                    "decode ${cur_decode_thread_num_per_server}"
                consecutive_error_num=$(( ${consecutive_error_num}+1 ))
                continue
            fi
            consecutive_error_num=0
            error_detect
            if [ "${error}" == "1" ]; then
                work=0
            else
                cur_tx_thread_num=${try_tx_thread_num}
                echo "Run successful"
            fi
        else
            work=0
        fi
    done

    echo "Setup ant ${cur_ant} ue ${cur_ue} is rx ${cur_rx_thread_num}" \
        "tx ${cur_tx_thread_num} fft ${cur_fft_thread_num_total}" \
        "zf ${cur_zf_thread_num_total} demul ${cur_demul_thread_num_total}" \
        "decode ${cur_decode_thread_num_per_server}"
    create_template_json ${cur_ant} ${cur_ue} ${cur_rx_thread_num} ${cur_tx_thread_num}
    create_deploy_json ${cur_fft_thread_num_total} ${cur_zf_thread_num_total} \
        ${cur_demul_thread_num_total} ${cur_decode_thread_num_per_server} ${cur_ant}
done

# abort_detect
# if [ "${abort}" == "1" ]; then
#     echo "Aborted!"
#     exit
# fi
# error_detect
# if [ "${error}" == "1" ]; then
#     get_error_slot
#     detect_bottleneck
# fi