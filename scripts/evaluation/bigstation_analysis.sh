#! /bin/bash

script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
hydra_root_dir=$( cd ${script_dir}/../.. >/dev/null 2>&1 && pwd )

source ${hydra_root_dir}/scripts/utils/utils.sh
source ${hydra_root_dir}/scripts/control/init_platform.sh

server_list=("roce95" "roce94" "roce91" "roce92" "roce83")

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
    for i in ${!server_list[@]}; do
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
    bar=$(( ${error_slot}+50 ))
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

abort_detect
if [ "${abort}" == "1" ]; then
    echo "Abort detected"
    exit
fi
error_detect
if [ "${error}" == "1" ]; then
    get_error_slot
    detect_bottleneck
fi