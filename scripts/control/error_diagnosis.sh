#! /bin/bash

for (( i=0; i<${hydra_rru_num}; i++ )) do
    server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i $i '.rru_servers[$i]' | tr -d '"')
    log_fn="/tmp/hydra/log_${server_name}.txt"
    diagnose_start=0
    while IFS= read -r line
    do
        if [ "$diagnose_start" == "0" ]; then
            if [ -n $(echo "$line" | grep "Worker thread ends") ]; then
                diagnose_start=1
            fi
        fi
    done < "$log_fn"
    if [ "$diagnose_start" == "0" ]; then
        echored "Error: RRU server ${server_name} did not finish normal execution"
    fi
done

pkt_loss_rru=0
bottleneck=0
success=0
for (( i=0; i<${hydra_app_num}; i++ )) do
    server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i $i '.hydra_servers[$i]' | tr -d '"')
    log_fn="/tmp/hydra/log_${server_name}.txt"
    diagnose_start=0
    pkt_msg=0
    while IFS= read -r line
    do
        if [ -n "$(echo "$line" | grep "Agora: No error detected")" ]; then
            success=1
            break
        fi
        if [ "$diagnose_start" == "0" ]; then
            if [ -n "$(echo "$line" | grep "Diagnosis start")" ]; then
                diagnose_start=1
            fi
        elif [ "$diagnose_start" == "1" ]; then
            diagnose_start=2
        elif [ "$diagnose_start" == "2" ]; then
            diagnose_start=3
        elif [ "$diagnose_start" == "3" ]; then
            diagnose_start=4
        elif [ "$diagnose_start" == "4" ]; then
            diagnose_start=5
        elif [ "$diagnose_start" == "5" ]; then
            diagnose_start=6
            if [ -n "$(echo "$line" | grep "Decoding might be")" ]; then
                pkt_msg=1
            elif [ -n "$(echo "$line" | grep "Did not receive all demod packets")" ]; then
                pkt_msg=2
            elif [ -n "$(echo "$line" | grep "It could be a combination of problems")" ]; then
                pkt_msg=3
            elif [ -n "$(echo "$line" | grep "Packet loss for data packets")" ]; then
                pkt_msg=4
                pkt_loss_rru=1
            elif [ -n "$(echo "$line" | grep "Packet loss for pilot packets")" ]; then
                pkt_msg=5
                pkt_loss_rru=1
            fi
        elif [ "$diagnose_start" == "6" ]; then
            diagnose_start=7
        elif [ "$diagnose_start" == "7" ]; then
            diagnose_start=8
        elif [ "$diagnose_start" == "8" ]; then
            diagnose_start=9
            if [ -n "$(echo "$line" | grep "No bottleneck found")" ]; then
                if [ "${pkt_msg}" == "1" ]; then
                    echored "An unknown error occurred in Hydra server ${server_name}. Please check its log for further information."
                elif [ "${pkt_msg}" == "4" ]; then
                    echored "Packet loss detected in Hydra server ${server_name}. Please check its log for further information."
                elif [ "${pkt_msg}" == "5" ]; then
                    echored "Packet loss detected in Hydra server ${server_name}. Please check its log for further information."
                fi
            elif [ -n "$(echo "$line" | grep "Decode")" ]; then
                bottleneck=1
                echored "Decoding bottleneck detected in Hydra server ${server_name}. Please check its log for further information."
            elif [ -n "$(echo "$line" | grep "Subcarrier")" ]; then
                bottleneck=1
                echored "Subcarrier bottleneck detected in Hydra server ${server_name}. Please check its log for further information."
            fi
            break
        fi
    done < "$log_fn"
    if [ "${success}" == "1" ]; then
        break
    fi
    if [ "${diagnose_start}" != "9" ]; then
        echored "Error: Log file for Hydra server ${server_name} is incomplete. Please check its log for further information."
    fi
done

if [ "${success}" == "0" ]; then
    if [ "${pkt_loss_rru}" == "0" ]; then
        if [ "${bottleneck}" == "0" ]; then
            echored "Analysis failed. Please check the log files for further information."
        fi
    fi
else
    echocyan "Hydra app completes normally."
fi