#! /bin/bash

node_list_str=$(cat node_list)
addr_list_str=$(cat addr_list)

node_id=0
for node in ${node_list_str}
do
    node_list[${node_id}]=$node
    node_id=$(( node_id+1 ))
done
node_id=0
for addr in ${addr_list_str}
do
    addr_list[${node_id}]=$addr
    node_id=$(( node_id+1 ))
done

ssh -oStrictHostKeyChecking=no ${node_list[0]} "ssh-keygen -N \'\' -f ~/.ssh/id_rsa << y"
echo
pub_key=$(ssh -oStrictHostKeyChecking=no ${node_list[0]} "cat ~/.ssh/id_rsa.pub")

rm -f tmp_ssh_config
node_id=0
for node in ${node_list_str}
do
    echo "Host ${node}" >> tmp_ssh_config
    echo -e "\tHostName ${addr_list[${node_id}]}" >> tmp_ssh_config
    echo -e "\tUser junzhig" >> tmp_ssh_config
    node_id=$(( node_id+1 ))
done

node_id=0
for node in ${node_list_str}
do
    eval "scp -oStrictHostKeyChecking=no tmp_ssh_config ${node}:~/.ssh/config"
    eval "ssh -oStrictHostKeyChecking=no ${node} 'echo ${pub_key} >> ~/.ssh/authorized_keys'"
done