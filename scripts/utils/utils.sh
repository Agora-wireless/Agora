#! /bin/bash

function echored() {
    es=`tput setaf 1`
    ee=`tput sgr0`
    echo "${es}$1${ee}"
}

function echocyan() {
    es=`tput setaf 6`
    ee=`tput sgr0`
    echo "${es}$1${ee}"
}

function checkpkg() {
    res=$(dpkg-query -l "$1")
    if [ -z "$res" ]; then
        echored "Error: Package $1 not installed"
        checkpkg_res=0
    else
        checkpkg_res=1
    fi
}