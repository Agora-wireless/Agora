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