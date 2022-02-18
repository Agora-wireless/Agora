#! /bin/bash

function echored() {
  export TERM=xterm-256color
  es=`tput setaf 1`
  ee=`tput sgr0`
  echo "${es}$1${ee}"
}

function echocyan() {
  export TERM=xterm-256color
  es=`tput setaf 6`
  ee=`tput sgr0`
  echo "${es}$1${ee}"
}

function checkpkg() {
  res=$(dpkg-query -l "$1" 2> /dev/null) || :
  if [ -z "$res" ]; then
    echored "[$(hostname)] Error: Package $1 not installed"
    checkpkg_res=0
  else
    checkpkg_res=1
  fi
}

function checkret() {
  if [ "$?" != "0" ]; then
    exit
  fi
}