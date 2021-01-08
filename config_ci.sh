#!/bin/bash

function version { echo "$@" | awk -F. '{ printf("%d%03d%03d%03d\n", $1,$2,$3,$4); }'; }

GIT_MIN_VER="2.9"
GIT_VER=$( git --version | perl -pe '($_)=/([0-9]+([.][0-9]+)+)/' )

if [ $(version $GIT_VER) -lt $(version $GIT_MIN_VER) ]; then
      echo "Your Git version is ${GIT_VER}. Need ${GIT_MIN_VER} or higher."
fi

# configure git hooks path
git config core.hooksPath .githooks
echo configured git hooks path

