#!/bin/bash

function version { echo "$@" | awk -F. '{ printf("%d%03d%03d%03d\n", $1,$2,$3,$4); }'; }

GIT_MIN_VER="2.9"
GIT_VER=$( git --version | perl -pe '($_)=/([0-9]+([.][0-9]+)+)/' )

if [ $(version $GIT_VER) -lt $(version $GIT_MIN_VER) ]; then
      echo "Your Git version is ${GIT_VER}. Need ${GIT_MIN_VER} or higher."
fi

HOOKS_PATH=.git/hooks
TEMPLATE_DIR=.githooks
HOOK_NAME=""

# Purge old hooks
echo -e 'Purging old pre-push/post-checkout hooks... Continue? [y/N] \c'
read consent
if [ -z "$consent" ] || [ "$consent" == 'n' ]; then
      echo 'Exiting...'
      exit 0
fi
rm -f ${HOOKS_PATH}/{pre-push,post-checkout}

# Install new hook 
if [ "$#" == 0 ] || [ "$1" == "post-checkout" ]; then
      cp ${TEMPLATE_DIR}/templateReadmeCIBadgeHook.sh ${HOOKS_PATH}/post-checkout
      HOOK_NAME="post-checkout"
elif [ "$1" == "pre-push" ]; then
      cp ${TEMPLATE_DIR}/templateReadmeCIBadgeHook.sh ${HOOKS_PATH}/pre-push
      HOOK_NAME="pre-push"
fi

sed -i -e "1a echo 'Starting ${HOOK_NAME} hook ...'" ${HOOKS_PATH}/${HOOK_NAME}

echo "configured git ${HOOK_NAME} hook."

