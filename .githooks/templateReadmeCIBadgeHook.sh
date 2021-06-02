#!/bin/sh

CI_BUILD_STATUS_BADGE_OLD=$(grep -i "build status" README.md)
BRANCH_NAME=$(git rev-parse --abbrev-ref HEAD)
CI_BUILD_STATUS_BADGE="[![Build Status](https://falcon.ecg.rice.edu:443/buildStatus/icon?job=github_public_agora%2F${BRANCH_NAME})](https://falcon.ecg.rice.edu:443/job/github_public_agora/job/${BRANCH_NAME}/)"
echo $BRANCH_NAME
echo $CI_BUILD_STATUS_BADGE

if [ "${CI_BUILD_STATUS_BADGE_OLD}" == "${CI_BUILD_STATUS_BADGE}" ]; then
    exit 0
fi

sed -i -e "/Build Status/ c $CI_BUILD_STATUS_BADGE" README.md
git add README.md
git commit -m "update CI build badge"

exit 0

