#!/bin/bash

PROCESS='flow_scrape_rf.py'
SCRAPE='/home/jakeware/flow_scrape_rf.py'

if ps ax | grep -v grep | grep ${PROCESS} > /dev/null
then
    echo "${PROCESS}: UP"
else
    echo "${PROCESS}: DOWN"
    echo "RESTARTING"
    python ${SCRAPE}
fi
