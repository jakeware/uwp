#!/bin/bash

# run this from pod root directory

# west 5
echo "h270_s05"
flow_plan -f /home/jakeware/wind_fields/mit/hdf/h270_s05/ -q -r 50 -d -m 800

# west 10
echo "h270_s10"
flow_plan -f /home/jakeware/wind_fields/mit/hdf/h270_s10/ -q -r 50 -d -m 800

# west 15
echo "h270_s15"
flow_plan -f /home/jakeware/wind_fields/mit/hdf/h270_s15/ -q -r 50 -d -m 800

# east 5
echo "h090_s05"
flow_plan -f /home/jakeware/wind_fields/mit/hdf/h090_s05/ -q -r 50 -d -m 800

# east 10
echo "h090_s10"
flow_plan -f /home/jakeware/wind_fields/mit/hdf/h090_s10/ -q -r 50 -d -m 800

# east 15
echo "h090_s15"
flow_plan -f /home/jakeware/wind_fields/mit/hdf/h090_s15/ -q -r 50 -d -m 800
