#!/usr/bin/env bash
new_name=$(python full.py $1)
cd ../calibration/scripts
./run_all.sh > ../../$new_name"_calibration_params.txt"
