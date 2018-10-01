#!/usr/bin/env bash
new_name=$(python gen_params.py $1)
cd ../calibration/scripts
./run_all.sh |& tee ../../"$new_name"/"calibration_params.txt"
