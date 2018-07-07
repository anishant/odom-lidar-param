#! usr/bin/env python
import os
import sys
from datetime import datetime
import pandas as pd
import re

p = re.compile('^.*\/(.*).rosbag$')
bagname = p.search(sys.argv[1]).group(1)

generated_folder = bagname+'_'+str(datetime.now())
base = os.path.dirname(os.path.abspath(__file__))
os.mkdir(base+'/'+generated_folder+'/')
gen_path = base+'/'+generated_folder
os.mkdir(gen_path+'/dirty_csv/')

# Generate CSV from raw data and put into dirty_csv

from subprocess import check_output

topics = ['/odom', '/scan', '/mobile_base/sensors/core']
raw_data = []
for index, topic in enumerate(topics):
    raw_data.append(check_output(['rostopic', 'echo', '-b', sys.argv[1], '-p', topic]))

with open(gen_path+'/dirty_csv/odom.csv', 'w') as csv:
    csv.write(raw_data[0])
with open(gen_path+'/dirty_csv/scan.csv', 'w') as csv:
    csv.write(raw_data[1])
with open(gen_path+'/dirty_csv/mobile_base_sensors_core.csv', 'w') as csv:
    csv.write(raw_data[2])

# Generates clean_data.csv files

def laser_clean(filename):
    df = pd.read_csv(gen_path+'/dirty_csv/'+filename+'.csv')
    df = df.drop([column for column in df.columns if 'intensities' in column],axis=1)
    columns = ['field.header.frame_id', 'field.time_increment', 'field.scan_time']
    df.drop(columns, axis=1, inplace=True)
    df.to_csv(gen_path+'/'+'clean_'+filename+'.csv', encoding='utf-8', index=False)

def odom_clean(filename):
    df = pd.read_csv(gen_path+'/dirty_csv/'+filename+'.csv')
    df = df.drop([column for column in df.columns if 'field.twist.covariance' in column],axis=1)
    columns = ['field.header.frame_id', 'field.child_frame_id', 'field.twist.twist.linear.z', 'field.twist.twist.angular.x', 'field.twist.twist.angular.y']
    df.drop(columns, axis=1, inplace=True)
    df.to_csv(gen_path+'/'+'clean_'+filename+'.csv', encoding='utf-8', index=False)

def core_clean(filename):
    df = pd.read_csv(gen_path+'/dirty_csv/'+filename+'.csv')
    df = df.drop([column for column in df.columns if 'bottom' in column],axis=1)
    df = df.drop([column for column in df.columns if 'current' in column],axis=1)
    df = df.drop([column for column in df.columns if 'analog_input' in column],axis=1)
    columns = ['field.header.frame_id', 'field.bumper', 'field.wheel_drop', 'field.cliff', 'field.buttons', 'field.charger', 'field.battery', 'field.digital_input']
    df.drop(columns, axis=1, inplace=True)
    df.to_csv(gen_path+'/'+'clean_'+filename+'.csv', encoding='utf-8', index=False)

def complete_clean():
    laser_clean('scan')
    odom_clean('odom')
    core_clean('mobile_base_sensors_core')

complete_clean()

# Modify and run matlab script on clean CSV files

import shutil
import re
shutil.copy2(base+'/lavish.m', gen_path)
with open(base+'/lavish.m', 'r') as matlab_file:
    all_text = matlab_file.read()
    all_text = re.sub('uniquestringtoreplace', gen_path+'/', all_text)
    with open(gen_path+'/lavish.m', 'w') as new_file:
        new_file.write(all_text)

from subprocess import call

run_string = 'run("'+gen_path+'/lavish.m");exit;'
call(['matlab', '-nodisplay', '-nosplash', '-nodesktop', '-r', run_string])


# Zip the prepared log.log file multiple times
# log.log -> log.log.bz2 -> exp1/log.log.bz2 -> fromscript.tar.gz

import bz2

def bz2zipper():
    with open(gen_path+'/clean_log.log', 'rb') as data:
        bz2_content = bz2.compress(data.read())
    
    with open(gen_path+'/log.log.bz2', 'wb') as data:
        data.write(bz2_content)

bz2zipper()

import tarfile 

def targzipper():
    os.mkdir(gen_path+'/exp1')
    os.rename(gen_path+'/log.log.bz2', gen_path+'/exp1/log.log.bz2')
    with tarfile.open(gen_path+'/fromscript.tar.gz', 'w:gz') as tar:
        tar.add(gen_path+'/exp1', arcname=os.path.basename(gen_path+'/exp1'))

targzipper()

# Clean and delete extra generated files
# Comment out line mentioning any extracted data you want to preserve

import shutil

def cleanup():
    os.remove(gen_path+'/clean_log.log')
    os.remove(gen_path+'/exp1/log.log.bz2')
    os.rmdir(gen_path+'/exp1')
    shutil.rmtree(gen_path+'/dirty_csv')
    os.remove(gen_path+'/lavish.m')
    os.remove(gen_path+'/clean_odom.csv')
    os.remove(gen_path+'/clean_scan.csv')
    os.remove(gen_path+'/clean_mobile_base_sensors_core.csv')

cleanup()
        
# Copy generated tar.gz file to calibration-master to execute
shutil.copy2(gen_path+'/fromscript.tar.gz', os.path.abspath('../calibration/data'))
shutil.move(gen_path, os.path.join(gen_path, '../'))

print bagname
