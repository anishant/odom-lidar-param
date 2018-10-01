#! /usr/bin/env python
import re
import os
import sys
import json
import math
import pandas as pd
import numpy as np
from datetime import datetime
from subprocess import check_output
from quat2eul import euler_from_quaternion

p = re.search('\/(.*?).bag$', sys.argv[1])
bagname = p.group(1)

generated_folder = bagname+'_'+str(datetime.now())
base = os.path.dirname(os.path.abspath(__file__))
os.mkdir(base+'/'+generated_folder+'/')
gen_path = base+'/'+generated_folder
os.mkdir(gen_path+'/dirty_csv/')

topics = ['/odom', '/scan', '/mobile_base/sensors/core']
raw_data = []
for index, topic in enumerate(topics):
    raw_data.append(check_output(['rostopic', 'echo', '-b', sys.argv[1], '-p', topic]))

with open(gen_path+'/dirty_csv/odom.csv', 'w') as csv:
    csv.write(raw_data[0])
with open(gen_path+'/dirty_csv/scan.csv', 'w') as csv:
    csv.write(raw_data[1])
with open(gen_path+'/dirty_csv/core.csv', 'w') as csv:
    csv.write(raw_data[2])

# Generates clean_data.csv files
df_odom = pd.read_csv(gen_path+'/dirty_csv/odom.csv')
df_scan = pd.read_csv(gen_path+'/dirty_csv/scan.csv')
df_core = pd.read_csv(gen_path+'/dirty_csv/core.csv')


def get_yaw(a):
    return euler_from_quaternion(a)[2]
df_odom['yaw'] = df_odom.apply(lambda row: get_yaw([row['field.pose.pose.orientation.w'], row['field.pose.pose.orientation.x'], row['field.pose.pose.orientation.y'], row['field.pose.pose.orientation.z']]), axis=1)


# Remove first two rows of scan
df_scan = df_scan.iloc[2:]
num_rows = len(df_scan.index)
df_scan.index = range(num_rows)

# Call cleanup

# Scan match
odom_indices = []
core_indices = []
for index, row_scan in df_scan.iterrows():
    odom_indices.append(list(np.where(df_odom['field.header.stamp'] <= row_scan['field.header.stamp'])[0])[-1])
    core_indices.append(list(np.where(df_core['field.header.stamp'] <= row_scan['field.header.stamp'])[0])[-1])
df_odom.drop(df_odom.index[list(set(df_odom.index)-set(odom_indices))], inplace=True)
df_core.drop(df_core.index[list(set(df_core.index)-set(core_indices))], inplace=True)


df_odom.index = range(num_rows)
df_core.index = range(num_rows)

base_time = df_core.iloc[0]['field.header.stamp']

timestamp_temp = df_scan['field.header.stamp']
timestamp_temp = timestamp_temp.apply(lambda x: ((x - base_time)*0.001))
timestamp = []
for i in range(num_rows):
    x = timestamp_temp[i]
    r = x%1000000
    y = (x-r)/1000000
    timestamp.append([y, round(r)])


left_temp = df_core['field.left_encoder']
left = pd.DataFrame(index=np.arange(num_rows), columns=['left'])
count_left = 0
max_ticks = 65535
for i in range(1,num_rows):
    if (left_temp[i-1]-left_temp[i]>=63000):
        count_left = count_left + 1
    if (left_temp[i-1]-left_temp[i]<=-63000):
        count_left = count_left -1
    left.at[i-1, 'left'] = max_ticks*count_left + left_temp[i]
left.at[num_rows-1, 'left'] = left.iloc[num_rows-2]['left']

right_temp = df_core['field.right_encoder']
right = pd.DataFrame(index=np.arange(num_rows), columns=['right'])
count_right = 0
max_ticks = 65535
for i in range(1,num_rows):
    if (right_temp[i-1]-right_temp[i]>=63000):
        count_right = count_right + 1
    if (right_temp[i-1]-right_temp[i]<=-63000):
        count_right = count_right -1
    right.at[i-1, 'right'] = max_ticks*count_right + right_temp[i]
right.at[num_rows-1, 'right'] = right.iloc[num_rows-2]['right']

temp_time = df_core['field.header.stamp'].apply(lambda x: (x - base_time)*0.001)
leftTimestamp = []
for i in range(num_rows):
    x = temp_time[i]
    r = x%1000000
    y = (x-r)/1000000
    leftTimestamp.append([y, round(r)])

rightTimestamp = leftTimestamp

ranges_head = [col for col in df_scan.columns if 'ranges' in col]

readings= df_scan[ranges_head]
df_readings = pd.DataFrame(index=np.arange(num_rows), columns=['readings'])
valid = pd.DataFrame(1, index=np.arange(num_rows), columns=ranges_head)
for col in ranges_head:
    df_readings[col] = readings[col].apply(lambda x: 0 if str(x) == 'inf' else x)
    valid[col] = readings[col].apply(lambda x: 0 if str(x) == 'inf' else 1)
valid_list = valid[ranges_head].values.tolist()

new = -math.pi+math.radians(1)
theta = []
for i in range(360):
    theta.append(new)
    new = new + math.radians(1)

import decimal

readings= df_readings[ranges_head].values.tolist()
df_leftTimestamp = pd.DataFrame(index=np.arange(num_rows), columns=["leftTimestamp"])
df_rightTimestamp = pd.DataFrame(index=np.arange(num_rows), columns=["rightTimestamp"])
df_timestamp = pd.DataFrame(index=np.arange(num_rows), columns=["timestamp"])
df_nrays = pd.DataFrame(360, index=np.arange(num_rows), columns=['nrays'])
df_min_theta = pd.DataFrame(-math.pi+math.radians(1), index=np.arange(num_rows), columns=['min_theta'])
df_max_theta = pd.DataFrame(math.pi, index=np.arange(num_rows), columns=['max_theta'])
df_theta = pd.DataFrame(index=np.arange(num_rows), columns=["theta"])
df_readings = pd.DataFrame(index=np.arange(num_rows), columns=["readings"])
df_valid = pd.DataFrame(index=np.arange(num_rows), columns=["valid"])
df_odometry = pd.DataFrame(index=np.arange(num_rows), columns=["odometry"])
df_estimate = pd.DataFrame(index=np.arange(num_rows), columns=["estimate"])
df_true_pose = pd.DataFrame(index=np.arange(num_rows), columns=["true_pose"])

for i in range(num_rows):
    df_rightTimestamp.at[i, 'rightTimestamp'] = rightTimestamp[i]
    df_leftTimestamp.at[i, 'leftTimestamp'] = leftTimestamp[i]
    df_timestamp.at[i, 'timestamp'] = timestamp[i]
    df_valid.at[i, 'valid'] = valid_list[i]
    df_readings.at[i, 'readings'] = readings[i]
    df_theta.at[i, 'theta'] = theta
    df_odometry.at[i, 'odometry'] = [0,0,0]
    df_estimate.at[i, 'estimate'] = [None, None, None]
    df_true_pose.at[i, 'true_pose'] = [None, None, None]

df_FINAL = pd.concat([df_timestamp, df_nrays, df_min_theta, df_max_theta, df_theta, df_readings, df_valid, df_odometry, df_estimate, df_true_pose, left, right, df_leftTimestamp, df_rightTimestamp], axis=1)

# df_FINAL = pd.concat([df_readings, df_timestamp], axis=1)

df_FINAL['readings'] = df_FINAL['readings'].apply(lambda x: [round(val,6) for val in x])
df_FINAL['timestamp'] = df_FINAL['timestamp'].apply(lambda x: [int(val) for val in x])
df_FINAL['leftTimestamp'] = df_FINAL['leftTimestamp'].apply(lambda x: [int(val) for val in x])
df_FINAL['rightTimestamp'] = df_FINAL['rightTimestamp'].apply(lambda x: [int(val) for val in x])

import json as json

def df_to_json(df):
    x = df.reset_index().T.to_dict().values()
    with open(gen_path+'/log.json', 'w+') as f: 
        f.write(json.dumps(x))
    return x

df_to_json(df_FINAL)

def json_to_log():
    with open(gen_path+'/log.json', 'rb') as data:
        cont = data.read()
        cont = cont[1:len(cont)-1]
        cont = re.sub(r', {"index": \d+, ',r'{',cont)
        cont = re.sub(r'{"index": \d+, ',r'{',cont)
        

    with open(gen_path+'/log.log', 'w+') as f: 
        f.write(cont)

json_to_log()

import bz2

def bz2zipper():
    with open(gen_path+'/log.log', 'rb') as data:
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
import shutil

def cleanup():
    os.remove(gen_path+'/exp1/log.log.bz2')
    os.rmdir(gen_path+'/exp1')
    shutil.rmtree(gen_path+'/dirty_csv')

cleanup()


shutil.copy2(gen_path+'/fromscript.tar.gz', os.path.abspath('../calibration/data'))
shutil.move(gen_path, os.path.join(gen_path, '../..'))

print generated_folder

# Comment out line mentioning any extracted data you want to preserve

