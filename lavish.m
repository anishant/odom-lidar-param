%% Main file
clc
clear all;
% close all;

% loading the data
% lidar_scans : (1)%time (2)field.header.seq (3)field.header.stamp	
%               (4)field.angle_min	(5)field.angle_max	(6)field.angle_increment
%               (7)min_range (8) max_range
%	            (9-368)field.ranges [0....359] 
location = 'uniquestringtoreplace';
data_set = 'clean';
lidar_scans = csvread(strcat(location,data_set,'_scan.csv'),1,0);
if strcmp(data_set,'wsn_2')    
    lidar_scans = [zeros(size(lidar_scans,1),1) lidar_scans];
end
% ticks : (1)%time (2)field.header.seq (3)field.header.stamp (4)field.time_stamp
%         (5)field.left_encoder (6)field.right_encoder (7)field.left_pwm (8)field.right_pwm
ticks = csvread(strcat(location,data_set,'_mobile_base_sensors_core.csv'),1,0);

% odometry : (1)field.header.seq	(2)field.header.stamp	
%            (3)field.pose.position.x	(4)field.pose.position.y (5)field.pose.position.z	
%            (6)orientation.x (7)orientation.y (8)orientation.z (9)orientation.w
%            (10-34)field.pose.covariance[0-25]
%            (35)field.twist.linear.x (36)field.twist.linear.y (37)field.twist.angular.z
%            (38-40) field.pose.pose.orientation.z,y,x (eulers).
odometry = csvread(strcat(location,data_set,'_odom.csv'),1,0);
if ~strcmp(data_set,'wsn_2')
    odometry(:,1)=[];
end
odometry = [odometry quat2eul(odometry(:,6:9))];
ground_truth = [odometry(:,1:8) odometry(:,34:36) quat2eul(odometry(:,6:9))];
ground_truth(:,14) = ground_truth(:,12);
odom_temp = [odometry(:,2) ground_truth(:,3:4) ground_truth(:,14)];


% removing 1st scan
lidar_scans(1:2,:)=[];
tick = [];
odom = [];
for i=1:size(lidar_scans,1)
    tick_index = find(ticks(:,3)<lidar_scans(i,3));
    if size(tick_index,1)~=0
        tick = [tick;ticks(tick_index(end),:)];
    end
    odom_index = find(odom_temp(:,1)<lidar_scans(i,3));
    if size(odom_index,1)~=0
        odom = [odom;odom_temp(odom_index(end),:)];
    end
end

lidar_scans = [lidar_scans tick];
% lidar_scans = [lidar_scans(:,1:371) zeros(size(lidar_scans,1),1) lidar_scans(:,1:372:end)];
% lidar_scans : (1)%time (2)scan.header.seq (3)scan.header.stamp	
%               (4)scan.angle_min	(5)scan.angle_max	(6)scan.angle_increment
%               (7)min_range (8) max_range
%	            (9-368)scan.ranges [0....359] 
%               (369)%time (370)ticks.header.seq (371)ticks.header.stamp (372)ticks.time_stamp
%               (373)ticks.left_encoder (374)ticks.right_encoder (375)ticks.left_pwm (376)ticks.right_pwm

timestamp_temp = lidar_scans(:,3);
% timestamp = [];
% for i=1:size(lidar_scans,1)
%     x = timestamp_temp(i);
%     r = mod(x,1000000);
%     y = (x - r)/1000000;
%     timestamp = [timestamp ; y r];
% end

nrays = 360;
min_theta = -pi+deg2rad(1);
max_theta = pi;
theta_temp = min_theta:deg2rad(1):max_theta;
theta = repmat(theta_temp,size(lidar_scans,1),1);

readings = lidar_scans(:,9:368);
valid_temp = ones(size(readings,1),size(readings,2));
% Put valid 0 where there is NaN, use isnan() function.
for i=1:size(readings,1)
    for j=1:size(readings,2)
        if isinf(readings(i,j))
            valid_temp(i,j) = 0;
            readings(i,j) = 0;
        end
    end
end

odom(:,1)=[]; % We can vaidate timestamps by not removing the 1st coloumn
odom = zeros(size(odom));

left_temp = lidar_scans(:,373);
left = left_temp;
count_left = 0;
max_ticks = 65535;
for i=2:size(left_temp,1)
    if (left_temp(i-1)-left_temp(i)>=63000)
        count_left = count_left + 1;
    elseif (left_temp(i-1)-left_temp(i)<=-63000)
        count_left = count_left - 1;
    end
    left(i) = max_ticks*count_left + left_temp(i);
end

right_temp = lidar_scans(:,374);
right = right_temp;
count_right = 0;
for i=2:size(right_temp,1)
    if right_temp(i-1)-right_temp(i)>=63000
        count_right = count_right + 1;
    elseif right_temp(i-1)-right_temp(i)<=-63000
        count_right = count_right - 1;
    end
    right(i) = max_ticks*count_right + right_temp(i);
end

timestamp_temp = (timestamp_temp - repmat(lidar_scans(1,371),size(timestamp_temp,1),1))*10^(-3);
timestamp = [];
for i=1:size(lidar_scans,1)
    x = timestamp_temp(i);
    r = mod(x,1000000);
    y = (x - r)/1000000;
    timestamp = [timestamp ; y round(r)];
end

temp_time = (lidar_scans(:,371) - repmat(lidar_scans(1,371),size(timestamp_temp,1),1))*10^(-3);
leftTimestamp = [];
for i=1:size(lidar_scans,1)
    x = temp_time(i);
    r = mod(x,1000000);
    y = (x - r)/1000000;
    leftTimestamp = [leftTimestamp ; y round(r)];
end
rightTimestamp = leftTimestamp;
disp(size(timestamp,1));
disp(size(lidar_scans,1));
disp(size(theta,1));
disp(size(readings,1));
disp(size(valid_temp,1));
disp(size(odom,1));
disp(size(left,1));
disp(size(right,1));
disp(size(leftTimestamp,1));
disp(size(rightTimestamp,1));

if size(odom,1) ~= size(timestamp,1)
    odom = [odom(1,:); odom];
end

data_to_log = [timestamp lidar_scans(:,4) lidar_scans(:,5) theta readings valid_temp odom left right leftTimestamp rightTimestamp];

%% Logging Data

part1 = '{"timestamp": [%d, %d], "nrays": 360, "min_theta": %.6f, "max_theta": %.6f, ';
part2_theta = '"theta": [ %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f ], ';
part3_readings = '"readings": [ %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f ], ';
part4_valid = '"valid": [ %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d ], ';
part5_odom = '"odometry": [ %.6f, %.6f, %.6f ], "estimate": [ null, null, null ], "true_pose": [ null, null, null ], ';
part6_ticks = '"left": %d, "right": %d, "leftTimestamp": [ %d, %d], "rightTimestamp": [ %d, %d] }';
formatSpec = strcat(part1,part2_theta,part3_readings,part4_valid,part5_odom,part6_ticks);

delete(strcat(data_set,'_log.log'));
for i=1:size(data_to_log,1)
    fileID = fopen(strcat(data_set,'_log.log'),'a');
    fprintf(fileID,formatSpec,data_to_log(i,:));
    fclose(fileID);
end


