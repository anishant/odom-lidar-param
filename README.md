# Odometry-LiDAR Calibration

Setup is lengthy, I recommend running the following as a shell script to reduce hassles.  
Though I explain each step below briefly.  
```
mkdir odom_lidar_calibration
cd odom_lidar_calibration
sudo apt install gsl-bin libgsl0-dev
git clone git://github.com/AndreaCensi/csm.git
cd csm
cmake .
make
sudo make install
export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig/:$PKG_CONFIG_PATH
cd ..
git clone git://github.com/AndreaCensi/calibration.git
cd calibration/src
cmake .
make
cd ../..
git clone git://github.com/anishant/odom-lidar-param.git
sed -i -e 's/"l90 lmov lstraight"/"fromscript"/g' calibration/scripts/script_variables.sh
```

1. Create a new working directory for calibration purposes
2. Install CSM
    1. sudo apt install gsl-bin libgsl0-dev
    2. git clone git://github.com/AndreaCensi/csm.git
    3. cd csm
    4. cmake .
    5. make
    6. sudo make install
    7. export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig/:$PKG_CONFIG_PATH
3. Install Calibration in the same directory
    1. git clone git://github.com/AndreaCensi/calibration.git
    2. cd calibration/src
    3. cmake .
    4. make 
4. Install odom-lidar-param in the same directory
    * git clone git://github.com/anishant/odom-lidar-param.git
5. Change name of file to be used in calibration package
    * sed -i -e 's/"l90 lmov lstraight"/"fromscript"/g' calibration/scripts/script_variables.sh
    