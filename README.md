# METR4202 Ximea Camera Setup
## Introduction
This tutorial is provided as guidelines for setting up the given Ximea cameras with ROS.

There will be four steps to this tutorial:
- Step 1: Installing Ximea Software Package
- Step 2: Install the ROS Ximxea Package
- Step 3: Camera Calibration
- Step 4: Setup the ArUco Tag Detection Library

**Note:**
- **This can be done on any Ubuntu 20.04 with IO/USB access.**
- **You cannot run the Ximea Camera on Construct/ROSjects or WSL.**
- **You should run this natively, on the RPi4, or on a Virtual Machine.**

## Step 1: Installing Ximea Software Package
- If you're on Ubuntu (Linux), make sure all of your packages are up to date, and that you have these installed
```
sudo apt-get update && sudo apt-get install build-essential linux-headers-"$(uname -r)" 
```
- On any machine running *Ubuntu*, download this software package and follow instructions from the ximea website [here](https://www.ximea.com/support/wiki/apis/ximea_linux_software_package).
- Or paste this commands into the terminal
```
wget https://www.ximea.com/downloads/recent/XIMEA_Linux_SP.tgz
```
```
tar xzf XIMEA_Linux_SP.tgz
```
```
cd package
```
```
./install
```
- You need to run the following command after each boot to disable the USB memory limits
```
echo 0 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
```
- Set realtime priority to the /etc/security/limits.conf
- Place the following in /etc/security/limits.conf to make the Ximea camera driver have real time priority
```
*               -       rtprio          0
@realtime       -       rtprio          81
*               -       nice            0
@realtime       -       nice            -16
```
- Then add the current user to the group realtime:
```
sudo groupadd realtime
sudo gpasswd -a $USER realtime
```

## Step 2: Install the ROS Ximea Package
For this step, you can follow this instructions, as per the tutorial [here](https://github.com/wavelab/ximea_ros_cam)
- Go to your catkin workspace source directory
```
cd ~/catkin_ws/src
```
- Clone the Ximea ROS Camera Repository

```
git clone https://github.com/wavelab/ximea_ros_cam.git
```
- Go back to the root of the workspace

```
cd ~/catkin_ws
```
- Then build the packages

```
catkin_make
```
- Make sure to setup your workspace

```
source devel/setup.sh
```
- Edit the ```example_camera``` file with ```rosed```

```
rosed ximea_ros_cam example_cam.launch
```
- Change the serial number to the number on your camera (also on the cable and case)

```
<launch>
    <node pkg="ximea_ros_cam" type="ximea_ros_cam_node" name="ximea_cam" output="screen">
        <param name="serial_no"       type="string" value="{INSERT YOUR SERIAL NUMBER HERE}" />
        <param name="cam_name"        type="string" value="ximea_cam" />
        <param name="calib_file"      type="string" value=""         />
        <param name="frame_id"        type="string" value="0"        />
        <param name="num_cams_in_bus" type="int"    value="2"        />
        <param name="bw_safetyratio"  type="double" value="1.0"      />
        <param name="publish_xi_image_info" type="bool" value="true"/>
        <param name="poll_time"       type="double" value="2.0"/>
        <param name="poll_time_frame" type="double" value="0.001"/>
        <rosparam command="load" file="$(find ximea_ros_cam)/config/example_cam_config.yaml" />
    </node>
</launch>
```
Test that everything is working correctly by running the launch file and ```rqt_image_view```

```
roslaunch ximea_ros_cam example_cam.launch
```
Run the GUI program ```rqt_image_view``` on a new terminal session
```
rosrun rqt_image_view rqt_image_view
```
Select the ```/ximea_cam/image_raw``` topic and you should see the output of the camera
## Step 3: Camera Calibration
- If it isn't installed already, install the ```camera_calibration``` package on ROS

```sudo apt install ros-noetic-camera-calibration```
- Next, while the example camera node is running, from the launch file, run the calibration python script
- Note: This is the same process for any time of camera, but with slightly different parameters
- The argument ```square``` should be the side length of each grid square in meters
- The argument ```size``` is the number of corner points (the grid given to you should be fine)
```
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.025 image:=/ximea_cam/image_raw camera:=/ximea_cam
```
- Follow the instructions on the GUI to CALIBRATE and COMMIT
## Step 4: Setup the ArUco Tag Detection Library
- Install the fiducials library by cloning the repository
```
cd ~/catkin_ws/src
```
```
git clone https://github.com/UbiquityRobotics/fiducials.git
```
Install the dependency ```vision_msgs```
```
sudo apt install ros-noetic-vision-msgs
```
Then make the library again
```
cd ~/catkin_ws
```
```
catkin_make
```
Finally, launch the ```aruco_detect``` node
```
roslaunch aruco_detect aruco_detect.launch camera:=/ximea_cam image:=/image_raw
```
Again, this should publish the images so you can run the following to view the image with the fiducials
```
rosrun rqt_image_view rqt_image_view
```
## Ending Notes

Please contact the METR4202 teaching team via Ed for the course, if you would like more clarification.
