# Fivecomm modem package for ROS

The package must be cloned into the src folder of the ROS workspace
```
cd ~/catkin_ws/src
git clone https://github.com/Fivecomm/fivecomm_5g_router.git
```
Next the workspace must be compiled with catkin_make or catkin build
```
cd ~/catkin_ws
catkin_make or catkin build
source devel/setup.bash
```
Finally the package is ready and can be launched as:
```
roslaunch fivecomm_5g_router fivecomm_5g_router.launch freq:=1
```
This file has 4 launch arguments that can be set:

- **freq**: Set topic frequency (default 1)
- **gateway**: Set modem gateway (default 192.168.2.1)
- **username**: Set modem username (default root)
- **password**: Set modem password (default fivecomm)

To check that it works, read the topic published as:
```
rostopic echo /fivecomm_5g_router/info
```


For more information about the Fivecomm 5G modem:
https://fivecomm.eu/
<img src="./fivecomm.png" width="15%" alt="Fivecomm"/>
