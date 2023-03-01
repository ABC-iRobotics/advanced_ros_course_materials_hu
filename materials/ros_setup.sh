#!/bin/bash
#############################################################################
# ROS

usage="$(basename "$0") [-h] [-v <1|2|b>] -- source the chosen ROS environment

where:
    -h  show this help text
    -v  chose ROS version
        1: ROS1
        2: ROS2
        b: Ros1_bridge"

while getopts "hv:" option; do
case ${option} in
v ) #For option v
    ROS_VER=${OPTARG}
    case ${ROS_VER} in
    1)
    echo 'Configuring ROS1 environment.'  
    # ROS 1
    source /opt/ros/noetic/setup.bash
    source ~/catkin_ws/devel/setup.bash
    ;;
    
    2)
    echo 'Configuring ROS2 environment.'
    # ROS2 
    source /opt/ros/foxy/setup.bash
    source ~/ros2_ws/install/setup.bash
    source ~/doosan2_ws/install/setup.bash
    ;;
    
    b)
    echo 'Configuring environment for ROS1_bridge.'
    echo ""
    echo 'Configuring ROS1 environment.'  
    # ROS 1
    source /opt/ros/noetic/setup.bash
    source ~/catkin_ws/devel/setup.bash
    echo ""
    echo 'Configuring ROS2 environment.'
    # ROS2 
    source /opt/ros/foxy/setup.bash
    source ~/ros2_ws/install/setup.bash
    source ~/doosan2_ws/install/setup.bash
    #echo ""
    #echo 'Configuring ROS1_bridge environment.'
    # ROS bridge
    #source ~/ros1_bridge_ws/install/setup.bash
    ;;
    
    * ) #For invalid option
    echo "Invalid version. You have to use: -v <1|2|b>"
    ;;
    esac

;;
h ) #For option h
echo "$usage"
exit
;;
\? ) #For invalid option
echo "You have to use: [-h] or [-v <1|2|b>]"
;;
esac
done
shift $((OPTIND - 1))




