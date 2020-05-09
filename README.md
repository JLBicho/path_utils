# path_utils
Some nodes usefull to create, save and load paths

# Description
This package contains two nodes: one that reads poses from Rviz 2DNavGoal (with topic /path_poses) and saves them in a txt file. Another one that reads a txt file with poses saved.

## ROS Version
Kinetic

## Files
  
src
  - load_path.cpp
  - save_path.cpp

## Topics
<b>load_path.cpp</b>
  - Pubs:
    - /path (nav_msgs::Path): path to publish

<b>save_path.cpp</b>
  - Subs:
    - /path_poses (gemoetry_msgs::PoseStamped): poses read from Rviz 2DNavGoal (WARNING: Change name of topic in Rviz->Tool Properties)
  - Pubs:
    - /path (nav_msgs::Path): path to save and publish

  
