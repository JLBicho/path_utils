/*
	This node SAVEs a path stored in a file, in the same
	order it is written or backwards

	Author: Jose Luis Millan Valbuena
*/


/* Libraries */
#include <fstream>
#include <iostream>
#include <string>

/* ROS libraries */
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class SavePath{
public:
	SavePath(std::string filename){

		// Publishers
		pub_path_ = nh_.advertise<nav_msgs::Path>("path", 10);

		// Subscribers
		sub_poses_ = nh_.subscribe("path_poses", 10, &SavePath::pathPosesCb, this);

		// File to save path
		std::cout<<"[SAVE PATH] Saving path in: "<<filename<<std::endl;
		File_.open(filename);
		if(!File_){
			std::cout<<"[SAVE PATH] ERROR. No file was opened"<<std::endl;
			exit(1);
		}

		data_type_ = "Poses";
		File_<<data_type_<<"\n";

		// Frame_id
		path2send_.header.frame_id = "map";
	}

	~SavePath(){
		std::cout<<"\n[SAVE PATH] Shutting down"<<std::endl;
		if(File_.is_open()){
			std::cout<<"[SAVE PATH] Closing file"<<std::endl;
			File_.close();
		}
	}

	void pathPosesCb(const geometry_msgs::PoseStamped::ConstPtr& msg){
		std::string x_str, y_str, yaw_str;
		double x, y, yaw, roll, pitch;

		current_pose_ = *msg;

		// Push back current pose to path
		path2send_.poses.push_back(current_pose_);
		// Publish path
		pub_path_.publish(path2send_);

		// Get position
		x_str = std::to_string(current_pose_.pose.position.x);
		y_str = std::to_string(current_pose_.pose.position.y);
		// Get orientation to yaw 
		tf2::Quaternion q2;
		tf2::convert(current_pose_.pose.orientation,q2);
		tf2::Matrix3x3 m(q2);
		m.getRPY(roll,pitch,yaw);		
		yaw_str= std::to_string(yaw);

		if(data_type_=="Points"){
		// Save position
			File_<<x_str<<" "<<y_str<<"\n";
		}else if(data_type_=="Poses"){
		// Save pose
			File_<<x_str<<" "<<y_str<<" "<<yaw_str<<"\n";
		}

	}

	void savePoints(){
		
	}

private:
	ros::NodeHandle nh_;
	ros::Publisher pub_path_;
	ros::Subscriber sub_poses_;

	nav_msgs::Path path2send_;
	geometry_msgs::PoseStamped current_pose_;

	std::ofstream File_;

	std::string data_type_;
	
};

int main(int argc, char **argv){
	// Start ros node
	ros::init(argc, argv, "save_path_node");

	std::string filename;

	if (argc>1){
		filename = argv[1];
	}else{
		filename = "$HOME/Desktop/default_path.txt";
	}

	// Instantiate object
	SavePath sp(filename);
	ros::spin();

	return 0;
}