/*
	This node loads a path stored in a file, in the same
	order it is written or backwards

	Author: Jose Luis Millan Valbuena
*/


/* Libraries */
#include <fstream>
#include <iostream>
#include <string>

/* ROS libraries */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class LoadPath{
public:
	LoadPath(std::string filename){

		// Initialize publisher
		pub_path_ = nh_.advertise<nav_msgs::Path>("path", 10);


		// File where path is stored
		std::cout<<"[LOAD PATH] Loading path stored in: "<<filename<<std::endl;
		File_.open(filename);
		if(!File_){
			std::cout<<"[LOAD PATH] ERROR. No file was opened"<<std::endl;
			exit(1);
		}
		std::string data_type;
		getline(File_, data_type);
		std::cout<<"[LOAD PATH] Reading: "<<data_type<<""<<std::endl;
		
		// Wait for subscribers
		while(pub_path_.getNumSubscribers()<1){
			// std::cout<<"[LOAD PATH] Waiting for subscribers "<<std::endl;
		}

		// Frame_id
		path2send_.header.frame_id="map";

		if (data_type=="Points"){	
			loadPointsFromFile();
		}else if(data_type=="Poses"){
			loadPosesFromFile();
		}else{
			std::cout<<"[LOAD PATH] Data type not recognized"<<data_type<<" ."<<std::endl;
		}
		
		
		
	}

	/* If the text contains POINTS */
	void loadPointsFromFile(){
		std::string x_str, y_str;
		float x,y;
		geometry_msgs::PoseStamped current_pose;
		current_pose.header.frame_id = "map";

		current_pose.pose.orientation.x = 0.0;
		current_pose.pose.orientation.y = 0.0;
		current_pose.pose.orientation.z = 0.0;
		current_pose.pose.orientation.w = 1.0;
		
		while(getline(File_,x_str,' ')){
			getline(File_,y_str);
			x = std::stof(x_str);
			y = std::stof(y_str);

			current_pose.pose.position.x = x;
			current_pose.pose.position.y = y;

			path2send_.poses.push_back(current_pose);

		}
		File_.close();
		
		pub_path_.publish(path2send_);
		std::cout<<"[LOAD PATH] Path sent"<<std::endl;
	}

	/* If the text contains POSES */
	void loadPosesFromFile(){
		std::string x_str, y_str, yaw_str;
		float x,y, yaw;

		geometry_msgs::PoseStamped current_pose;
		tf2::Quaternion q2;
		current_pose.header.frame_id = "map";
		
		while(getline(File_,x_str,' ')){
			getline(File_,y_str,' ');
			getline(File_,yaw_str);
			x = std::stof(x_str);
			y = std::stof(y_str);
			yaw = std::stof(yaw_str);

			current_pose.pose.position.x = x;
			current_pose.pose.position.y = y;

			q2.setRPY(0,0,yaw);
			q2.normalize();
			tf2::convert(q2,current_pose.pose.orientation);

			path2send_.poses.push_back(current_pose);
		}
		File_.close();

		std::cout<<"[LOAD PATH] Sending path"<<std::endl;
		pub_path_.publish(path2send_);
	}

	~LoadPath(){
		std::cout<<"\n[LOAD PATH] Shutting down"<<std::endl;
		if(File_.is_open()){
			std::cout<<"[LOAD PATH] Closing file"<<std::endl;
			File_.close();
		}
	}

private:
	ros::NodeHandle nh_;
	ros::Publisher pub_path_;

	nav_msgs::Path path2send_;

	std::ifstream File_;

	
};

int main(int argc, char **argv){
	// Start ros node
	ros::init(argc, argv, "load_path_node");

	std::string filename;

	if (argc>1){
		filename = argv[1];
	}else{
		filename = "$HOME/catkin_ws/src/path_utils/src/test_path.txt";
	}

	// Instantiate object
	LoadPath lp(filename);
	ros::spin();

	return 0;
}