#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include <sstream>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/util/PPM.h>
#include <string>
#include <ompl/config.h>
#include <iostream>
#include <stdlib.h>
#include <cmath>
#include <functional>
#include <vector>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "global_planner/Goal.h"
#include "global_planner/Waypoints.h"

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;



class GlobalPlanner
{
public:
	GlobalPlanner(ros::NodeHandle* nodehandle);
	// ~GlobalPlanner();
	bool plan(double goalX, double goalY);
	bool transmit_plan_client();
	void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& laserCloud2);
	void odometryCallback(const nav_msgs::Odometry::ConstPtr& odom);
	bool receive_goal_service(global_planner::Goal::Request &req, global_planner::Goal::Response &res);
	bool handle_waypoint_service(global_planner::Waypoints::Request &req, global_planner::Waypoints::Response &res);
	double round_num(double x) const;
	void stack_pointcloud();
	void init_stack();
	void voxelCallback(const visualization_msgs::MarkerArray::ConstPtr& voxels);


private:
	bool at_goal(double goalX, double goalY);
	bool isStateValid(const ob::State *state) const;
	int coordinates_to_voxel_index(double x, double y) const;
	bool intersects_obstacle(double stateX, double stateY, double pointX, double pointY) const;
	void init_voxel();

	og::SimpleSetupPtr ss;
	const double PI = 3.1415926;
	//const int threshold = 0.3;
	double odomTime = 0.0;
	double planning_time = 1.0;
	double vehicleX=0.0; double vehicleY=0.0; double vehicleZ=0.0;
	double curTime = 0, waypointTime = 0;
	double frameRate = 5.0;
	double laser_radius = 10.0;
	double too_close_threshold = 0.2;
	double minBoundX=-20; double minBoundY=-10.0;
	double maxBoundX=10; double maxBoundY=10.0;
	double goalX,goalY = 0.0;
	double speed = 1.0;
	double confidence_boundary = 4.0;
	int *voxel_grid;// = new int[voxel_num_x*voxel_num_y];
	double voxel_size = 0.1;
	int voxel_num_x = 1000;
	int voxel_num_y = 1000;
	
	int laserCloudCount = 0;
	double voxel_offsetX = 0.0;
	double voxel_offsetY = 0.0;
	int v_size = 100000;
	bool wall_follow_right=true;
	bool stop_immediately = false;
	bool sendSpeed = true;
	double waypointXYRadius = 1.0;



	ros::NodeHandle nh;
	
	ros::Publisher pubPath;
	ros::Publisher pubPoint;
	ros::Publisher backPub;
	ros::Publisher pubBoundary;
	ros::Publisher pubPlannerCloud;
	ros::Publisher pubWaypoint;
	ros::Publisher pubLast;
	ros::Publisher pubSpeed;

	ros::Subscriber subLaserCloud;
	ros::Subscriber subclose;
	ros::Subscriber subvoxels;
	ros::ServiceServer service_goal;
	ros::ServiceServer plan_to_wp_service;

	geometry_msgs::PoseArray waypoint_array;
	vector <geometry_msgs::Point> voxelarray;

};

#endif