 /*
    By Alphonsus Adu-Bredu - alphonsusbq436@gmail.com
	Architecture:
	1. goal is received from a goal client by goal service
	2. In goal service, in a while loop, plan from current pose to goal
	3. Send plan to posearray client(waypointexample)
	4. In waypointexample, goal is followed until it reaches bounds 3x6
	5. Returns back to goal service while loop and plans again from 
		vehicle pose to goal. And transmits plan again
	6. while loop breaks when |currentpose - goalpose| < threshold.
	7. returns true to goal client
   */


#include "global_planner.h"

const int laserCloudStackNum = 15;
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudStack[laserCloudStackNum];


GlobalPlanner::GlobalPlanner(ros::NodeHandle* nodehandle):nh(*nodehandle)
{
	
	subclose = nh.subscribe<nav_msgs::Odometry>
								("/state_estimation",5,&GlobalPlanner::odometryCallback,this);
	subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>
	                              (/*"/registered_scan"/*/"/passthrough/output"/**/, 5, &GlobalPlanner::pointCloudCallback,this);
	pubPath = nh.advertise<nav_msgs::Path> ("/global_path", 5);
	pubPoint = nh.advertise<geometry_msgs::PointStamped>("/goal_point",5);
	backPub = nh.advertise<geometry_msgs::PointStamped>("/way_point",5);
	pubBoundary = nh.advertise<geometry_msgs::PolygonStamped> ("/search_space", 5);
	pubPlannerCloud = nh.advertise<sensor_msgs::PointCloud2> ("/plannercloud", 2);
	init_stack();

	service_goal = nh.advertiseService("goal_channel", &GlobalPlanner::receive_goal_service,this);
	ROS_INFO("PLAYER READY SAMA");

}


bool GlobalPlanner::plan(double goalX, double goalY)
{
	ros::spinOnce();
	ROS_INFO("Current Position: %f, %f",vehicleX,vehicleY);
	if (!ss) return false;
	ROS_INFO("Planning to coordinates %f,%f",goalX,goalY);

	ob::ScopedState<> start(ss->getStateSpace());
	start[0] = vehicleX;
	start[1] = vehicleY;

	ob::ScopedState<> goal(ss->getStateSpace());
	goal[0] = goalX;
	goal[1] = goalY;

	ss->setStartAndGoalStates(start,goal);
	// ss->setOptimizationObjective(getClearanceObjective(ss->getSpaceInformation()));

	for (int i=0; i<1; ++i)
	{
		if (ss->getPlanner())
			ss->getPlanner()->clear();
		ss->solve(planning_time);
	}

	const std::size_t ns = ss->getProblemDefinition()->getSolutionCount();

	OMPL_INFORM("Found %d solutions",(int)ns);

	if (ss->haveSolutionPath())
	{
		ss->simplifySolution();
		og::PathGeometric &p = ss->getSolutionPath();
		ss->getPathSimplifier()->simplifyMax(p);
		ss->getPathSimplifier()->smoothBSpline(p);

		return true;

	}
	return false;
}



bool GlobalPlanner::transmit_plan_client()
{
	ros::spinOnce();
	ROS_INFO("Sending global plan");
	if(!ss || !ss->haveSolutionPath())
		return false;

	nav_msgs::Path path;
	path.header.frame_id="map";

	og::PathGeometric &p = ss->getSolutionPath();
	p.interpolate();
	ros::ServiceClient client = nh.serviceClient<global_planner::Waypoints>("specific_waypoint_channel");
	global_planner::Waypoints srv;

	int path_size = p.getStateCount();
	ROS_INFO("Starting for loop. There are %d states",path_size);

	path.poses.resize(path_size);
	srv.request.waypoints.poses.resize(path_size);
	
	
	int cutoff_index = path_size;
	double dist=0;

	for(int i=0; i<path_size; i++)
	{
		const double x = 
			(double)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[0];

		const double y = 
			(double)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[1];

		dist = sqrt(pow((vehicleX-x),2) + pow((vehicleY-y),2));
	
		if (dist>confidence_boundary and i >0) 
			{cutoff_index = i; 
				cout <<"cat off: "<<i<<endl; break;
			}	
	}	
			
	for(int i=0; i<path_size; i++)
	{
		const double x = 
			(double)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[0];

		const double y = 
			(double)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[1];

		path.poses[i].pose.position.x = x;
		path.poses[i].pose.position.y = y;
		dist = sqrt(pow((vehicleX-x),2) + pow((vehicleY-y),2));		
		cout << x << "," << y <<endl;
	}



	for (std::size_t i=1; i<path_size; i++) //<cutoff_index
	{
		const double x = 
			(double)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[0];

		const double y = 
			(double)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[1];

		srv.request.waypoints.poses[i-1].position.x = x;
		srv.request.waypoints.poses[i-1].position.y = y;
		
		cout << x << "," << y <<endl;
		
	}
	pubPath.publish(path);
	
	ROS_INFO("Cut-off index is %d", cutoff_index);
	srv.request.waypoints.header.seq = path_size-1;
	
	
	if (client.call(srv))
	{
		ROS_INFO("Global plan reached");
		return true;
	}
		
}

void GlobalPlanner::init_stack()
{
	for (int i = 0; i < laserCloudStackNum; i++) {
    	laserCloudStack[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
}

double GlobalPlanner::round_num(double num) const
{
	return floor(num*10+0.5)/10;
}

void GlobalPlanner::stack_pointcloud()
{
	laserCloudStack[laserCloudCount]->clear();
	*laserCloudStack[laserCloudCount] = *laserCloud;
	laserCloudCount = (laserCloudCount + 1)%laserCloudStackNum;

	plannerCloud->clear();
	for (int i=0; i<laserCloudStackNum; i++)
		*plannerCloud+=*laserCloudStack[i];

 	sensor_msgs::PointCloud2 pcloud;
	pcl::toROSMsg(*plannerCloud, pcloud);
	pcloud.header.frame_id = "/map";
	pubPlannerCloud.publish(pcloud);
}

void GlobalPlanner::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& laserCloud2)
{
	plannerCloud->clear();
	pcl::fromROSMsg(*laserCloud2, *plannerCloud);	
	stack_pointcloud();
}


void GlobalPlanner::odometryCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
	double roll,pitch,yaw;
	geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
	tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w))
																.getRPY(roll, pitch, yaw);
	vehicleX = odom->pose.pose.position.x;
  	vehicleY = odom->pose.pose.position.y;
}



bool GlobalPlanner::receive_goal_service(global_planner::Goal::Request &req, global_planner::Goal::Response &res)
{
	ros::Rate loop_rate(0.5);
	bool replan = false;
	goalX = req.goal_pose.position.x;	goalY = req.goal_pose.position.y;

	geometry_msgs::PointStamped goal;
	goal.header.frame_id = "/map";
	goal.point.x = goalX;
	goal.point.y = goalY;
	pubPoint.publish(goal);

	int r = rand()%10;
	if (r < 5) wall_follow_right =true;
	else	wall_follow_right = false;

	while (ros::ok())
	{
		ros::spinOnce();
		auto space (std::make_shared<ob::RealVectorStateSpace>());
		//set bounds
		double delta = 1.0;
		if(goalY > vehicleY and goalX > vehicleX)
		{
			minBoundX = vehicleX - delta; maxBoundX = goalX+delta;
			minBoundY = vehicleY - delta; maxBoundY = goalY+delta;  
		}

		else if (goalY < vehicleY and goalX > vehicleX)
		{
			minBoundX = vehicleX-delta; maxBoundX = goalX+delta;
			minBoundY = goalY-delta; maxBoundY = vehicleY+delta;
		}

		else if (goalY > vehicleY and goalX < vehicleX)
		{
			minBoundX = goalX-delta; maxBoundX = vehicleX+delta;
			minBoundY = vehicleY-delta; maxBoundY = goalY+delta;
		}

		else if (goalY < vehicleY and goalX < vehicleX)
		{
			minBoundX = goalX-delta; maxBoundX = vehicleX+delta;
			minBoundY = goalY-delta; maxBoundY = vehicleY+delta;
		}

		else
		{
			minBoundX = vehicleX-10.0; maxBoundX = vehicleX+10.0;
			minBoundY = vehicleY-10.0; maxBoundY = vehicleY+10.0;
		}

		minBoundY-=10; maxBoundY+=10;
		minBoundX-=10; maxBoundX+=10;
		space->addDimension(minBoundX, maxBoundX);
		space->addDimension(minBoundY, maxBoundY);

		geometry_msgs::PolygonStamped boundaryMsgs;
		boundaryMsgs.polygon.points.resize(4);
		boundaryMsgs.header.frame_id = "/map";
		boundaryMsgs.polygon.points[0].x = minBoundX;
		boundaryMsgs.polygon.points[0].y = minBoundY;
		boundaryMsgs.polygon.points[3].x = minBoundX;
		boundaryMsgs.polygon.points[3].y = maxBoundY;
		boundaryMsgs.polygon.points[1].x = maxBoundX;
		boundaryMsgs.polygon.points[1].y = minBoundY;
		boundaryMsgs.polygon.points[2].x = maxBoundX;
		boundaryMsgs.polygon.points[2].y = maxBoundY;

		pubBoundary.publish(boundaryMsgs);

		ss = std::make_shared<og::SimpleSetup>(space);
		ss->setStateValidityChecker([this](const ob::State *state)
												{return isStateValid(state);});
		ss->getSpaceInformation()->setStateValidityCheckingResolution
														(1.0/space->getMaximumExtent());
		ss->setPlanner(ob::PlannerPtr (new og::RRTstar(ss->getSpaceInformation())));

		if (plan(goalX,goalY))
		{
			ROS_INFO("REPLANNING...");
			replan = transmit_plan_client();
		}

		else
		{
			geometry_msgs::PointStamped back;
			back.header.frame_id = "/map";
			// back.header.stamp = ros::Time().fromSec(curTime);
		
			
				back.point.x = vehicleX;
				if (wall_follow_right)
					back.point.y = vehicleY+0.5;
				else	
					back.point.y = vehicleY-0.5;
			
			
			// back.point.z = waypoint_array.poses[wayPointID].position.z;
			backPub.publish(back);
		}
		

		// if(replan)  //gotten to the end of transmitted global plan
		pubPoint.publish(goal);
    	ros::spinOnce();
    	loop_rate.sleep();

    	if (at_goal(goalX,goalY))
    	{
    		res.status=true;
    		break;
    	}

	}

	return true;

}




bool GlobalPlanner::at_goal(double goalX, double goalY)
{
	double dist = sqrt(pow((vehicleX-goalX),2) + pow((vehicleY-goalY),2));
	if (dist < 1.2)
		return true;
	else
		return false;
}

bool GlobalPlanner::intersects_obstacle(double stateX, double stateY, double pointX, double pointY) const
{
	double vehicle_state_gradient = ((vehicleY-stateY)/(vehicleX-stateX));
	double vehicle_point_gradient = ((vehicleY-pointY)/(vehicleX-pointY));
	double point_state_gradient = ((pointY-stateY)/(pointX-stateX));

	if (vehicle_state_gradient == vehicle_point_gradient)// and vehicle_point_gradient == point_state_gradient)
		return true;
	return false;
}

bool GlobalPlanner::isStateValid(const ob::State *state) const
{
	const double x = std::min((double)state->as<ob::RealVectorStateSpace::StateType>()->values[0],
		maxBoundX);
	const double y = std::min((double)state->as<ob::RealVectorStateSpace::StateType>()->values[1],
		maxBoundY);
	
	bool free = true;

	double disp = sqrt(pow((x-vehicleX),2) + pow((y-vehicleY),2));
	if (disp < laser_radius)
	{
		int cloudsize = plannerCloud->points.size();
		pcl::PointXYZI point;
		for(int i=0; i<cloudsize; i++)
		{
			point = plannerCloud->points[i];
			float pointX = point.x;
		    float pointY = point.y;
		    float pointZ = point.z;



		    float dis = sqrt(pow(pointX-x,2)+pow(pointY-y,2));

		    bool intersects = intersects_obstacle(x, y, pointX, pointY);
		    if (dis < too_close_threshold)
		    {
		    	free = false;
		    	return free;
		    }		   
		}
	}

	else
		free = true;
	return free;
}








int main(int argc, char ** argv)
{	
	ros::init(argc,argv,"global_oplanner");
	ros::NodeHandle nh;
	GlobalPlanner globalPlanner(&nh);
	ros::spin();
	return 0;
		
}