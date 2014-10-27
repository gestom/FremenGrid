#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>
#include "CFremenGrid.h"
#include "CTimer.h"

#include "fremen/Entropy.h"
#include "fremen/SaveLoad.h"
#include "fremen/AddView.h"
#include "fremen/Visualize.h"
#include <std_msgs/String.h>

#define MIN_X -10.0
#define MIN_Y -10.0
#define MIN_Z  0.0
#define DIM_X 400 
#define DIM_Y 400 
#define DIM_Z 80 
#define RESOLUTION 0.05 

using namespace std;

bool debug = false;
int integrateMeasurements = 0;

CFremenGrid *grid;
tf::TransformListener *tf_listener;

ros::Publisher *octomap_pub_ptr, *estimate_pub_ptr,*clock_pub_ptr;
ros::Publisher retrieve_publisher;;
 
bool loadGrid(fremen::SaveLoad::Request  &req, fremen::SaveLoad::Response &res)
{
    grid->load(req.filename.c_str());
    ROS_INFO("3D Grid of %ix%ix%i loaded !",grid->xDim,grid->yDim,grid->zDim);
    res.result = true;
    return true;
}

bool saveGrid(fremen::SaveLoad::Request  &req, fremen::SaveLoad::Response &res)
{
    grid->save(req.filename.c_str(), (bool) req.lossy,req.order);
    ROS_INFO("3D Grid of %ix%ix%i saved !",grid->xDim,grid->yDim,grid->zDim);
    return true;
}

void points(const sensor_msgs::PointCloud2ConstPtr& points2)
{
	CTimer timer;
	if (integrateMeasurements > 0){
		timer.reset();
		timer.start();
		sensor_msgs::PointCloud points1,points;
		sensor_msgs::convertPointCloud2ToPointCloud(*points2,points1);
		tf::StampedTransform st;
		try {
			tf_listener->waitForTransform("/map","/head_xtion_depth_optical_frame",points2->header.stamp, ros::Duration(0.2));
			tf_listener->lookupTransform("/map","/head_xtion_depth_optical_frame",points2->header.stamp,st);
		}
		catch (tf::TransformException ex) {
			ROS_ERROR("FreMEn map cound not incorporate the latest measurements %s",ex.what());
		        return;
		}
		printf("Transform arrived %i \n",timer.getTime());	
		timer.reset();	
		tf_listener->transformPointCloud("/map", points1,points);
		int size=points.points.size();
		std::cout << "There are " << size << " fields." << std::endl;
		std::cout << "Point " << st.getOrigin().x() << " " <<  st.getOrigin().y() << " " << st.getOrigin().z() << " " << std::endl;
		float x[size+1],y[size+1],z[size+1];
		int last = 0;
		for(unsigned int i = 0; i < size; i++){
			if (isnormal(points.points[i].x) > 0)
			{	
				x[last] = points.points[i].x; 
				y[last] = points.points[i].y;
				z[last] = points.points[i].z;
				last++;
			}
		}
		x[last] = st.getOrigin().x();
		y[last] = st.getOrigin().y();
		z[last] = st.getOrigin().z();
		printf("Point cloud processed %i \n",timer.getTime());
		timer.reset();	
		grid->incorporate(x,y,z,last);
		printf("Grid updated %i \n",timer.getTime());	
		integrateMeasurements--;
	}
}

bool addView(fremen::AddView::Request  &req, fremen::AddView::Response &res)
{
	integrateMeasurements = 1;
}

bool estimateEntropy(fremen::Entropy::Request  &req, fremen::Entropy::Response &res)
{
	res.value = 1.0;
}

bool visualizeGrid(fremen::Visualize::Request  &req, fremen::Visualize::Response &res)
{
	//init visualization markers:
	visualization_msgs::Marker markers;
	geometry_msgs::Point cubeCenter;

	std_msgs::ColorRGBA m_color;
	m_color.r = req.red;
	m_color.g = req.green;
	m_color.b = req.blue;
	m_color.a = req.alpha;

	markers.header.frame_id = "/map";
	markers.header.stamp = ros::Time::now();
	markers.ns = req.name;
	markers.action = visualization_msgs::Marker::ADD;
	markers.type = visualization_msgs::Marker::CUBE_LIST;
	markers.scale.x = RESOLUTION;
	markers.scale.y = RESOLUTION;
	markers.scale.z = RESOLUTION;
	markers.color = m_color;
	markers.points.clear();
	float maxX = MIN_X+DIM_X*RESOLUTION-3*RESOLUTION/4;
	float maxY = MIN_Y+DIM_Y*RESOLUTION-3*RESOLUTION/4;
	float maxZ = MIN_Z+DIM_Z*RESOLUTION-3*RESOLUTION/4;
	int cnt = 0;
	int cells = 0;
	float estimate,minP,maxP;
	minP = req.minProbability;
	maxP = req.maxProbability;
	for(float z = MIN_Z;z<maxZ;z+=RESOLUTION){
		for(float y = MIN_Y;y<maxY;y+=RESOLUTION){
			for(float x = MIN_X;x<maxX;x+=RESOLUTION){
				estimate = grid->estimate(cnt,0);
				if(estimate > minP && estimate < maxP)
				{
					cubeCenter.x = x;
					cubeCenter.y = y;
					cubeCenter.z = z;
					markers.points.push_back(cubeCenter);
					//double h = (1.0 - fmin(fmax((z - MIN_Z) / (maxZ - MIN_Z), 0.0), 1.0)) * 0.7;
					markers.colors.push_back(m_color);
					cells++;
				}
				cnt++;
			}
		}
	}
	retrieve_publisher.publish(markers);
	res.number = cells;
	return true;
}

int main(int argc,char *argv[])
{
    ros::init(argc, argv, "fremengrid");
    ros::NodeHandle n;
    grid = new CFremenGrid(MIN_X,MIN_Y,MIN_Z,DIM_X,DIM_Y,DIM_Z,RESOLUTION);
    tf_listener    = new tf::TransformListener();

    ros::Time now = ros::Time(0);
    tf_listener->waitForTransform("/head_xtion_depth_optical_frame","/map",now, ros::Duration(3.0));
    ROS_INFO("Fremen grid start");

    //Subscribers:
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2> ("/head_xtion/depth/points",  1000, points);
    retrieve_publisher = n.advertise<visualization_msgs::Marker>("/fremenGrid/visCells", 100);

    //Services:
    ros::ServiceServer retrieve_service = n.advertiseService("/fremenGrid/visualize", visualizeGrid);
    ros::ServiceServer information_gain = n.advertiseService("/fremenGrid/entropy", estimateEntropy);
    ros::ServiceServer add_service = n.advertiseService("/fremenGrid/measure", addView);
    ros::ServiceServer save_service = n.advertiseService("/fremenGrid/save", saveGrid);
    ros::ServiceServer load_service = n.advertiseService("/fremenGrid/load", loadGrid);

    ros::spin();

    free(grid);
    return 0;
}
