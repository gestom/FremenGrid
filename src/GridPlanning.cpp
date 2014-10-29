#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "fremen/Entropy.h"

#define MIN_X  -7.0
#define MIN_Y  -5.7
#define MIN_Z  0.0

#define DIM_X 300
#define DIM_Y 450
#define DIM_Z 80
#define RESOLUTION 0.05

#define MIN_ENTROPY 0.0
#define MAX_ENTROPY 132000

using namespace std;

double ptsInterval;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


int main(int argc, char *argv[])
{

    ros::init(argc, argv, "GridPlanning");
    ros::NodeHandle n;

    ros::NodeHandle nh("~");
    nh.param("interval", ptsInterval, 1.5);

    //Entropy Service Client
    ros::ServiceClient entropy_client = n.serviceClient<fremen::Entropy>("/fremenGrid/entropy");
    fremen::Entropy entropy_srv;

    //Publisher (Visualization of Points)
    ros::Publisher entropyPts_pub = n.advertise<visualization_msgs::MarkerArray>("/entropy_points", 100);
    ros::Publisher entropyValue_pub = n.advertise<visualization_msgs::MarkerArray>("/entropy_values", 100);


    visualization_msgs::MarkerArray points_markers;

    visualization_msgs::Marker test_marker;
    test_marker.header.frame_id = "/map";
    test_marker.header.stamp = ros::Time::now();
    test_marker.ns = "my_namespace";
    test_marker.action = visualization_msgs::Marker::ADD;
    test_marker.type = visualization_msgs::Marker::SPHERE;
    test_marker.scale.x = 0.3;
    test_marker.scale.y = 0.3;
    test_marker.scale.z = 0.3;
    test_marker.color.a = 0.8;
    test_marker.color.r = 0.1;
    test_marker.color.g = 0.0;
    test_marker.color.b = 1.0;
    test_marker.pose.orientation.w = 1.0;

    visualization_msgs::MarkerArray entropyValue;

    visualization_msgs::Marker entropy_txt;
    entropy_txt.header.frame_id = "/map";
    entropy_txt.header.stamp = ros::Time::now();
    entropy_txt.ns = "my_namespace";
    entropy_txt.action = visualization_msgs::Marker::ADD;
    entropy_txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    entropy_txt.scale.z = 0.1;
    entropy_txt.color.a = 1.0;
    entropy_txt.color.r = 1.0;
    entropy_txt.color.g = 1.0;
    entropy_txt.color.b = 1.0;
    entropy_txt.pose.orientation.w = 1.0;


    geometry_msgs::Point marker_point;

    //Entropy Grid (distance between points 1 meter)
    unsigned int nr_x, nr_y;
    nr_x = ((DIM_X*RESOLUTION-ptsInterval)/ptsInterval);
    nr_y = ((DIM_Y*RESOLUTION-ptsInterval)/ptsInterval);
    //    ROS_INFO("%d and %d", nr_x, nr_y);

    float entropy_grid[nr_x*nr_y];

    char output[1000];


    while (ros::ok())
    {

        float x = MIN_X+ptsInterval;
        float y = MIN_Y+ptsInterval;

        for(int i = 0; i < nr_x * nr_y; i++)
        {
            entropy_srv.request.x = x;
            test_marker.pose.position.x = x;
            entropy_txt.pose.position.x = x;
            entropy_srv.request.y = y;
            test_marker.pose.position.y = y;
            entropy_txt.pose.position.y = y;
            entropy_srv.request.z = 1.69;
            test_marker.pose.position.z = 1.69;
            entropy_txt.pose.position.z = 1.69;
            entropy_srv.request.r = 4;
            entropy_srv.request.t = 0.0;
            test_marker.id = i;
            entropy_txt.id = i;
            x += ptsInterval;

            //Entropy Srv
            if(entropy_client.call(entropy_srv)>0)
            {
                //                ROS_INFO("Entropy at Point (%f,%f) is %.3f", entropy_srv.request.x, entropy_srv.request.y, entropy_srv.response.value);
                entropy_grid[i] = entropy_srv.response.value;

                sprintf(output,"%.3f",entropy_srv.response.value);
                entropy_txt.text = output;
            }
            else
            {
                ROS_ERROR("Failed to call entropy service");
                return 1;
            }

            test_marker.scale.x = 0.5 * entropy_srv.response.value / MAX_ENTROPY;
            test_marker.scale.y = 0.5 * entropy_srv.response.value / MAX_ENTROPY;
            test_marker.scale.z = 0.5 * entropy_srv.response.value / MAX_ENTROPY;


            if(x > (MIN_X + DIM_X*RESOLUTION - ptsInterval))
            {
                x = MIN_X + ptsInterval;
                y += ptsInterval;
            }

            entropyValue.markers.push_back(entropy_txt);
            points_markers.markers.push_back(test_marker);
        }

        entropyPts_pub.publish(points_markers);
        entropyValue_pub.publish(entropyValue);


    }


    ros::spin();


    return 0;


}
