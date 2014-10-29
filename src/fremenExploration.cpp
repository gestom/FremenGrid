#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "fremen/Entropy.h"
#include "fremen/AddView.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace std;

//Parameteres
double exploration_radius;//just for a initial planning
int nr_points;

int main(int argc,char *argv[])
{
    ros::init(argc, argv, "FremenExploration");
    ros::NodeHandle n;

    ros::NodeHandle nh("~");
    nh.param("exploration_radius", exploration_radius, 2.0);
    nh.param("nr_points", nr_points, 12);

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    ROS_INFO("Starting exploration node...");

    tf::TransformListener tf_listener;

    //Publisher
    ros::Publisher testPts_pub = n.advertise<visualization_msgs::Marker>("/test_points", 100);
    ros::Publisher testEntropy_pub = n.advertise<visualization_msgs::MarkerArray>("/entropy_test", 100);

    geometry_msgs::Point position, next_position, marker_point;

    //entropy service client
    ros::ServiceClient entropy_client = n.serviceClient<fremen::Entropy>("/fremenGrid/entropy");
    fremen::Entropy entropy_srv;

    //measure service client
    ros::ServiceClient measure_client = n.serviceClient<fremen::AddView>("/fremenGrid/measure");
    fremen::AddView measure_srv;

    //move_base client
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "/map";


    //get robot pose
    tf::StampedTransform st;

    visualization_msgs::Marker test_marker;
    test_marker.header.frame_id = "/map";
    test_marker.header.stamp = ros::Time::now();
    test_marker.ns = "my_namespace";
    test_marker.action = visualization_msgs::Marker::ADD;
    test_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    test_marker.scale.x = 0.5;
    test_marker.scale.y = 0.5;
    test_marker.scale.z = 0.5;
    test_marker.color.a = 1.0;
    test_marker.color.r = 0.1;
    test_marker.color.g = 0.0;
    test_marker.color.b = 1.0;

    visualization_msgs::MarkerArray text_marker;

    visualization_msgs::Marker entropy_marker;
    entropy_marker.header.frame_id = "/map";
    entropy_marker.header.stamp = ros::Time();
    entropy_marker.ns = "my_namespace";
    entropy_marker.id = 1;
    entropy_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    entropy_marker.action = visualization_msgs::Marker::ADD;
    entropy_marker.pose.orientation.x = 0.0;
    entropy_marker.pose.orientation.y = 0.0;
    entropy_marker.pose.orientation.z = 0.0;
    entropy_marker.pose.orientation.w = 1.0;
    entropy_marker.scale.z = 0.5;
    entropy_marker.color.a = 1.0;
    entropy_marker.color.r = 0.0;
    entropy_marker.color.g = 0.0;
    entropy_marker.color.b = 0.0;

    char output[1000];


    while (ros::ok())
    {

        //AddView Srv
        measure_srv.request.stamp = 0.0;
        if(measure_client.call(measure_srv))
        {
            ROS_INFO("Measure added to grid!");
        }
        else
        {
            ROS_ERROR("Failed to call measure service");
            return 1;
        }

        try {
            tf_listener.waitForTransform("/map","/base_link",ros::Time::now(), ros::Duration(2));
            tf_listener.lookupTransform("/map","/base_link",ros::Time(0),st);

            position.x = st.getOrigin().x();
            position.y = st.getOrigin().y();

            float new_entropy = 0.0, old_entropy = 0.0;

            for(float i = 0; i < 2*M_PI; i+= 2*M_PI/nr_points)
            {
                entropy_srv.request.x = position.x + exploration_radius * cos(i);
                entropy_srv.request.y = position.y + exploration_radius * sin(i);
                entropy_srv.request.z = 1.69;
                entropy_srv.request.r = 4;
                entropy_srv.request.t = 0.0;

                marker_point.x = entropy_srv.request.x;
                marker_point.y = entropy_srv.request.y;
                marker_point.z = entropy_srv.request.z;

                entropy_marker.pose.position = marker_point;
                test_marker.points.push_back(marker_point);

                //Entropy Srv
                if(entropy_client.call(entropy_srv)>0)
                {
                    ROS_INFO("Entropy at Point (%f,%f) is %.3f", entropy_srv.request.x, entropy_srv.request.y, entropy_srv.response.value);
                    new_entropy = entropy_srv.response.value;
                }
                else
                {
                    ROS_ERROR("Failed to call entropy service");
                    return 1;
                }

                entropy_marker.pose.position = marker_point;
                test_marker.points.push_back(marker_point);

                sprintf(output,"%f",new_entropy);
                entropy_marker.text = output;

                if(new_entropy > old_entropy)
                {
                    old_entropy = new_entropy;
//                    ROS_ERROR("here!");
                    next_position.x = entropy_srv.request.x;
                    next_position.y = entropy_srv.request.y;
                    next_position.z = entropy_srv.request.z;

                }
                text_marker.markers.push_back(entropy_marker);

            }

            testPts_pub.publish(test_marker);
            testEntropy_pub.publish(text_marker);
            //Move Base
            ROS_INFO("Moving to point (%f,%f)...", next_position.x, next_position.y);
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = next_position.x;
            goal.target_pose.pose.position.y = next_position.y;
            goal.target_pose.pose.orientation.w = 1.0;

            ac.sendGoal(goal);
            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                ROS_INFO("Hooray!");
            else
                ROS_INFO("The base failed to move for some reason");

        }
        catch (tf::TransformException ex) {
            ROS_ERROR("FreMeEn map cound not incorporate the latest measurements %s",ex.what());
            return 0;
        }
    }


    ros::spin();


    return 0;
}
