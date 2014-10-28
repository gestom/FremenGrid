#include <ros/ros.h>
#include <tf/transform_listener.h>
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

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    ros::NodeHandle nh("~");
    nh.param("exploration_radius", exploration_radius, 1.5);
    nh.param("nr_points", nr_points, 8);

    ROS_INFO("Starting exploration node...");

    tf::TransformListener tf_listener;

    geometry_msgs::Point position, next_position;

    //entropy service client
    ros::ServiceClient entropy_client = n.serviceClient<fremen::Entropy>("information_gain");
    fremen::Entropy entropy_srv;

    //measure service client
    ros::ServiceClient measure_client = n.serviceClient<fremen::AddView>("measure");
    fremen::AddView measure_srv;

    //move_base client
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "/map";


    //get robot pose
    tf::StampedTransform st;

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
            tf_listener.lookupTransform("/map","/base_link",ros::Time(0),st);

            position.x = st.getOrigin().x();
            position.y = st.getOrigin().y();

            float new_entropy, old_entropy = 0.0;
            for(int i = 0; i < M_PI; i+= M_PI/nr_points)
            {
                entropy_srv.request.x = exploration_radius * cos(i);
                entropy_srv.request.y = exploration_radius * sin(i);
                entropy_srv.request.z = 1.69;
                entropy_srv.request.r = exploration_radius;
                entropy_srv.request.t = 0.0;

                //Entropy Srv
                if(entropy_client.call(entropy_srv))
                {
                    ROS_INFO("Entropy at Point (%f,%f) is %.3f", entropy_srv.request.x, entropy_srv.request.y, entropy_srv.response.value);
                    new_entropy = entropy_srv.response.value;
                }
                else
                {
                    ROS_ERROR("Failed to call entropy service");
                    return 1;
                }

                if(new_entropy > old_entropy)
                {
                    old_entropy = new_entropy;
                    next_position.x = entropy_srv.request.x;
                    next_position.y = entropy_srv.request.y;
                    next_position.z = entropy_srv.request.z;

                }

            }

            //Move Base
            ROS_INFO("Moving to point (%f,%f)...", entropy_srv.request.x, entropy_srv.request.y);
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
            ROS_ERROR("FreMEn map cound not incorporate the latest measurements %s",ex.what());
            return 0;
        }
    }


    ros::spin();


    return 0;
}
