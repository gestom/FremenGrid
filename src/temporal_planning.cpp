#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseArray.h>
#include <fremen/PlanningAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>


#include "fremen/Entropy.h"
#include "fremen/AddView.h"
#include "fremen/Visualize.h"
#include "fremen/SaveLoad.h"
#include "nav_msgs/GetPlan.h"

#define N 4

#define MAX_ENTROPY 450000

double MIN_X,MIN_Y,MIN_Z,RESOLUTION;
int DIM_X,DIM_Y,DIM_Z;

ros::ServiceClient *save_client_ptr;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionServer<fremen::PlanningAction> Server;

using namespace std;

float* entropies;

//Parameteres
double sensor_range, entropies_step;
int *numCellsX_ptr, *numCellsY_ptr;

MoveBaseClient *ac_ptr;

tf::TransformListener *tf_listener_ptr;

ros::ServiceClient *entropy_client_ptr;

 ros::Publisher *points_pub_ptr;

 float x[] = {-1.5, -2.6,-8.6, -5.7};
 float y[] = {-8.4, -1.7, -1.0, -8.6};

void execute(const fremen::PlanningGoalConstPtr& goal, Server* as)
{

    ROS_INFO("Generating goals for timestamp %f", goal->timestamp);
    as->acceptNewGoal();

    fremen::PlanningResult result;
    geometry_msgs::Pose pose;

    //update entropy grid and publishes markers
    fremen::Entropy entropy_srv;
    entropy_srv.request.z = 1.69;//convert to parameter
    entropy_srv.request.r = sensor_range;//convert to parameter
    entropy_srv.request.t = 0.0;

    //Markers Initialization
    visualization_msgs::MarkerArray points_markers;

    visualization_msgs::Marker test_point;
    test_point.header.frame_id = "/map";
    test_point.header.stamp = ros::Time::now();
    test_point.ns = "my_namespace";
    test_point.action = visualization_msgs::Marker::ADD;
    test_point.type = visualization_msgs::Marker::CUBE;
    test_point.color.a = 0.8;
    test_point.color.r = 0.1;
    test_point.color.g = 0.0;
    test_point.color.b = 1.0;
    test_point.pose.position.z = 0.1;
    test_point.pose.orientation.w = 1.0;
    test_point.scale.x = entropies_step;
    test_point.scale.y = entropies_step;


    //update grid
    int ind = 0;
    for(int i = 0; i < *numCellsX_ptr; i++)
    {
        for(int j = 0; j < *numCellsY_ptr; j++)
        {
            entropy_srv.request.x = MIN_X + entropies_step*i;
            entropy_srv.request.y = MIN_Y + entropies_step*j;

            //Entropy Service Call:
            if(entropy_client_ptr->call(entropy_srv) > 0)
            {
                entropies[ind] = entropy_srv.response.value;
                test_point.pose.position.x = entropy_srv.request.x;
                test_point.pose.position.y = entropy_srv.request.y;
                test_point.id = ind;
                test_point.color.r = 0.0;
                test_point.color.g = 1.0 - (1.0 * entropy_srv.response.value)/MAX_ENTROPY;
                test_point.color.b = (1.0 * entropy_srv.response.value)/MAX_ENTROPY;
                test_point.scale.z = 0.01 + (entropies_step * entropy_srv.response.value)/MAX_ENTROPY;
                test_point.pose.position.z = test_point.scale.z/2;
                points_markers.markers.push_back(test_point);
            }
            else
            {
                ROS_ERROR("Failed to call plan service!");
                exit(1);
            }
            ind++;
            ros::spinOnce();
        }
    }

    points_pub_ptr->publish(points_markers);

    //TODO - Generate goals (plan)
    for(int i = 0; i < N; i++)
    {
        pose.position.x = x[i];
        pose.position.y = y[i];
        pose.position.z = 0.0;
        pose.orientation.w = 1.0;
        result.locations.header.frame_id = "map";
        result.locations.poses.push_back(pose);
    }

    //send goals
    points_pub_ptr->publish(points_markers);
    as->setSucceeded(result);

}

int main(int argc,char *argv[])
{
    ros::init(argc, argv, "goal_generation");
    ros::NodeHandle n;

    ros::NodeHandle nh("~");
    nh.param("entropies_step", entropies_step, 0.5);
    nh.param("sensor_range", sensor_range, 4.6);

    n.getParam("/fremenGrid/minX",MIN_X);
    n.getParam("/fremenGrid/minY",MIN_Y);
    n.getParam("/fremenGrid/minZ",MIN_Z);
    n.getParam("/fremenGrid/dimX",DIM_X);
    n.getParam("/fremenGrid/dimY",DIM_Y);
    n.getParam("/fremenGrid/dimZ",DIM_Z);
    n.getParam("/fremenGrid/resolution",RESOLUTION);
    printf("Grid params %.2lf %.2lf %.2lf %i %i %i %.2f\n",MIN_X,MIN_Y,MIN_Z,DIM_X,DIM_Y,DIM_Z,RESOLUTION);


    int numCellsX = (RESOLUTION*DIM_X) / entropies_step;
    numCellsX_ptr = &numCellsX;
    int numCellsY = (RESOLUTION*DIM_Y) / entropies_step;
    numCellsY_ptr = &numCellsY;

    ROS_INFO("numCells: %d", numCellsX*numCellsY);

    //entropies grid
    entropies = (float*) malloc((numCellsX*numCellsY)*sizeof(float));

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);
    ac_ptr = &ac;

    ROS_INFO("Starting goal generation node...");

    tf::TransformListener tf_listener;
    tf_listener_ptr = &tf_listener;

    //entropy service client
    ros::ServiceClient entropy_client = n.serviceClient<fremen::Entropy>("/fremenGrid/entropy");
    entropy_client_ptr = &entropy_client;

    //plan service client
    ros::ServiceClient plan_client = n.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
    nav_msgs::GetPlan plan_srv;

    //move_base client
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "/map";


    ros::Publisher points_pub = n.advertise<visualization_msgs::MarkerArray>("/entropy_grid", 100);
    points_pub_ptr = &points_pub;

    //get robot pose
    tf::StampedTransform st;

    Server server(n, "planning", boost::bind(&execute, _1, &server), false);
    server.start();

    ros::spin();
    
    return 0;
}
