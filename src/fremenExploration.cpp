#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>
//#include <ros/move_base.h>

#include "fremen/Entropy.h"
#include "fremen/AddView.h"
#include "fremen/Visualize.h"
#include "nav_msgs/GetPlan.h"

#define MAX_ENTROPY 132000

bool drawEmptyCells = false;
bool drawCells = true;

double MIN_X,MIN_Y,MIN_Z,RESOLUTION;
int DIM_X,DIM_Y,DIM_Z;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace std;

struct keypoints{
    geometry_msgs::Point position;
    float entropy;
    float ratio;
    float ratioEstimate;
    double dist;
    double distEstimate;
    bool reachable;
};

int ptuMovementFinished = 0;

//Parameteres
double exploration_radius, entropy_step;

ros::Publisher ptu_pub;
sensor_msgs::JointState ptu;

//double distanceCalculate(double x1, double y1, double x2, double y2)
//{
//    return pow(x1 - x2,2)+pow(y1 - y2,2);
//}

double distanceCalculate(geometry_msgs::Point a, geometry_msgs::Point b)
{
    return sqrt(pow((a.x - b.x), 2) + pow((a.y - b.y), 2));
}


int comparekeypoints (const void * a, const void * b)
{
    struct keypoints *ia = (struct keypoints *)a;
    struct keypoints *ib = (struct keypoints *)b;
    if ( ia->ratioEstimate > ib->ratioEstimate) return -1;
    return 1;
}

void movePtu(float pan,float tilt)
{
    ptuMovementFinished = 0;
    ptu.name[0] ="pan";
    ptu.name[1] ="tilt";
    ptu.position[0] = pan;
    ptu.position[1] = tilt;
    ptu.velocity[0] = ptu.velocity[1] = 1.0;
    ptu_pub.publish(ptu);
}

void ptuCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    float pan,tilt;
    for (int i = 0;i<3;i++){
        if (msg->name[i] == "pan") pan = msg->position[i];
        if (msg->name[i] == "tilt") tilt = msg->position[i];
    }
    //printf("PTU: %.3f %.3f %i\n",pan,tilt,ptuMovementFinished);
    if (fabs(pan-ptu.position[0])<0.01 && fabs(tilt-ptu.position[1])<0.01) ptuMovementFinished++;
}

int main(int argc,char *argv[])
{
    ros::init(argc, argv, "FremenExploration");
    ros::NodeHandle n;

    ros::NodeHandle nh("~");
    nh.param("interval", entropy_step, 1.0);

    n.getParam("/fremenGrid/minX",MIN_X);
    n.getParam("/fremenGrid/minY",MIN_Y);
    n.getParam("/fremenGrid/minZ",MIN_Z);
    n.getParam("/fremenGrid/dimX",DIM_X);
    n.getParam("/fremenGrid/dimY",DIM_Y);
    n.getParam("/fremenGrid/dimZ",DIM_Z);
    n.getParam("/fremenGrid/resolution",RESOLUTION);
    printf("Grid params %.2lf %.2lf %.2lf %i %i %i %.2f\n",MIN_X,MIN_Y,MIN_Z,DIM_X,DIM_Y,DIM_Z,RESOLUTION);

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    ROS_INFO("Starting exploration node...");

    tf::TransformListener tf_listener;

    //Publishers (Visualization of Points + Entropy Values)
    ros::Publisher points_pub = n.advertise<visualization_msgs::MarkerArray>("/entropy_points", 100);
    ros::Publisher text_pub = n.advertise<visualization_msgs::MarkerArray>("/entropy_values", 100);

    //Subscribers
    ros::Subscriber ptu_sub = n.subscribe("/ptu/state", 10, ptuCallback);
    ptu.name.resize(3);
    ptu.position.resize(3);
    ptu.velocity.resize(3);
    ptu_pub = n.advertise<sensor_msgs::JointState>("/ptu/cmd", 10);


    //entropy service client
    ros::ServiceClient entropy_client = n.serviceClient<fremen::Entropy>("/fremenGrid/entropy");
    fremen::Entropy entropy_srv;

    //vizualize client
    ros::ServiceClient visualize_client = n.serviceClient<fremen::Visualize>("/fremenGrid/visualize");
    fremen::Visualize visualize_srv;

    //measure service client
    ros::ServiceClient measure_client = n.serviceClient<fremen::AddView>("/fremenGrid/depth");
    fremen::AddView measure_srv;

    //plan service client
    ros::ServiceClient plan_client = n.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
    nav_msgs::GetPlan plan_srv;

    //move_base client
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "/map";

    //get robot pose
    tf::StampedTransform st;

    //entropy points
    unsigned int nr_x, nr_y, grid_length;
    nr_x = ((DIM_X*RESOLUTION-entropy_step)/entropy_step);
    nr_y = ((DIM_Y*RESOLUTION-entropy_step)/entropy_step);

    grid_length = nr_x * nr_y;

    keypoints entropy_grid[grid_length];

    float x = MIN_X+entropy_step;
    float y = MIN_Y+entropy_step;

    //initialize grid:
    for(int i = 0; i < grid_length; i++)
    {
        entropy_grid[i].position.x = x;
        entropy_grid[i].position.y = y;
        entropy_grid[i].reachable = true;

        //Next coordinates:
        x += entropy_step;
        if(x > (MIN_X + DIM_X*RESOLUTION - entropy_step))
        {
            x = MIN_X + entropy_step;
            y += entropy_step;
        }
    }

    //Markers Initialization
    visualization_msgs::MarkerArray points_markers, values_markers;

    visualization_msgs::Marker test_point;
    test_point.header.frame_id = "/map";
    test_point.header.stamp = ros::Time::now();
    test_point.ns = "my_namespace";
    test_point.action = visualization_msgs::Marker::ADD;
    test_point.type = visualization_msgs::Marker::SPHERE;
    test_point.scale.x = 0.3;
    test_point.scale.y = 0.3;
    test_point.scale.z = 0.3;
    test_point.color.a = 0.6;
    test_point.color.r = 0.1;
    test_point.color.g = 0.0;
    test_point.color.b = 1.0;
    test_point.pose.position.z = 0.1;
    test_point.pose.orientation.w = 1.0;

    visualization_msgs::Marker text_point;
    text_point.header.frame_id = "/map";
    text_point.header.stamp = ros::Time::now();
    text_point.ns = "my_namespace";
    text_point.action = visualization_msgs::Marker::ADD;
    text_point.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_point.scale.z = 0.1;
    text_point.color.a = 1.0;
    text_point.color.r = 1.0;
    text_point.color.g = 1.0;
    text_point.color.b = 1.0;
    text_point.pose.position.z = 0.1;
    text_point.pose.orientation.w = 1.0;

    plan_srv.request.start.header.frame_id = "/map";
    plan_srv.request.start.pose.orientation.w = 1.0;
    plan_srv.request.goal.header.frame_id = "/map";
    plan_srv.request.goal.pose.orientation.w = 1.0;
    plan_srv.request.tolerance = 0.2;


    geometry_msgs::Pose current_pose;
    char output[1000];
    int max_ind = 0;
    double max_ratio = 0.0;
    double path_lenght = 0.0;

    int numPoints = 14;
    int point = 0;
    float pan[] =  { 0.00, 0.90, 1.80, 2.70, 2.70, 1.80, 0.90, 0.00,-0.90,-1.80,-2.70,-2.70,-1.80,-0.90,0.00};
    float tilt[] = { 0.50, 0.50, 0.50, 0.50,-0.30,-0.30,-0.30,-0.30,-0.30,-0.30,-0.30, 0.50, 0.50, 0.50,0.00};
    sleep(1);
    while (ros::ok())
    {
        point = 0;
        movePtu(pan[point],tilt[point]);
        ros::spinOnce();
        while (ros::ok() && point < numPoints)
        {
            measure_srv.request.stamp = 0.0;
            if (ptuMovementFinished > 10){
                if(measure_client.call(measure_srv))
                {
                    ROS_INFO("Measure added to grid!");
                }
                else
                {
                    ROS_ERROR("Failed to call measure service");
                    return 1;
                }
                point++;
                movePtu(pan[point],tilt[point]);
                ros::spinOnce();
                usleep(500000);
                if(drawCells){
                    visualize_srv.request.red = visualize_srv.request.blue = 0.0;
                    visualize_srv.request.green = visualize_srv.request.alpha = 1.0;
                    visualize_srv.request.minProbability = 0.9;
                    visualize_srv.request.maxProbability = 1.0;
                    visualize_srv.request.name = "occupied";
                    visualize_srv.request.type = 0;
                    visualize_client.call(visualize_srv);
                    ros::spinOnce();
                    usleep(100000);
                    if (drawEmptyCells){
                        visualize_srv.request.green = 0.0;
                        visualize_srv.request.red = 1.0;
                        visualize_srv.request.minProbability = 0.0;
                        visualize_srv.request.maxProbability = 0.1;
                        visualize_srv.request.alpha = 0.005;
                        visualize_srv.request.name = "free";
                        visualize_srv.request.type = 0;
                        visualize_client.call(visualize_srv);
                        ros::spinOnce();
                        usleep(100000);
                    }
                }

            }
            ros::spinOnce();
        }
        movePtu(0.0,0.0);

        max_ind = 0;
        max_ratio = 0.0;

        try
        {
            tf_listener.waitForTransform("/map","/base_link",ros::Time::now(), ros::Duration(2));
            tf_listener.lookupTransform("/map","/base_link",ros::Time(0),st);

            current_pose.position.x = st.getOrigin().x();
            current_pose.position.y = st.getOrigin().y();
            current_pose.position.z = st.getOrigin().z();

            //Evaluate Points
            plan_srv.request.start.pose.position = current_pose.position;

            for(int i = 0; i < grid_length; i++)
            {
                entropy_srv.request.x = entropy_grid[i].position.x;
                entropy_srv.request.y = entropy_grid[i].position.y;
                entropy_srv.request.z = 1.69;//convert to parameter
                entropy_srv.request.r = 4;//convert to parameter
                entropy_srv.request.t = 0.0;

                //Entropy Service Call:
                if(entropy_client.call(entropy_srv) > 0)
                {
                    //Save values for planning
                    entropy_grid[i].entropy = entropy_srv.response.value;
                    entropy_grid[i].ratioEstimate = entropy_srv.response.value/distanceCalculate(current_pose.position, entropy_grid[i].position);

                    //Add test point (size of the marker depends on the entropy value)
                    test_point.pose.position.x = entropy_grid[i].position.x;
                    test_point.pose.position.y = entropy_grid[i].position.y;
                    test_point.id = i;
                    test_point.scale.x = 0.6 * entropy_srv.response.value / MAX_ENTROPY;
                    test_point.scale.y = 0.6 * entropy_srv.response.value / MAX_ENTROPY;
                    points_markers.markers.push_back(test_point);

                    //Add text position  and value
                    text_point.pose.position.x = entropy_grid[i].position.x;
                    text_point.pose.position.y = entropy_grid[i].position.y;
                    sprintf(output,"%.3f",entropy_srv.response.value);
                    text_point.text = output;
                    text_point.id = i;
                    values_markers.markers.push_back(text_point);

                }
                else
                {
                    ROS_ERROR("Failed to call plan service!");
                    return 1;
                }
            }

            //Publish Visualization Markers
            points_pub.publish(points_markers);
            text_pub.publish(values_markers);

            //Sort (qsort)
            qsort (entropy_grid, grid_length, sizeof(keypoints), comparekeypoints);

            //Planning
            for(int i = 0; i < grid_length; i++)
            {
                //get plan
                plan_srv.request.goal.pose.position.x = entropy_grid[i].position.x;
                plan_srv.request.goal.pose.position.y = entropy_grid[i].position.y;

                path_lenght = 0.0;

                if(plan_client.call(plan_srv))//path received
                {
                    if((int) plan_srv.response.plan.poses.size() > 0)//path lenght is calculated
                    {

                        //lenght of the received path:
                        path_lenght = distanceCalculate(current_pose.position,plan_srv.response.plan.poses[0].pose.position);

                        for(int j = 0; j < (int) plan_srv.response.plan.poses.size() - 1; j++)
                            path_lenght += distanceCalculate(plan_srv.response.plan.poses[j].pose.position, plan_srv.response.plan.poses[j+1].pose.position);

                        entropy_grid[i].dist = path_lenght;

                        entropy_grid[i].ratio = entropy_grid[i].entropy/entropy_grid[i].dist;

                    }
                    else
                    {
                        entropy_grid[i].dist = 0.0;
                        entropy_grid[i].ratio = 0.0;
                        entropy_grid[i].reachable = false;
                    }

                    if(entropy_grid[i].ratio > max_ratio)
                    {
                        max_ratio = entropy_grid[i].ratio;
                        max_ind = i;
                    }

                    ROS_INFO("It: %d | Point: (%.1f,%.1f) | Entropy: %.3f | EstimatedRatio: %.3f | Ratio: %.3f | Next Estimated Ratio: %.3f", i, entropy_grid[i].position.x, entropy_grid[i].position.y, entropy_grid[i].entropy, entropy_grid[i].ratioEstimate, entropy_grid[i].ratio, entropy_grid[i+1].ratioEstimate);
                    if(max_ratio > entropy_grid[i+1].ratioEstimate && entropy_grid[i].reachable == true)
                    {
                        //move_base
                        ROS_INFO("Moving to point (%.1f,%.1f)...", entropy_grid[max_ind].position.x, entropy_grid[max_ind].position.y);
                        goal.target_pose.header.stamp = ros::Time::now();
                        goal.target_pose.pose.position.x = entropy_grid[max_ind].position.x;
                        goal.target_pose.pose.position.y = entropy_grid[max_ind].position.y;
                        goal.target_pose.pose.orientation.w = 1.0;

                        ac.sendGoal(goal);
                        ac.waitForResult();

                        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                            ROS_INFO("Hooray!");
                        else
                        {
                            entropy_grid[max_ind].reachable = false;
                            ROS_INFO("The base failed to move for some reason");
                        }
                        break;
                    }

                }
            }


        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("FreMeEn exploration could not estimate robot position %s",ex.what());
            return 0;
        }

        ros::spinOnce();
    }
    return 0;
}
