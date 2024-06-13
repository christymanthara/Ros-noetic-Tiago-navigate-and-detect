#ifndef TIAGOSERVER_H
#define TIAGOSERVER_H

//ROS
#include <ros/ros.h>

// ACTION CLIENT, ACTION SERVER
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>

// MOVE BASE
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>

//CUSTOM ACTION
#include <rosnavigate/TiagoMoveAction.h>

// GEOMETRY_MSGS
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

// SENSOR_MSGS
#include <sensor_msgs/LaserScan.h>

// STL
#include <list>

// DETECTION
#include <detection/obstacle_detection/point.h>
#include <detection/obstacle_detection/circle.h>
#include <detection/obstacle_detection/obstacle_extractor.h>

// Action specification for move_base
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; 

class TiagoServer {

    private:
        ros::NodeHandle nh;                                                                  //Node for managing the action server
        actionlib::SimpleActionServer<rosnavigate::TiagoMoveAction> server;    //The action server
        MoveBaseClient client;                                                               //Client to the Navigation Stack 
        rosnavigate::TiagoMoveFeedback feedback;                               //Feedback message
        rosnavigate::TiagoMoveResult result;                                   //Result message
        rosnavigate::TiagoMoveGoal goal;                                       //Goal message 

        /** method for managing all the navigation part of the task
        @param &goal reference to the MoveDetectAction message goal
        */
        void doNavigation(const rosnavigate::TiagoMoveGoalConstPtr &goal);

        /** method for the detection task
        */
        void doDetection();

    public:

        /** constructor
         * @param name of the action server
        */
        TiagoServer(std::string name);


        /** callback for the main task of the robot
         * @param &goal final pose of the navigation stack 
        */
        void navAndDetectCallback(const rosnavigate::TiagoMoveGoalConstPtr &goal);

    
};

#endif // TIAGOSERVER
