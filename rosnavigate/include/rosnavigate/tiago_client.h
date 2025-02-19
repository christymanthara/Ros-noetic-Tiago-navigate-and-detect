#ifndef TIAGOCLIENT_H
#define TIAGOCLIENT_H

//ROS
#include <ros/ros.h>

//CREATING THE ACTION CLIENT, ACTION SERVER
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

//CREATING THE CUSTOM ACTION
#include <rosnavigate/TiagoMoveAction.h>

class TiagoClient{

    private: 
        actionlib::SimpleActionClient<rosnavigate::TiagoMoveAction> client;       
        float x;
        float y;
        float orZ; 

        /** callback for action done
         * @param &state final state 
         * @param &result_ptr Boost Pointer to the final result of the move and detect action
        */
        void doneCb(const actionlib::SimpleClientGoalState &state,const rosnavigate::TiagoMoveResultConstPtr &result_ptr);

        //callback for when robot starts the task
        void activeCb();

        /** callback for feedback messages
         * @param &feedback_ptr pointer to the MoveDetect Feedback messages
        */
        void feedbackCb(const rosnavigate::TiagoMoveFeedbackConstPtr &feedback_ptr);

    public: 

        /*  
        @param x coordinate x of the final pose (w.r.t. map ref. frame)
        @param y coordinate y of the final pose (w.r.t. map ref. frame)
        @param orZ orientation in the z axis of the final pose (w.r.t. map ref. frame)
        */
        TiagoClient(float x, float y, float orZ, std::string serverName);
        
        /* method for sending the goal and start the task */
        void sendGoal();

}; 

#endif // TIAGOSERVER
