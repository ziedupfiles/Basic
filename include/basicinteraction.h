#ifndef BASIC_H
#define BASIC_H
#include "ros/ros.h"
#include <string>
#include <vector>
#include <vehicle_msgs/FloatArray.h>
#include <vehicle_msgs/BasicInteraction.h>
#include <vehicle_msgs/SeatPressureSensor.h>
#include <vehicle_msgs/PressureMat.h>
#include <chrono>

#define printMsg 1
#define mattYstart 1
#define mattYlength 30
#define mattXlength 32

#define percUpdate 5

class Basic
{
public:
    //send msgs: handsOn handsOff Leanback SitMid SitRight SitLeft SitFront SitBack
    //settings:backMin backMax HandsOnMin HandsOnMax LooseMin LooseMax TightMin TightMax right front
    Basic( );
    void setVals(std::vector<bool> sendMsgs = {true,true,true,true,true,true,true,true},float backMin=15 ,float backMax=50 ,float wheelHandsOnMin=0,float wheelHandsOnMax=50,float wheelLooseMin=50,float wheelLooseMax=150,float wheelTightMin=150,float wheelTightMax=450,int rightSideMax=19,int frontSideMax=14 );
    void spin();

    float getMax(float tempArr [] );
    void publishMsg(std::string name, std::vector<int> vals , std::vector<int> perc);
    void wheelCallBack(const vehicle_msgs::FloatArray& msg );
    void mattCallBack(const vehicle_msgs::SeatPressureSensor& msg );
    int getConf(int value, int start , int max); //value<start:0%  value>max:100%

    bool sendHandsMsg( float tempWheelArr [8]  );

    bool sendLeaningBackMsg( float arr[32][32]  );

    std::vector<int> getSittingConf ( float arr[mattYlength][mattXlength] ); // conf : mid,right,left,front,back
    std::vector<int> getCenterOfPressure (float arr[mattYlength][mattXlength] );
    bool sendSittingMsg( float arr[mattYlength][mattXlength]  );

private:
    ros::NodeHandle n;
    ros::Publisher interactionPub;
    std::string wheelTopic = "sensors/grip_force/h_mode_wheel_raw";
    std::string mattTopic = "sensors/seat_pressure";
    ros::Subscriber wheelSub;
    ros::Subscriber mattlSub;

    bool sendHandsOn;
    bool sendHandsOff;
    bool sendLeaningBack;
    bool sendSitting[5];

    int oldHandsOnConf =0;
    int oldHandsOffConf =0;
    int oldLeaningBackConf =0;

    int oldSittingConf[5] ={0,0,0,0,0}; //mid right left front back

    float backArr[32][32];
    float seatArr[mattYlength][mattXlength];
    float wheelArr[8]; //transfer from 16arr to 8arr

    float backMin; //min value to get >0%
    float backMax; //max value to get 100%

    int rightSideMax;
    int frontSideMax;
    float wheelHandsOnMin; //min to get >0% handsOn
    float wheelHandsOnMax; //max to get 100% handson
    float wheelLooseMin; //min to get >0% handsOn
    float wheelLooseMax; //max to get 100% handsOn
    float wheelTightMin; // min to get >0% tight grip
    float wheelTightMax; // to get 100% tight grip

    vehicle_msgs::FloatArray wheelMsg;
    vehicle_msgs::PressureMat backMsg;
    vehicle_msgs::PressureMat seatMsg;
    vehicle_msgs::SeatPressureSensor mattMsg;
    /*vehicle_msgs::FloatArray oldWheelMsg;
    vehicle_msgs::PressureMat oldBackMsg;
    vehicle_msgs::PressureMat oldSeatMsg;

    vehicle_msgs::FloatArray newWheelMsg;
    vehicle_msgs::PressureMat newBackMsg;
    vehicle_msgs::PressureMat newSeatMsg;*/
};

#endif // BASIC_H
