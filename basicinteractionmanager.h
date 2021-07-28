#ifndef BASICINTERACTIONMANAGR_H
#define BASICINTERACTIONMANAGR_H
#include <string>
#include <vector>

#include "ros/ros.h"
#include "../catkin_ws/devel/include/vehicle_msgs/FloatArray.h"
#include "../catkin_ws/devel/include/vehicle_msgs/BasicInteraction.h"
#include "../catkin_ws/devel/include/vehicle_msgs/SeatPressureSensor.h"
#include "../catkin_ws/devel/include/vehicle_msgs/PressureMat.h"

#define mattYlength 30
#define mattXlength 32

#define frontSide 30
#define rightSide 32

#define percUpdate 5

class BasicInteractionManager
{
public:
    //send msgs: handsOn handsOff Leanback SitMid SitRight SitLeft SitFront SitBack
//settings:backMin backMax HandsOnMin HandsOnMax LooseMin LooseMax TightMin TightMax right front
    BasicInteractionManager(std::vector<bool> sendMsgs = {true,true,true,true,true,true,true,true},
                           std::vector<int> settings = {0,10,10,20,20,50,50,100,32,30} );
    float getMax(float tempArr [] );
    void publishMsg(std::string name, std::vector<int> vals , std::vector<int> perc);
    void wheelCallBack(const vehicle_msgs::FloatArray& msg );
    void mattCallBack(const vehicle_msgs::SeatPressureSensor& msg );
    int getConf(int value, int start , int max); //value<start:0%  value>max:100%

    int getHandsOnConf( float maxPressure );
    int getHandsOffConf(float maxPressure );
    bool sendHandsMsg( float tempWheelArr [8]  );

    int getLeaningBackConf ( float pressureSum );
    bool sendLeaningBackMsg( float arr[32][32]  );

    std::vector<int> getCenterOfPressure (float arr[mattXlength][mattYlength]);
    //int getSittingRightConf ( float arr[mattXlength][mattYlength] );
    //int getSittingLeftConf ( float arr[mattXlength][mattYlength] );
    //int getSittingFrontConf ( float arr[mattXlength][mattYlength] );
    //int getSittingBackConf ( float arr[mattXlength][mattYlength] );
    std::vector<int> getSittingConf ( float arr[mattXlength][mattYlength] ); // conf : mid,right,left,front,back
    bool sendSittingMsg( float arr[mattXlength][mattYlength]  );

private:
    ros::NodeHandle n;
    ros::Publisher interactionPub;
    std::string wheelTopic = "sensors/grip_force/h_mode_wheel";
    std::string mattTopic = "sensors/preasssureMatt";
    ros::Subscriber wheelSub;
    ros::Subscriber mattlSub;

    bool sendHandsOn;
    bool sendHansOff;
    bool sendLeaningBack;
    bool sendSittingMid;
    bool sendSittingRight;
    bool sendSittingLeft;
    bool sendSittingFront;
    bool sendSittingBack;

    int oldHandsOnConf =0;
    int oldHandsOffConf =0;
    int oldLeaningBackConf =0;

    int oldSittingConf[5] ={0,0,0,0,0};
    int oldSittingRightConf =0;
    int oldSittingLeftConf =0;
    int oldSittingFrontConf =0;
    int oldSittingBackConf =0;

    float backArr[32][32];
    float seatArr[mattXlength][mattYlength];
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

#endif // BASICINTERACTIONMANAGR_H
