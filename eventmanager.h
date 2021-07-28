#ifndef EVENTMANAGER_H
#define EVENTMANAGER_H
#include "../catkin_ws/devel/include/vehicle_msgs/FloatArray.h"
#include "../catkin_ws/devel/include/vehicle_msgs/BasicInteraction.h"
#include "../catkin_ws/devel/include/vehicle_msgs/SeatPressureSensor.h"
#include "../catkin_ws/devel/include/vehicle_msgs/PressureMat.h"

#include <string>

#include "ros/ros.h"
#include <vector>

#define mattYlength 30
#define rightSide 32
//only update the old values to current ones if the confidence have changed more than
#define percUpdate 5
class eventManager
{
public:
    eventManager();
    void publishMsg(std::string name, std::vector<int> vals , std::vector<int> perc);
    void wheelCallBack(const vehicle_msgs::FloatArray& msg );
    //returns max value of the size 8 arr
    float getMax(float tempArr [8] );
    //calculates the confidance of the pressure state
    int getHandsOnProb( float pressure);
    int getHandsOffProb( float pressure);
    //send a handsOn/Off message
    bool handsOn( float tempArr [8]);
    //send message of number of hands on wheel
    void handsNumber( float tempArr [8] );
    vehicle_msgs::BasicInteraction getHandsNumberMsg(float tempArr [8]);

    void pressureMattCallback(const vehicle_msgs::SeatPressureSensor& msg);
    //sends a message if leaning back
    void leaningBack(float arr[32][32] , float arrOld[32][32]);
    float getAvg(float arr[32][32]);

    void pressureMattNormCallback(const vehicle_msgs::SeatPressureSensor& msg);
    std::vector<int> getCenterOfPressure ( float arr [32][mattYlength] );
    //sends topics abt, sitting pos: mid,right,left,front,back
    bool centerOfPressureMsg(float newArr [32][mattYlength],float oldArr [32][mattYlength]);
    //returns mid conf,right,left,front,back
    std::vector<int> sittingConfidence( std::vector<int> pos );

    int sittingForwardConfidence(  std::vector<int> pos  );
    int sittingBackConfidence(  std::vector<int> pos  );

    int sittingRightConfidence(  std::vector<int> pos  );
    int sittingLeftConfidence(  std::vector<int> pos  );


private:
    ros::NodeHandle n;
    ros::Publisher interactionPub;

    std::string wheelTopic = "sensors/grip_force/h_mode_wheel";
    ros::Subscriber wheelSub;
    float wheelArr[8];
    float wheelMin;
    float wheelMaxHandsOn;
    float wheelMax;

    std::string mattTopic = "sensors/preasssureMatt";
    std::string mattTopicNorm = "sensors/preasssureMattNorm";
    ros::Subscriber mattlSub;
    ros::Subscriber mattlSubNorm;
    float mattArr[32][32];
    float backMin;
    float backMax;
    //used for the back of the seat
    vehicle_msgs::SeatPressureSensor oldMatPress;
    vehicle_msgs::SeatPressureSensor newMatPress;
    //used for the seat
    vehicle_msgs::SeatPressureSensor oldMatPressNorm;
    vehicle_msgs::SeatPressureSensor newMatPressNorm;


};

#endif // EVENTMANAGER_H
