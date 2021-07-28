#ifndef EVENTVECTOR_H
#define EVENTVECTOR_H

#include <vector>
#include <ros/ros.h>
#include <vehicle_msgs/BasicInteraction.h>
#include <chrono>

enum class EventValues {
        EVENT_DEFAULT = 0,
        handsOn = 1,
        handsOff = 2,
        leaningBack = 3,
        sittingMid = 4,
        sittingRight = 5,
        sittingLeft=6,
        sittingFront=7,
        sittingBack=8
    };

class eventElement
{
public:
    eventElement();
    eventElement(EventValues typ, long int  Time, float conf);

    EventValues EventType;
    long int  timeStamp;
    float confidence;

    // override to use it for sorting
    bool operator <(const eventElement& d) {
        if (this->timeStamp < d.timeStamp) {
            return true;
        }
        return false;
    }

    bool operator ==(const eventElement& d) {
        if (this->timeStamp == d.timeStamp) {
            return true;
        }
        return false;
    }

};


class eventVector
{
public:
    //eventVector();
    eventVector(int duration= 1000); // duration is the vector length from first to last ele, used to delete old elements
    void callBack(const vehicle_msgs::BasicInteraction& msg  );
    void addElement(eventElement* event);
    void checkTime();
    std::vector<eventElement*> returnVector();
    std::vector<eventElement*> receivedElements;
    void spin();
private:
    std::vector<eventElement*> newreceivedElements;
    ros::NodeHandle n;
    ros::Subscriber eventSub;
    int vectorDuration;
    std::string topic = "BasicInteraction";
    std::string subTypes[9] = {"EVENT_DEFAULT","handsOn","handsOff","leaningBack","sittingMid","sittingRight","sittingLeft","sittingFront","sittingBack"};
};


#endif
