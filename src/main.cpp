//#include "include/basicinteraction.h"
//#include "include/patternmatcher.h"
/*
#include "std_msgs/String.h"
#include "include/newpattern.h"
#include "basic.h"
#include <vector>

int main (int argc, char** argv)
{
   std::cout<<"main";
   ros::init(argc, argv,"receiver");
   std::vector<bool> sendMsgs = {true,true,false,false,false,false,false,false};
   int settings[] = {1,50,0,50,50,150,150,450,31,29};

   Basic* manager = new Basic();
   manager->setVals(sendMsgs,15,100,0,50,50,150,150,450,31,29);

   eventVector* receiver = new eventVector(5000);
   newSubPattern sub1 = newSubPattern();
   sub1.addElement(newPatternEvent(EventValues::handsOff,0,0,80.0/100,1) );
   sub1.addElement(newPatternEvent(EventValues::handsOff,8,20,60.0/100,-1) );
   newSubPattern sub2 = newSubPattern();
   sub2.addElement(newPatternEvent(EventValues::handsOn,20,27,79.0/100,1) );
   //sub2.addElement( newPatternEvent(EventValues::leaningBack,20,100,50.0/100,-1) );
   std::vector<newSubPattern> subPatterns = {sub1,sub2};

    newPattern* pattern = new newPattern(subPatterns);
    pattern->vect = receiver;
    ros::Rate loop_rate(10);
manager->spin();
     while (ros::ok()){

        receiver->spin();
        std::list<long int> returnlist = pattern->patternIsValid();
        std::cout<<"\npattern valid at"<<std::endl;
        for ( auto it =returnlist.begin(); it!=returnlist.end();it++ ) std::cout<<*it<<std::endl;
        std::cout<<receiver->receivedElements.size()<<std::endl;
        loop_rate.sleep();
     }



}
*/
