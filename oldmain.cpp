/*
#include "basic.h"

int oldmain(int argc, char *argv[])
{
//    eventManager* vManager = new eventManager();
ros::init(argc, argv, "node_name");
std::vector<bool> sendMsgs = {true,true,false,false,false,false,false,false};
int settings[] = {1,50,0,50,50,150,150,450,31,29};

//settings:backMin backMax HandsOnMin HandsOnMax LooseMin LooseMax TightMin TightMax right front
   // Basic* manager = new Basic(sendMsgs,1,50,0,50,50,150,150,450,31,29);
    Basic* manager = new Basic();
    manager->setVals(sendMsgs,15,100,0,50,50,150,150,450,31,29);
     //while (true) manager->spin();
}
*/

#include "std_msgs/String.h"
#include "include/newpattern.h"
#include "basic.h"
#include <vector>

int main (int argc, char** argv)
{
   std::cout<<"main";
   ros::init(argc, argv,"receiver");
   std::vector<bool> sendMsgs = {true,true,true,false,false,false,false,false};
   int settings[] = {1,50,0,50,50,150,150,450,31,29};

   Basic* manager = new Basic();
   manager->setVals(sendMsgs,15,100,0,50,50,150,150,450,31,29);

   eventVector* receiver = new eventVector(1000);
   newSubPattern sub1 = newSubPattern();
   //sub1.addElement(newPatternEvent(EventValues::handsOff,0,0,80.0/100,1) );
   sub1.addElement(newPatternEvent(EventValues::handsOff,0,0,60.0/100,-1) );
   newSubPattern sub2 = newSubPattern();
   sub2.addElement(newPatternEvent(EventValues::handsOn,100,100,79.0/100,1) );
   sub2.addElement( newPatternEvent(EventValues::leaningBack,1000,1000,50.0/100,-1) );
   std::vector<newSubPattern> subPatterns = {sub1,sub2};

    newPattern* pattern = new newPattern(subPatterns);
    pattern->vect = receiver;

    ros::Rate loop_rate(10);
    //manager->spin();
     while (ros::ok()){
        manager->spin();
        receiver->spin();
        receiver->checkTime();
        std::list<long int> returnlist = pattern->patternIsValid();
        if (returnlist.size()>0 ) {
            std::cout<<"\npattern valid at"<<std::endl;
            for ( auto it =returnlist.begin(); it!=returnlist.end();it++ ) std::cout<<*it<<std::endl;

        }
        //std::cout<<"stored basic interactions: "<<receiver->receivedElements.size()<<std::endl;
        loop_rate.sleep();
     }

   /*PatternMatcher* matcher = new PatternMatcher(4000);
   std::vector<PatternEvent> sittingPatternElements;

   sittingPatternElements.push_back( PatternEvent( EventValues::handsOff  , 100, 0.2 ,1,"-1" ) );
   sittingPatternElements.push_back( PatternEvent( EventValues::handsOn  , 300, 0.6 ,1,"sequence" ) );
   sittingPatternElements.push_back( PatternEvent( EventValues::leaningBack  , 200, 0.1 , -1,"sequence" ) );
   sittingPatternElements.push_back( PatternEvent( EventValues::sittingMid  , 200, 0.2 , 1,"or" ) );

   std::vector<PatternEvent> patr = {  };
   Pattern* takingPattern = new Pattern ( sittingPatternElements );
   matcher->addPattern(takingPattern);

   ros::Rate loop_rate(10);

   //eventVector* vec = new eventVector(5000);vec->spin();
   while (ros::ok()){

      matcher->spin();
       PatternsFound p = matcher->CalculatePatternWeights( *takingPattern );
       for ( auto it =p.totalWeightList.begin(); it!=p.totalWeightList.end();it++ ) std::cout<<*it<<std::endl;
       loop_rate.sleep();

   }*/


}
