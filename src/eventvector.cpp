#include "include/eventvector.h"

eventElement::eventElement() {
    this->EventType = EventValues::EVENT_DEFAULT;
    this->timeStamp = 0;
    this->confidence = 0;
};
eventElement::eventElement(EventValues typ, long int Time, float conf) {
    this->EventType = typ;
    this->timeStamp = Time;
    this->confidence = conf;
};
void eventVector::spin(){
    ros::spinOnce();
}
eventVector::eventVector(int duration){
    char** a;
    int b=0;
    ros::init(b, a,"receiever");
    std::cout<<"eventVector init"<< std::endl;
    this->vectorDuration = duration;
    this->eventSub = this->n.subscribe(this->topic,100, &eventVector::callBack, &(*this));


};

void eventVector::callBack(const vehicle_msgs::BasicInteraction& msg){
    //std::cout<<"msg received"<< msg.name<<std::endl;
    int msgType=0;
    for (int i=0;i<sizeof(this->subTypes);i++ ){
        if ( msg.name == this->subTypes[i] ) {
            msgType=i;
            //std::cout<<"subtype "<<i<<std::endl;
            break;
        }
    }

    //long int ms = std::chrono::duration_cast< std::chrono::milliseconds >( std::chrono::system_clock::now().time_since_epoch() ).count();
    eventElement* newEle = new eventElement(EventValues(msgType),msg.time, float(msg.percentage[0])/100 );
    this->receivedElements.push_back(  newEle );
    this->checkTime();

};

void eventVector::addElement(eventElement* element){
    this->receivedElements.push_back(element);
    //std::cout<<"added msg "<<this->receivedElements.size()<<std::endl;

}
void eventVector::checkTime(){
    //long int lastTime = std::chrono::duration_cast< std::chrono::milliseconds >( std::chrono::system_clock::now().time_since_epoch() ).count();
    //std::cout<<"checkTime   "<< lastTime <<" duration : "<<this->vectorDuration<<std::endl;
    long int lastTime = std::chrono::duration_cast< std::chrono::milliseconds >( std::chrono::system_clock::now().time_since_epoch() ).count();
    this->newreceivedElements = std::vector<eventElement*> { };
    for (auto it = this->receivedElements.begin();it!=this->receivedElements.end();it++)
    {

        if ( abs(lastTime - (*it)->timeStamp) <this->vectorDuration or lastTime == (*it)->timeStamp ) {
            this->newreceivedElements.push_back(*it);

        }
        //else std::cout<<"msg not saved "<<lastTime << " " << (*it)->timeStamp<<std::endl;
    }

    this->receivedElements.swap( this->newreceivedElements );
}

std::vector<eventElement*> eventVector::returnVector(){
    //this->checkTime();
    std::vector<eventElement* > temp;
    temp= this->receivedElements;
    return temp;
}
