#include "include/basicinteraction.h"

Basic::Basic( )
{

}
void Basic::setVals(std::vector<bool> sendMsgs ,float backMin ,float backMax ,float wheelHandsOnMin,float wheelHandsOnMax,float wheelLooseMin,float wheelLooseMax,float wheelTightMin,float wheelTightMax ,int rightSideMax,int frontSideMax){
    this->sendHandsOn = sendMsgs[0];
    this->sendHandsOff= sendMsgs[1];
    this->sendLeaningBack= sendMsgs[2];
    for (int i =0;i<5;i++){
        this->sendSitting[i]=sendMsgs[i+3];
    }

    this->backMin = backMin ;
    this->backMax= backMax;
    this->wheelHandsOnMin= wheelHandsOnMin;
    this->wheelHandsOnMax= wheelHandsOnMax;
    this->wheelLooseMin= wheelLooseMin;
    this->wheelLooseMax= wheelLooseMax;
    this->wheelTightMin= wheelTightMin;
    this->wheelTightMax= wheelTightMax;
    this->rightSideMax = rightSideMax;
    this->frontSideMax = frontSideMax;



    for (int i=0;i<7;i++) this->wheelArr[i] =0;
    for (int i=0;i<32;i++) for(int j=0;j<32;j++) this->backArr[i][j]=0;
    for (int i=0;i<mattXlength;i++) for(int j=0;j<mattYlength;j++) this->seatArr[i][j]=0;

    this->wheelSub = this->n.subscribe(this->wheelTopic,100, &Basic::wheelCallBack, &(*this));
    this->wheelSub = this->n.subscribe(this->mattTopic,100, &Basic::mattCallBack, &(*this));
    this->interactionPub = this->n.advertise<vehicle_msgs::BasicInteraction>("BasicInteraction", 1000);
//ros::spin();
}

void Basic::spin(){
    ros::spinOnce();
}

void Basic::publishMsg(std::string name, std::vector<int> vals , std::vector<int> perc){
    vehicle_msgs::BasicInteraction newMsg;
    newMsg.name = name;
    for (auto it = vals.begin();it!=vals.end();it++) newMsg.values.push_back(*it);
    for (auto it = perc.begin();it!=perc.end();it++) newMsg.percentage.push_back(*it);
    long int ms = std::chrono::duration_cast< std::chrono::milliseconds >( std::chrono::system_clock::now().time_since_epoch() ).count();
    newMsg.time = ms;
    this->interactionPub.publish(newMsg);
    if( printMsg ) std::cout<< name <<" "<<perc[0]<<std::endl;
}

int Basic::getConf(int value, int startVal , int endVal){
    int temp =0;
    if (startVal<endVal ) {
        if  (value<endVal &&value >startVal)
        temp = 100* (value-startVal)/ abs(startVal-endVal);
        else if ( value>=endVal) temp=100;
        else if (value < startVal ) temp=0;
        }
    else if (startVal>endVal  ) {
        if (value>endVal &&value <startVal)
        temp = 100* abs(value-startVal)/ abs(startVal-endVal);
        else if (value<=endVal) temp=100;
        else if (value >startVal) temp=0;
    }
    return temp;
}

float Basic::getMax(float tempArr [8] ){
    float max = tempArr[0];
    for( auto i=0;i<8;i++) {
        if(tempArr[i]>max) max = tempArr[i];
    }
    return max;
}

void Basic::wheelCallBack(const vehicle_msgs::FloatArray& msg ){
    float smgArr[16];
    for ( int i =0; i<16;i++){
        smgArr[i] = msg.data[i];
    }
    float tempArr[8];
    int front[] ={1,2,10,9,13,14,6,5}; //the sides positions from top-left going clock-wise
    for( auto i=0;i<8;i++) tempArr[i] =(smgArr[ front[i] ]+smgArr[ front[i]+2] );

    bool update=false; //update the wheel array only if a msg was sent

    update =update or this->sendHandsMsg(tempArr);
    if (update) for( auto i=0;i<8;i++) this->wheelArr[i] = tempArr[i];
}

bool Basic::sendHandsMsg(float tempWheelArr[8]){
    bool update = false;
    float newMax = this->getMax( tempWheelArr );
    int newHandsOnConf = this->getConf(newMax,this->wheelHandsOnMin,this->wheelHandsOnMax);
    int newHandsOffConf = this->getConf(newMax,this->wheelHandsOnMin,0);
    if ( abs(newHandsOnConf-this->oldHandsOnConf)>percUpdate or (newHandsOnConf==0 and this->oldHandsOnConf!=0) or (newHandsOnConf==100 and this->oldHandsOnConf!=100) ){
        if (this->sendHandsOn)  this->publishMsg("handsOn",std::vector<int> {1}, std::vector<int> {newHandsOnConf} );
        update = true;
        this->oldHandsOnConf = newHandsOnConf;
    }

    if ( abs(newHandsOffConf-this->oldHandsOffConf)>percUpdate or (newHandsOffConf==0 and this->oldHandsOffConf!=0) or (newHandsOffConf==100 and this->oldHandsOffConf!=100 ) ){
        if (this->sendHandsOff) this->publishMsg("handsOff",std::vector<int> {1}, std::vector<int> {newHandsOffConf} );
        update = true;
        this->oldHandsOffConf = newHandsOffConf;
    }

    return update;
}


void Basic::mattCallBack(const vehicle_msgs::SeatPressureSensor& msg ){
    this->mattMsg = msg;
    this->backMsg = this->mattMsg.seatBackSensor;
    this->seatMsg = this->mattMsg.seatSensor;

    float backPressureArr[32][32];
    for ( int i = 0;i<32;i++){
        for (int j =0;j<32;j++){
            backPressureArr[i][j] = this->backMsg.rows[i].data[j];
        }
    }
    this->sendLeaningBackMsg(backPressureArr);

    float seatPressureArr[mattYlength][mattXlength];
    for ( int i = 0;i<mattYlength;i++){
        for (int j =0;j<32;j++){
            seatPressureArr[i][j] = this->seatMsg.rows[i+mattYstart].data[j];
        }
    }
    this->sendSittingMsg(seatPressureArr);

}

bool Basic::sendLeaningBackMsg( float arr[32][32]){
    float sum=0;
    for (int i=0;i<32;i++) for(int j=0;j<32;j++) sum=sum+arr[i][j];

    int newLeaningBackConf = this->getConf(sum,this->backMin,this->backMax);
    if( abs(this->oldLeaningBackConf-newLeaningBackConf)>percUpdate or (this->oldLeaningBackConf!=0 and newLeaningBackConf==0) or (this->oldLeaningBackConf!=100 and newLeaningBackConf==100) ){
        if (this->sendLeaningBack) this->publishMsg("leaningBack",std::vector<int> {1},std::vector<int> {newLeaningBackConf});
        this->oldLeaningBackConf = newLeaningBackConf;
    }
    return false;
}

bool Basic::sendSittingMsg( float arr[mattYlength][mattXlength]  ){
    bool update = false;
    std::vector<int> sitConf =this->getSittingConf(arr);
    std::string topics[5] ={"sittingMid","sittingRight","sittingLeft","sittingFront","sittingBack"};
    for(int i =0;i<5;i++){
        if( abs(sitConf[i]-this->oldSittingConf[i] )>percUpdate or (sitConf[i]==0 and this->oldSittingConf[i]!=0 ) or (sitConf[i]==100 and this->oldSittingConf[i]!=100 ) ){
            if (this->sendSitting[i]) this->publishMsg(topics[i],std::vector<int> {1},std::vector<int> {sitConf[i]});
            this->oldSittingConf[i] = sitConf[i];
            update = true;
        }
    }
    return update;
}

std::vector<int> Basic::getSittingConf(float arr[mattYlength][mattXlength] ){
    std::vector<int> pos = this->getCenterOfPressure(arr);
    int rightConf = this->getConf( pos[0],abs( mattXlength/2 - this->rightSideMax), this->rightSideMax );
    int leftSide = abs(mattXlength-this->rightSideMax);
    int leftConf = this->getConf( pos[0], abs(mattXlength/2-leftSide ) ,leftSide );

    int frontConf = this->getConf( pos[1], abs(mattYlength/2-this->frontSideMax ) ,this->frontSideMax );
    int backSide= abs(mattYlength-this->frontSideMax);
    int backConf = this->getConf( pos[1], abs(mattYlength/2-backSide ) ,backSide );

    int midConf = (100-rightConf-leftConf)*(100-frontConf-backConf)/100;
    return std::vector<int> { midConf,rightConf,leftConf,frontConf,backConf};
}

std::vector<int> Basic::getCenterOfPressure(float arr[mattYlength][mattXlength]){
    std::vector<int> pos;
    float cx=0; float cy=0; float m=0;
    for(int x = 0; x < mattXlength; x++ ) {
      for(int y = 0; y < mattYlength; y++) {
        cx += arr[y][x] * x;
        cy += arr[y][x] * y;
        m += arr[y][x];
      }
    }
    pos.push_back( cx/m);
    pos.push_back( cy/m);
    return pos;
}
