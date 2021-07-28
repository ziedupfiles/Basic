#include "eventmanager.h"

eventManager::eventManager()
{
    this->wheelSub = this->n.subscribe(this->wheelTopic,100, &eventManager::wheelCallBack, &(*this));
    this->wheelSub = this->n.subscribe(this->mattTopic,100, &eventManager::pressureMattCallback, &(*this));

    for (int i=0;i<7;i++) this->wheelArr[i] =0;
    this->interactionPub = this->n.advertise<vehicle_msgs::BasicInteraction>("BasicInteraction", 1000);
}

void eventManager::publishMsg(std::string name, std::vector<int> vals , std::vector<int> perc){
    vehicle_msgs::BasicInteraction newMsg;
    newMsg.name = name;
    for (auto it = vals.begin();it!=vals.end();it++) newMsg.values.push_back(*it);
    for (auto it = perc.begin();it!=perc.end();it++) newMsg.percentage.push_back(*it);
    this->interactionPub.publish(newMsg);

}

void eventManager::wheelCallBack(const vehicle_msgs::FloatArray& msg ){
    float smgArr[16];
    for ( int i =0; i<16;i++){
        smgArr[i] = msg.data[i];
    }

    float tempArr[8];
    int front[] ={1,2,10,9,13,14,6,5};
    for( auto i=0;i<8;i++) tempArr[i] =(smgArr[ front[i] ]+smgArr[ front[i]+2] );
    bool update = false;
    update= update or this->handsOn(tempArr);
    //update = update or this->handsOff(tempArr);
    this->handsNumber(tempArr);

    if (update) for( auto i=0;i<8;i++) this->wheelArr[i]=tempArr[i];
    //for(int i=0; i<8; i++) tempArr[i] = this->wheelArr[i];


}
float eventManager::getMax(float tempArr [8] ){
    float max = tempArr[0];
    for( auto i=0;i<8;i++) {
        if(tempArr[i]>max) max = tempArr[i];
    }
    return max;
}

int eventManager::getHandsOnProb(float pressure){
   int temp =0;
    if ( pressure > this->wheelMin ){
        temp = int (100*(pressure-this->wheelMin)/(wheelMaxHandsOn-this->wheelMin) );
        if( pressure>this->wheelMaxHandsOn)  temp=100;
    }

   return temp;
}

int eventManager::getHandsOffProb(float pressure){
   int temp =0;
    if ( pressure < this->wheelMax ) {
        temp = int (100- 100*pressure/this->wheelMin );
        if( pressure>this->wheelMaxHandsOn)  temp=100;
    }
   if( temp<0) temp=0;
   if (temp>100)  temp=100;
   return temp;
}

bool eventManager::handsOn(float tempArr[8]){
    float oldMax = this->getMax( this->wheelArr);
    float max = this->getMax(tempArr);
    bool send = false;
    //only send a message if val has changed more than 3

    if(  abs(oldMax-max)>3  and abs( getHandsOnProb(max) - getHandsOnProb(oldMax) )>percUpdate  ) {
        this->publishMsg("HandsOnWheel",std::vector<int> {1},std::vector<int> {this->getHandsOnProb(max)} );
        send = true;
    }
    if ( abs(oldMax-max)>3 and max<this->wheelMaxHandsOn and abs( getHandsOnProb(max) - getHandsOnProb(oldMax) )>percUpdate ) {
        this->publishMsg("HandsOffWheel",std::vector<int> {1},std::vector<int> {this->getHandsOffProb(max)} );
        send= true;
    }
    return send;
}
bool myfunction (int i,int j) { return (i<j); }
void eventManager::handsNumber(float tempArr[8]){

    vehicle_msgs::BasicInteraction newMsg = this->getHandsNumberMsg(tempArr);
    vehicle_msgs::BasicInteraction oldMsg = this->getHandsNumberMsg(this->wheelArr);
    std::vector<int> newMsgSorted;
    std::vector<int> oldMsgSorted;

    for (int i = 0;i <newMsg.percentage.size();i++ ) newMsgSorted.push_back(newMsg.percentage[i]);
    for (int i = 0;i <oldMsg.percentage.size();i++ ) oldMsgSorted.push_back(oldMsg.percentage[i]);

    std::sort (newMsgSorted.begin(), newMsgSorted.end(), myfunction);
    std::sort (oldMsgSorted.begin(), oldMsgSorted.end(), myfunction);

    if( newMsg.values[0]!=newMsg.values[0]) this->interactionPub.publish(newMsg);
    else {
        for ( int  i=0;i<newMsg.percentage.size();i++) {
            if( abs(newMsgSorted[i]-oldMsgSorted[i])>5 ){
                 this->interactionPub.publish(newMsg);
                break;
            }
        }
    }
    //this->interactionPub.publish(newMsg);
}

vehicle_msgs::BasicInteraction eventManager::getHandsNumberMsg(float tempArr[8]){

    std::vector<int> usedPoints;
    std::vector< std::vector<int> > groups;
    for ( int i=0;i<8;i++){
        if ((tempArr[i]>this->wheelMin) and (std::find(usedPoints.begin(), usedPoints.end(), i) != usedPoints.end())) {

            std::vector<int> newgroup;

            newgroup.push_back(i);
            usedPoints.push_back(i);

            //check the left elements
            while (true){
                int j= i-1;
                j=j %8;
                if (j==i) break;
                if(( tempArr[i]>this->wheelMin) and (std::find(usedPoints.begin(), usedPoints.end(), i) != usedPoints.end() )) {
                    newgroup.push_back(j);
                    usedPoints.push_back(j);

                }
            }
            //check the right elements
            while (true){
                int j= i+1;
                j=j %8;
                if (j==i) break;
                if(( tempArr[i]>this->wheelMin) and (std::find(usedPoints.begin(), usedPoints.end(), i) != usedPoints.end() )) {
                    newgroup.push_back(j);
                    usedPoints.push_back(j);

                }
            }
        //push the newgroup
        groups.push_back(newgroup);

        }
    }
    int numberOfGroups = groups.size();
    vehicle_msgs::BasicInteraction newMsg;
    newMsg.name = "HandsOnWheel";
    for (auto it =groups.begin();it!=groups.end();it++){
        if( (*it).size()>3) numberOfGroups = numberOfGroups+1;
        float max = 0;
        for (auto jt=(*it).begin(); jt!=(*it).end();jt++ ){
            if (max> this->getHandsOnProb(tempArr[*jt])) max= this->getHandsOnProb(tempArr[*jt]);
        }

        newMsg.percentage.push_back( max);
    }
    newMsg.values.push_back(numberOfGroups);
    return newMsg;
}

void eventManager::pressureMattCallback(const vehicle_msgs::SeatPressureSensor& msg){
    //vehicle_msgs::PressureMat backPressure= msg.seatBackSensor; //= msg.seatBackSensor;
    this->newMatPress = msg;

    vehicle_msgs::PressureMat backPressure= this->newMatPress.seatBackSensor;
    vehicle_msgs::PressureMat seatPressure= this->newMatPress.seatSensor;

    vehicle_msgs::PressureMat oldBackPressure= this->oldMatPress.seatBackSensor;
    vehicle_msgs::PressureMat oldSeatPressure= this->oldMatPress.seatSensor;

    float backPressureArr[32][32];
    float seatPressureArr[32][32];
    float oldBackPressureArr[32][32];
    float oldSeatPressureArr[32][32];
    for ( int i = 0;i<32;i++){
        for (int j =0;j<32;j++){
            backPressureArr[i][j] = backPressure.rows[i].data[j];
            seatPressureArr[i][j] = seatPressure.rows[i].data[j];
            oldBackPressureArr[i][j] = oldBackPressure.rows[i].data[j];
            oldSeatPressureArr[i][j] = oldSeatPressure.rows[i].data[j];
        }
    }
    this->leaningBack(backPressureArr,oldBackPressureArr);


    this->oldMatPress = msg;
}


float eventManager::getAvg(float arr[32][32]){
    float avg =0;
    for ( int i = 0;i<32;i++){
        for (int j =0;j<32;j++) avg= avg+ arr[i][j];
    }
    avg =avg /(32*32);
    return avg;
}
void eventManager::leaningBack(float arr[32][32] , float arrOld[32][32]){
    float newAvg = this->getAvg(arr);
    float oldAvg = this->getAvg(arrOld);

    int perc =0;
    if( abs(newAvg-oldAvg)>10 ){
        if (newAvg> this->backMax ) perc =100;
        else if(newAvg> this->backMin ) perc = 100*newAvg/this->backMax;
    }

    int val = 1;
    if( perc ==0) val=0;
    this->publishMsg("LeaningBack",std::vector<int> {val},std::vector<int> {perc} );


}

void eventManager::pressureMattNormCallback(const vehicle_msgs::SeatPressureSensor &msg){
    this->newMatPressNorm = msg;
    float newArr [32][mattYlength];
    float oldArr [32][mattYlength];
    vehicle_msgs::PressureMat seatPressure = this->newMatPressNorm.seatSensor;
    vehicle_msgs::PressureMat oldSeatPressure = this->oldMatPressNorm.seatSensor;

    for ( int i = 0;i<32;i++){
        for (int j =0;j<mattYlength;j++){
            newArr[i][j] = seatPressure.rows[i].data[j];
            oldArr[i][j] = oldSeatPressure.rows[i].data[j];
        }
    }
    bool update = this->centerOfPressureMsg( newArr ,oldArr);
    if (update) this->oldMatPressNorm = msg;
}

bool eventManager::centerOfPressureMsg(float newArr[32][mattYlength], float oldArr[32][mattYlength]){
    std::vector<int> oldCenter = this->getCenterOfPressure(oldArr);
    std::vector<int> newCenter = this->getCenterOfPressure(newArr);
    bool update = false;
    if ( abs(oldCenter[0]-newCenter[0])>2 or abs(oldCenter[1]-newCenter[1])>2 ){
        std::vector<int>  newMidConf = sittingConfidence(newCenter);
        std::vector<int>  oldMidConf = sittingConfidence(oldCenter);
        std::string topics[5] ={"SittingMid","SittingRight","SittingLeft","SittingForward","SittingBack"};

        for (int i=0;i<5;i++) {
            if (abs(newMidConf[i]-oldMidConf[i])>percUpdate){
                this->publishMsg(topics[i],std::vector<int> {1},std::vector<int> {newMidConf[i]} );
                update = true;
            }
        }
    }
    return update;
}

std::vector<int> eventManager::getCenterOfPressure(float arr[30][mattYlength] ){
    std::vector<int> pos;
    float cx; float cy; float m;
    for(int x = 0; x < 30; x++ ) {
      for(int y = 0; y < mattYlength; y++) {
        cx += arr[x][y] * x;
        cy += arr[x][y] * y;
        m += arr[x][y];
      }
    }
    pos.push_back( cx/m);
    pos.push_back( cy/m);
    return pos;
}

int eventManager::sittingForwardConfidence(  std::vector<int> pos  ){
    int temp=0;
    int mid = mattYlength/2;
    if( pos[1]>mid ) temp = 100*( pos[1]-mid )/mid ;
    return  temp;
}
int eventManager::sittingBackConfidence(  std::vector<int> pos  ){
    int temp=0;
    int mid = mattYlength/2;
    if( pos[1]<mid ) temp = 100*( mid - pos[1] )/mid ;
    return  temp;
}
int eventManager::sittingRightConfidence(  std::vector<int> pos  ){
    int temp=0;
    int mid = 32/2;

    if( abs(pos[0]- rightSide) < mid ) temp = 100*( mid - abs(pos[0]- rightSide) )/mid ;
    return  temp ;
}
int eventManager::sittingLeftConfidence(  std::vector<int> pos  ){
    int temp=0;
    int mid = 32/2;

    if( abs(  pos[0]- abs(rightSide-32) ) < mid ) temp = 100*( mid - abs(pos[0]- abs(rightSide-32) )) /mid ;
    return  temp ;
}

std::vector<int> eventManager::sittingConfidence(  std::vector<int> pos  ){
    int right = this->sittingRightConfidence(pos);
    int left = this->sittingLeftConfidence(pos);
    int forward = this->sittingForwardConfidence(pos);
    int back = this->sittingBackConfidence(pos);
    int midx = 100-(right+left);
    int midy = 100-(forward+back);
    return std::vector<int> {(midx*midy)/100,right,left,forward,back };

}
