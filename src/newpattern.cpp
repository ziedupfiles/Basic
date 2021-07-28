#include "include/newpattern.h"

newPatternEvent::newPatternEvent() {
    this->EventType = EventValues::EVENT_DEFAULT;
    this->timeDuration = 0;
    this->minConfidence = 50;
    this->sign = 1;
};

newPatternEvent::newPatternEvent(EventValues typ, long int pTime,long int  fTime, float minconf, int sign) {
    this->EventType = typ;
    this->timeDuration = fTime;
    this->preTime = pTime;
    this->afterTime = fTime;
    this->minConfidence = minconf;
    this->sign = sign;

};

std::vector<eventElement* > newPatternEvent::getPossiblePlacementInVector(std::vector<eventElement*>* currVector, long int timeStamp){
    std::vector<eventElement* > tempList;

    if (this->preTime == 0 && this->afterTime ==0 ){
        for (auto it = (*currVector).begin(); it !=(*currVector).end();it++){

            if ( (*it)->EventType==this->EventType && ( ((*it)->confidence>=this->minConfidence && this->sign==1)||((*it)->confidence<=this->minConfidence &&this->sign==-1)) ){
                //std::cout<<"tempList.push_back"<<(*it)->confidence<<"//";
                if (!( std::find(tempList.begin(), tempList.end(), *it) != tempList.end() ))

                tempList.push_back(*it);
            }
        }
    }
    /*
    else if ( this->linkPreElement=="sequence"){
        for (auto it = (*currVector).begin(); it !=(*currVector).end();it++){
            if ( (*it)->timeStamp>=timeStamp &&(*it)->timeStamp<=timeStamp+this->timeDuration
                 &&(*it)->EventType==this->EventType &&
                 ( ((*it)->confidence>=this->minConfidence && this->sign==1)||((*it)->confidence<=this->minConfidence &&this->sign==-1))){
                //if (!( std::find(tempList.begin(), tempList.end(), *it) != tempList.end() ))
                {std::cout<<"*"<<(*it)->confidence<<"at "<<(*it)->timeStamp<<"-"<<timeStamp <<"//";tempList.push_back(*it);}
            }
        }
    }*/

    else {
        for (auto it = (*currVector).begin(); it !=(*currVector).end();it++){
            if ( (*it)->timeStamp>=timeStamp-this->preTime &&(*it)->timeStamp<=timeStamp+this->afterTime

                 &&(*it)->EventType==this->EventType
                 &&( ((*it)->confidence>=this->minConfidence && this->sign==1)||((*it)->confidence<=this->minConfidence &&this->sign==-1))){
                //if (!( std::find(tempList.begin(), tempList.end(), *it) != tempList.end() ))
                tempList.push_back(*it);
            }
        }
    }
    return tempList;

}
std::vector<eventElement* > newPatternEvent::getAllPossiblePlacementInVectorFromPrevious(std::vector<eventElement*>* currVector,std::vector<eventElement* > Previous ){
   std::vector<eventElement* > returnList;
   for ( auto it = Previous.begin(); it!= Previous.end();it++){
       std::vector<eventElement* > tempList = this->getPossiblePlacementInVector(currVector,(*it)->timeStamp);
       for (auto jt =tempList.begin();jt!=tempList.end();jt++ ){
           if (!( std::find(returnList.begin(), returnList.end(), *jt) != returnList.end() ))
               returnList.push_back(*jt);


       }
   }
    return returnList;
}

std::vector<newPatternEvent> newSubPattern::getPattern(){
    return this->pattern;
}

newSubPattern::newSubPattern(std::vector<newPatternEvent> elements ){
    for (auto it = elements.begin();it!= elements.end();it++ ) this->pattern.push_back(*it);

}
void newSubPattern::addElement(newPatternEvent event){
    this->pattern.push_back(event);
}

std::list<long int> newSubPattern::subPatternIsValid(std::vector<eventElement*>* currVector, long int timeStamp){
    std::vector<eventElement*> Previous;
    for ( auto it = this->pattern.begin(); it != this->pattern.end();it++){
        if ( it==this->pattern.begin()) Previous = (*it).getPossiblePlacementInVector( currVector,timeStamp);
        else Previous = (*it).getAllPossiblePlacementInVectorFromPrevious(currVector,Previous);

    }
    std::list<long int > returnList;
    for ( auto it=Previous.begin();it!=Previous.end();it++) returnList.push_back((*it)->timeStamp);
    return  returnList;
}
std::list<long int> newSubPattern::subPatternIsValidFromAllPrevious(std::vector<eventElement*>* currVector, std::list<long int>* timeStamps){
    std::list<long int> newStamp;
    for ( auto it = timeStamps->begin();it != timeStamps->end();it++){
        std::list<long int>   temp = this->subPatternIsValid(currVector,*it);

        for (auto j=temp.begin();j!=temp.end();j++)
            if (!( std::find(newStamp.begin(), newStamp.end(), *j) != newStamp.end() ))
            newStamp.push_back(*j);
    }
    return newStamp;
}
newPattern::newPattern(std::vector<newSubPattern> elements){
    for ( int i =0;i<elements.size();i++){
        this->pattern.push_back(elements[i]);
    }
}

std::list<long int> newPattern::patternIsValid(){
    std::vector<eventElement*> currVector= vect->returnVector();

    std::list<long int>  timeStamps;

    for (auto subPatterns = this->pattern.begin();subPatterns!=this->pattern.end();subPatterns++){
        //first subpattern can start at any time
        if (subPatterns== this->pattern.begin() ) {
            timeStamps = subPatterns->subPatternIsValid(&currVector);
        //std::cout<<"timeStamps.size()"<<timeStamps.size()<<std::endl;
        }
        //the others must come after each other
        else {
            //std::cout<<"2nd sub Previous.size()"<<timeStamps.size()<<std::endl;
            if (timeStamps.size()==0) return std::list<long int>  {};
            else timeStamps = subPatterns->subPatternIsValidFromAllPrevious(&currVector,&timeStamps);
        }
    }
    return timeStamps;
}



