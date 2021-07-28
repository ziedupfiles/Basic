#ifndef NEWPATTERN_H
#define NEWPATTERN_H

#include "include/eventvector.h"
#include <string>
#include <vector>
class newPatternEvent {
public:
    newPatternEvent();
    newPatternEvent(EventValues typ = EventValues::EVENT_DEFAULT, long int  pTime = 0,long int  fTime = 0, float minconf = 0, int sign=1);
    std::vector<eventElement* > getPossiblePlacementInVector(std::vector<eventElement*>* currVector , long int timeStamp);
    std::vector<eventElement* > getAllPossiblePlacementInVectorFromPrevious(std::vector<eventElement*>* currVector,std::vector<eventElement* > Previous );

    EventValues EventType;
    long int  timeDuration;
    long int preTime;
    long int afterTime;
    float minConfidence;
    int sign;

    bool operator ==(const eventElement& d) {
        if (this->EventType == d.EventType) {
            return true;
        }
        return false;
    }

};

class newSubPattern{
public:
    newSubPattern(){};
    newSubPattern(std::vector<newPatternEvent> elements );
    void addElement(newPatternEvent event);

    std::vector<newPatternEvent> getPattern();
    //returns the timestamp list when the pattern is valid
    //set timestamp as 0 for the first subpattern
    //the firt element of the first subpattern should hae linker -1
    std::list<long int> subPatternIsValid(std::vector<eventElement*>* currVector,long int timeStamp=0);
    std::list<long int> subPatternIsValidFromAllPrevious(std::vector<eventElement*>* currVector,std::list<long int>* timeStamps);

private:
    std::vector<newPatternEvent> pattern;
};

class newPattern{
public:
    //newPattern();
    newPattern(std::vector<newSubPattern> elements );
    void addElement(newSubPattern event);
    std::vector<newSubPattern> getPattern();
    std::list<long int> patternIsValid();
    eventVector* vect;
private:
    std::vector<newSubPattern> pattern;
};

#endif
