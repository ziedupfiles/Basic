#ifndef PATTERN_H
#define PATTERN_H

#include "include/eventvector.h"
#include <string>
#include <vector>
class PatternEvent {
public:
    PatternEvent();
    PatternEvent(EventValues typ = EventValues::EVENT_DEFAULT, long int  Time = 0, float weight = 0, int sign=1,std::string linker="-1");

    EventValues EventType;
    long int  timeDuration;
    float weight;
    int sign;
    std::string linkPreElement;

    bool operator ==(const eventElement& d) {
        if (this->EventType == d.EventType) {
            return true;
        }
        return false;
    }

};

class Pattern{
public:
    Pattern();
    Pattern(std::vector<PatternEvent> elements );
    void addElement(PatternEvent event);
    std::vector<PatternEvent> getPattern();
private:
    std::vector<PatternEvent> pattern;
};

#endif
