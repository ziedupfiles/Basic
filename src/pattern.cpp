#include "include/pattern.h"

PatternEvent::PatternEvent() {
    this->EventType = EventValues::EVENT_DEFAULT;
    this->timeDuration = 0;
    this->weight = 0;
    this->sign = 1;
};

PatternEvent::PatternEvent(EventValues typ, long int Time, float weight, int sign,std::string linker) {
    this->EventType = typ;
    this->timeDuration = Time;
    this->weight = weight;
    this->sign = sign;
    this->linkPreElement = linker;
};

std::vector<PatternEvent> Pattern::getPattern(){
    return this->pattern;
}

Pattern::Pattern(std::vector<PatternEvent> elements ){
    for (auto it = elements.begin();it!= elements.end();it++ ) this->pattern.push_back(*it);

}
void Pattern::addElement(PatternEvent event){
    this->pattern.push_back(event);
}
