

#include "include/patternmatcher.h"
PatternMatcher::PatternMatcher(int duration){
    this->receiver = new eventVector(duration);
}
void PatternMatcher::addPattern(Pattern* pattern){
    this->patternList.push_back(pattern);
}
void PatternMatcher::getWeightList(PatternEvent patternElement,std::vector<eventElement*> PipeLine, std::vector<eventElement*>& preElementList, std::vector<float>& totalWeightList) {
    int indx = 0;
    std::vector<eventElement* > newElementList;
    //std::vector<eventElement> PipeLine  = this->receiver->returnVector();

    std::vector<float> newWeightList;

    if (preElementList.size() == 0) {
        newElementList = this->returnAll(PipeLine, patternElement.EventType);
        for (auto ele = newElementList.begin(); ele != newElementList.end(); ele++) {
            newWeightList.push_back((*ele)->confidence * patternElement.weight * patternElement.sign);
        }

    }

    for (auto preElement = preElementList.begin(); preElement != preElementList.end();preElement++){
        std::vector<eventElement*> list2 = this->returnAll(PipeLine, (*preElement)->EventType , 0, (*preElement)->timeStamp, 2);
        std::vector<eventElement*> allElements;

        if (patternElement.linkPreElement == "or")
            allElements = this->returnAllNext(PipeLine, patternElement, patternElement.timeDuration, (*preElement)->timeStamp, 2);

        else if (patternElement.linkPreElement == "and")
            allElements = this->returnAllNext(PipeLine, patternElement, patternElement.timeDuration, (*preElement)->timeStamp, 2);

        else if (patternElement.linkPreElement == "sequence")
            allElements  = this->returnAllNext(PipeLine, patternElement, patternElement.timeDuration, (*preElement)->timeStamp, 1);

        if (allElements.size() > 0) {
            for (auto ele = allElements.begin(); ele != allElements.end(); ele++) {
                if ((patternElement.linkPreElement == "or") or ((patternElement.linkPreElement == "and" or patternElement.linkPreElement == "sequence") and (list2).size() > 0)) {
                    newElementList.push_back(*ele);
                    newWeightList.push_back(totalWeightList[indx] + (*ele)->confidence * patternElement.weight * patternElement.sign);
                    //std::cout << " totalWeightList[indx] " << totalWeightList[indx] << "ele->confidence  " << ele->confidence << "patternElement.weight " << patternElement.weight << "patternElement.sign " << patternElement.sign << std::endl;
                    //std::cout << "adding " << totalWeightList[indx] + ele->confidence * patternElement.weight * patternElement.sign << std::endl;
                }
                else {
                    eventElement* tempele = new eventElement(EventValues::EVENT_DEFAULT, (*preElement)->timeStamp,0);
                    newElementList.push_back(tempele);
                    newWeightList.push_back(totalWeightList[indx]);
                }
            }
        }

        else{
            eventElement* tempele = new eventElement(EventValues::EVENT_DEFAULT, (*preElement)->timeStamp,0);
            newElementList.push_back(tempele);
            newWeightList.push_back(totalWeightList[indx]);
            //std::cout<<"***************"<<std::endl;
        }
    indx = indx+1;
    }
    //std::cout<<"new weight list size "<<newWeightList.size()<<" old size "<<totalWeightList.size()<<std::endl;
    totalWeightList = newWeightList;
    std::cout<<"new weight list size "<<newWeightList.size()<<" old size "<<totalWeightList.size()<<std::endl;
    preElementList.swap(newElementList);
}

PatternsFound PatternMatcher::CalculatePatternWeights( Pattern pattern ){
    //std::cout<<pattern.getPattern().size()<<std::endl;
    //std::cout<<receiver->receivedElements.size() <<std::endl;

    std::vector<eventElement* > preElement;
    std::vector<float> totalWeightList;
    std::vector<PatternEvent> currentPattern = pattern.getPattern();
    //std::cout<<"curr patt " <<currentPattern.size()<<std::endl;
    std::vector<eventElement* > PipeLine  = this->receiver->receivedElements;

    for (long i = 0; i < currentPattern.size(); i++) {
        PatternEvent patternElement = currentPattern[i];


        std::cout << this->as_integer (patternElement.EventType)<<" TotalWeights{ ";
        for (auto element = totalWeightList.begin(); element != totalWeightList.end(); element++) {
            std::cout << *element << " ";
        }

        std::cout << " } Prelement{ ";
        for (auto element = preElement.begin(); element != preElement.end(); element++) {
            std::cout << this->as_integer( (*element)->EventType) << " ";
        }
        std::cout << " } "<<std::endl;

        getWeightList(patternElement, PipeLine, preElement, totalWeightList);

    }
    PatternsFound patternList;
    patternList.totalWeightList = totalWeightList;


    return patternList;
}


//find all/or previous elements
std::vector<eventElement*> PatternMatcher::returnAll(std::vector<eventElement*> PipeLine, EventValues patternElementVal, long int duration , long int timeStamp , int direction) {
    std::vector<eventElement*> newElementList;

    for (auto element = PipeLine.begin(); element != PipeLine.end(); element++) {
        if ((*element)->EventType == patternElementVal) {
            if ( (direction == 2 and (abs((*element)->timeStamp - timeStamp) <= duration or duration == -1) ) || (direction == 1 and ((*element)->timeStamp - timeStamp >= duration or ((*element)->timeStamp >= timeStamp and duration == -1)) ) ) {
                newElementList.push_back(*element);
                //std::cout<<"*-*-*-*-*-"<<std::endl;
            }

        }
    }

    return newElementList;
}
//find all/or following elements
std::vector<eventElement*> PatternMatcher::returnAllNext(std::vector<eventElement*> PipeLine, PatternEvent patternElement, long int duration , long int timeStamp , int direction ) {
    std::vector<eventElement*> newElementList;

    for (auto element = PipeLine.begin(); element != PipeLine.end(); element++) {
        if ((*element)->EventType == patternElement.EventType) {
            if( (direction == 2 and (abs((*element)->timeStamp - timeStamp) <= duration or duration == -1) ) || (direction == 1 and ((*element)->timeStamp - timeStamp <= duration or ((*element)->timeStamp <= timeStamp and duration == -1)))) {
                //std::cout << "added ele conf" << element->confidence << " " << std::endl;
                newElementList.push_back(*element);
            }

        }
    }

    return newElementList;
}
void PatternMatcher::spin(){

    receiver->spin();
}
