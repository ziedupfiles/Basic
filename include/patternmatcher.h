#ifndef PATTERNMATCHER_H
#define PATTERNMATCHER_H
#include <include/eventvector.h>
#include <include/pattern.h>
#include <vector>

class PatternsFound {
public:
    std::vector<float> totalWeightList;
    std::vector<int> startTime;
    std::vector<int> endTime;
};

class PatternMatcher{
public:
    PatternMatcher(int vectorDuration= 1000);
    void addPattern(Pattern* pattern);
    PatternsFound CalculatePatternWeights( Pattern pattern );
    void spin();
private:
    void getWeightList(PatternEvent patternElement,std::vector<eventElement*> PipeLine, std::vector<eventElement*>& preElementList, std::vector<float>& totalWeightList);
    std::vector<eventElement*> returnAll(std::vector<eventElement*> PipeLine, EventValues patternElementVal, long int duration = -1, long int = -1, int direction = 2);
    std::vector<eventElement*> returnAllNext(std::vector<eventElement*> PipeLine, PatternEvent patternElement, long int duration = -1, long int = -1, int direction = 2);

    //eventVector* msgVector;
    std::vector<Pattern*> patternList;
    eventVector* receiver;

    template <typename Enumeration>
    auto as_integer(Enumeration const value)
    -> typename std::underlying_type<Enumeration>::type
    {
        return static_cast<typename std::underlying_type<Enumeration>::type>(value);
    }

};




#endif
