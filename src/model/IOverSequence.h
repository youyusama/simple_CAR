#ifndef IOVERSEQUENCE_H
#define IOVERSEQUENCE_H

#include <vector>
#include <memory>
namespace car
{

class IOverSequence
{
public:
    virtual void Insert(std::shared_ptr<std::vector<int> > uc, int index) = 0;
	
	virtual void GetFrame(int frameLevel, std::vector<std::shared_ptr<std::vector<int> > >& out) = 0;
	
    virtual bool IsBlockedByFrame(std::vector<int>& state, int frameLevel, bool isPartial) = 0;
	
	virtual int GetLength() = 0;

	int effectiveLevel;
	bool isForward = false;
};

}//namespace car
#endif