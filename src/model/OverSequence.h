#ifndef OVERSEQUENCE_H
#define OVERSEQUENCE_H

#include <vector>
#include <cmath>
#include "IOverSequence.h"
#include<memory>
namespace car
{

class OverSequence : public IOverSequence
{
public:
	OverSequence() {}
	OverSequence(int inputNums): m_numInputs(inputNums)
	{

	}

	void Insert(std::shared_ptr<std::vector<int> > uc, int index) override;
	
	void GetFrame(int frameLevel, std::vector<std::shared_ptr<std::vector<int> > >& out) override;
	
	bool IsBlockedByFrame(std::vector<int>& state, int frameLevel) override;
	
	int GetLength() override;

private:
	bool IsImply ( std::vector<int>& v1,  std::vector<int>& v2);

	bool comp (int i, int j)
	{
		return abs (i) < abs(j);
	}

	bool comp (std::vector<int>& a, std::vector<int>& b)
	{
		if (a.size() < b.size())
		{
			return true;
		}
		else if (a.size() > b.size())
		{
			return false;
		}

		for (int i = 0; i < a.size(); ++i)
		{
			if (abs(a[i]) < abs(b[i]))
			{
				return true;
			}
			else if (abs(a[i]) > abs(b[i]))
			{
				return false;
			}
		}
		return true;
	}
	
	int GetInsertPos(std::vector<int>& uc, int frameLevel)
	{
		int left = 0, right = m_sequence[frameLevel].size()-1;
		if (right == -1)
		{
			return 0;
		}
		int mid;
		while (left < right)
		{
			mid = (left + right)/2;
			if (comp(*m_sequence[frameLevel][mid], uc))
			{
				left = mid + 1;
			}
			else
			{
				right = mid - 1;
			}
		}
		if (comp(*m_sequence[frameLevel][left], uc))
		{
			return left+1;
		}
		else
		{
			return left;
		}
	}

	int m_numInputs;
	std::vector<std::vector<std::shared_ptr<std::vector<int> > > > m_sequence;//frameLevel //uc //literal m_sequence[0][2]
	//for()

};

}//namespace car



#endif
