#ifndef OVERSEQUENCEFORPROP_H
#define OVERSEQUENCEFORPROP_H

#include "IOverSequence.h"
#include <cmath>
#include <memory>
namespace car
{


class OverSequenceForProp: public IOverSequence 
{
public:
	OverSequenceForProp() {}
	OverSequenceForProp(int inputNums): m_numInputs(inputNums)
	{

	}

	void Insert(std::shared_ptr<std::vector<int> > uc, int index) override;
	
	void GetFrame(int frameLevel, std::vector<std::shared_ptr<std::vector<int> > >& out) override;
	
	bool IsBlockedByFrame(std::vector<int>& state, int frameLevel) override;
	
	int GetLength() override;

	std::vector<std::shared_ptr<std::vector<int> > >& GetProp(int frameLevel)
	{
		return m_prop[frameLevel];
	}

	std::vector<std::shared_ptr<std::vector<int> > >& GetUnProp(int frameLevel)
	{
		return m_unprop[frameLevel];
	}

	void InsertIntoProped(std::shared_ptr<std::vector<int> > uc, int index)
	{
		if (index >= m_prop.size())
		{
			m_unprop.emplace_back(std::vector<std::shared_ptr<std::vector<int> > >());
            m_prop.emplace_back(std::vector<std::shared_ptr<std::vector<int> > >());
		}
		std::vector<std::shared_ptr<std::vector<int> > > tmp;
        for (int i = 0; i < m_prop[index].size(); ++i)
		{
			if (!IsImply(*m_prop[index][i], *uc))
			{
				tmp.push_back(m_prop[index][i]);
			}
		}
		m_prop[index].swap(tmp);
		m_prop[index].push_back(uc);
	}

private:
	bool IsImply ( std::vector<int>& v1,  std::vector<int>& v2);

	bool comp (int i, int j)
	{
		return abs (i) < abs(j);
	}
	
	int m_numInputs;
	std::vector<std::vector<std::shared_ptr<std::vector<int> > > > m_unprop;
	std::vector<std::vector<std::shared_ptr<std::vector<int> > > > m_prop;
	//for()
};


	



}//namespace car


#endif