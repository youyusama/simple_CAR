#include "OverSequenceForProp.h"

namespace car
{
    void OverSequenceForProp::Insert(std::shared_ptr<std::vector<int> > uc, int index) 
	{
		if (index >= m_unprop.size())
		{
			m_unprop.emplace_back(std::vector<std::shared_ptr<std::vector<int> > >());
            m_prop.emplace_back(std::vector<std::shared_ptr<std::vector<int> > >());
		}
		std::vector<std::shared_ptr<std::vector<int> > > tmp;
		tmp.reserve(m_unprop[index].size()+1);
		for (int i = 0; i < m_unprop[index].size(); ++i)
		{
			if (!IsImply(*m_unprop[index][i], *uc))
			{
				tmp.push_back(m_unprop[index][i]);
			}
		}
		m_unprop[index].swap(tmp);
        
        tmp.clear();
        for (int i = 0; i < m_prop[index].size(); ++i)
		{
			if (!IsImply(*m_prop[index][i], *uc))
			{
				tmp.push_back(m_prop[index][i]);
			}
		}
		m_prop[index].swap(tmp);
		m_unprop[index].push_back(uc);
	}

	void OverSequenceForProp::GetFrame(int frameLevel, std::vector<std::shared_ptr<std::vector<int> > >& out) 
	{
		if (frameLevel >= m_prop.size()) return;
        std::vector<std::shared_ptr<std::vector<int> > > tmp;
        tmp.reserve(m_prop[frameLevel].size() + m_unprop[frameLevel].size());
        tmp.insert(tmp.begin(), m_unprop[frameLevel].begin(), m_unprop[frameLevel].end());
        tmp.insert(tmp.end(), m_prop[frameLevel].begin(), m_prop[frameLevel].end());
		out.swap(tmp);
	}

	bool OverSequenceForProp::IsBlockedByFrame(std::vector<int>& state, int frameLevel)
	{
		int index;
		for (int i = 0; i < m_prop[frameLevel].size(); ++i)//for each uc
		{
			bool isBlockedByUc = true;
			for (int j = 0; j < m_prop[frameLevel][i]->size(); ++j)//for each literal in uc
			{
				index = abs((*m_prop[frameLevel][i])[j]) - m_numInputs - 1;
				if (state[index] != (*m_prop[frameLevel][i])[j])
				{
					isBlockedByUc = false;
					break;
				}
			}
			if (isBlockedByUc)
			{
				return true;
			}
		}

        for (int i = 0; i < m_unprop[frameLevel].size(); ++i)//for each uc
		{
			bool isBlockedByUc = true;
			for (int j = 0; j < m_unprop[frameLevel][i]->size(); ++j)//for each literal in uc
			{
				index = abs((*m_unprop[frameLevel][i])[j]) - m_numInputs - 1;
				if (state[index] != (*m_unprop[frameLevel][i])[j])
				{
					isBlockedByUc = false;
					break;
				}
			}
			if (isBlockedByUc)
			{
				return true;
			}
		}

		return false;
	}
	
	int OverSequenceForProp::GetLength()
	{
		return m_unprop.size();
	}

    bool OverSequenceForProp::IsImply ( std::vector<int>& v1,  std::vector<int>& v2)
	{

		if (v1.size () < v2.size ())
			return false;
		std::vector<int>::iterator first1 = v1.begin (), first2 = v2.begin (), last1 = v1.end (), last2 = v2.end ();
		while (first2 != last2) 
		{
			if ( (first1 == last1) || comp (*first2, *first1) ) 
				return false;
			if ((*first1) == (*first2)) 
				++ first2;
			++ first1;
		}
		return true;
	}
}//namespace car