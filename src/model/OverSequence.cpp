#include "OverSequence.h"

namespace car
{
	void OverSequence::Insert(std::shared_ptr<std::vector<int> > uc, int index) 
	{
		

		if (index >= m_sequence.size())
		{
			m_sequence.push_back(std::vector<std::shared_ptr<std::vector<int> > >());
		}
		if (isForward && index == 0)
		{
			m_sequence[0].push_back(uc);
			return;
		}
		std::vector<std::shared_ptr<std::vector<int> > > tmp;
		tmp.reserve(m_sequence[index].size()+1);
		for (int i = 0; i < m_sequence[index].size(); ++i)
		{
			if (!IsImply(*m_sequence[index][i], *uc))
			{
				tmp.push_back(m_sequence[index][i]);
			}
		}
		m_sequence[index].swap(tmp);
		m_sequence[index].push_back(uc);
	}

	/*
    void OverSequence::Insert(std::vector<int>& uc, int index) 
	{
		if (index >= m_sequence.size())
		{
			m_sequence.emplace_back(std::vector<std::vector<int> >());
		}
		int pos = GetInsertPos(uc, index);
		std::vector<std::vector<int> > tmp;
		tmp.reserve(m_sequence[index].size()+1);
		tmp.insert(tmp.begin(), m_sequence[index].begin(), m_sequence[index].begin()+pos);
		tmp.push_back(uc);
		for (int i = pos; i < m_sequence[index].size(); ++i)
		{
			if (!IsImply(m_sequence[index][i], uc))
			{
				tmp.push_back(m_sequence[index][i]);
			}
		}
		m_sequence[index].swap(tmp);
		//m_sequence[index].push_back(uc);
	}
	*/

	void OverSequence::GetFrame(int frameLevel, std::vector<std::shared_ptr<std::vector<int> > >& out) 
	{
		if (frameLevel >= m_sequence.size()) return;
		out = m_sequence[frameLevel];
	}
	
	bool OverSequence::IsBlockedByFrame(std::vector<int>& state, int frameLevel, bool isPartial)
	{
		if(!isPartial)
		{
			int index;
			for (int i = 0; i < m_sequence[frameLevel].size(); ++i)//for each uc
			{
				bool isBlockedByUc = true;
				for (int j = 0; j < m_sequence[frameLevel][i]->size(); ++j)//for each literal in uc
				{
					index = abs((*m_sequence[frameLevel][i])[j]) - m_numInputs - 1;
					if (state[index] != (*m_sequence[frameLevel][i])[j])
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
		}
		else
		{
			for (int i = 0; i < m_sequence[frameLevel].size(); ++i)//for each uc
			{	 
				if (IsImply(state,*(m_sequence[frameLevel][i])))
				{
					return true;
				}
			}	
		}
		
		return false;
	}
	
	int OverSequence::GetLength()
	{
		return m_sequence.size();
	}

    bool OverSequence::IsImply (std::vector<int>& v1,  std::vector<int>& v2) //if v2->v1
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