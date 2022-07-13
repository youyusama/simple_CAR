#include "OverSequence.h"

namespace car
{
	void OverSequence::Insert(std::shared_ptr<std::vector<int> > uc, int index) 
	{
		std::sort(uc->begin(), uc->end(), comp_abs);

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
	
	bool OverSequence::IsBlockedByFrame(std::vector<int>& state, int frameLevel)
	{
		blockQuerryTimes ++;
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
				blockedTimes ++;
				return true;
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
		implyQuerryTimes ++;
		if (v1.size () <= v2.size ())
			return false;
		if (std::includes(v1.begin(), v1.end(), v2.begin(), v2.end(), comp_abs)){
			implyTimes ++;
			return true;
		}
		else
			return false;
		// std::vector<int>::iterator iter1 = v1.begin (), iter2 = v2.begin (), last1 = v1.end (), last2 = v2.end ();
		// while (iter2 != last2) 
		// {
		// 	if (abs(*iter2) > abs(*iter1) || (iter1 == last1)) 
		// 		return false;
		// 	if ((*iter1) == (*iter2)) 
		// 		iter2 ++;
		// 	iter1 ++;
		// }
		// return true;
	}

	std::vector<int> OverSequence::GetStat(){
		std::vector<int> temp;
		temp.push_back(blockQuerryTimes);
		temp.push_back(blockedTimes);
		temp.push_back(implyQuerryTimes);
		temp.push_back(implyTimes);
		return temp;
	}

}//namespace car