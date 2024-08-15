#ifndef UNDERSEQUENCE_H
#define UNDERSEQUENCE_H

#include "State.h"
#include <iostream>
#include <vector>
namespace car {

class UnderSequence {
  public:
    UnderSequence() {}
    ~UnderSequence() {
        for (int i = 0; i < m_sequence.size(); ++i) {
            for (int j = 0; j < m_sequence[i].size(); ++j) {
                m_sequence[i][j] = nullptr;
            }
        }
    }

    void push(shared_ptr<State> state) {
        while (m_sequence.size() <= state->depth) {
            m_sequence.emplace_back(vector<shared_ptr<State>>());
        }
        m_sequence[state->depth].push_back(state);
    }

    int size() { return m_sequence.size(); }

    vector<shared_ptr<State>> &operator[](int i) { return m_sequence[i]; }

  private:
    vector<vector<shared_ptr<State>>> m_sequence;
};

} // namespace car

#endif