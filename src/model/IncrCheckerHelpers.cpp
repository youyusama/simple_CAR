#include "IncrCheckerHelpers.h"

namespace car {

Branching::Branching(int type) {
    branching_type = type;
    conflict_index = 1;
    mini = 1 << 20;
    counts.clear();
}


Branching::~Branching() {}


void Branching::Update(const shared_ptr<cube> uc) {
    if (uc->size() == 0) return;
    conflict_index++;
    switch (branching_type) {
    case 1: {
        Decay();
        break;
    }
    case 2: {
        if (conflict_index == 256) {
            for (int i = mini; i < counts.size(); i++)
                counts[i] *= 0.5;
            conflict_index = 0;
        }
        break;
    }
    }
    // assumes cube is ordered
    int sz = abs(uc->back());
    if (sz >= counts.size()) counts.resize(sz + 1);
    if (mini > abs(uc->at(0))) mini = abs(uc->at(0));
    for (auto l : *uc) {
        switch (branching_type) {
        case 1:
        case 2: {
            assert(abs(l) < counts.size());
            counts[abs(l)]++;
            break;
        }
        case 3:
            counts[abs(l)] = (counts[abs(l)] + conflict_index) / 2.0;
            break;
        }
    }
}


void Branching::Decay() {
    for (int i = mini; i < counts.size(); i++)
        counts[i] *= 0.99;
}


void Branching::Decay(const shared_ptr<cube> uc, int gap = 1) {
    if (uc->size() == 0) return;
    conflict_index++;
    // assumes cube is ordered
    int sz = abs(uc->back());
    if (sz >= counts.size()) counts.resize(sz + 1);
    if (mini > abs(uc->at(0))) mini = abs(uc->at(0));
    for (auto l : *uc) {
        counts[abs(l)] *= 1 - 0.01 * (gap - 1);
    }
}


static bool _cmp(int a, int b) {
    if (abs(a) != abs(b))
        return abs(a) < abs(b);
    else
        return a < b;
}


// ================================================================================
// @brief: if a implies b
// @input:
// @output:
// ================================================================================
bool OverSequenceSet::Imply(const cube &a, const cube &b) {
    if (a.size() > b.size())
        return false;
    return (includes(b.begin(), b.end(), a.begin(), a.end(), _cmp));
}


bool OverSequenceSet::Insert(const cube &uc, int index, bool implyCheck) {
    auto f = GetFrame(index);
    if (f->find(uc) != f->end()) return false;

    m_blockSolver->AddUC(uc, index);

    if (implyCheck) {
        for (auto it = f->begin(); it != f->end();) {
            if (Imply(uc, *it)) {
                it = f->erase(it);
            } else {
                ++it;
            }
        }
    }
    f->emplace(uc);

    return true;
}


shared_ptr<frame> OverSequenceSet::GetFrame(int lvl) {
    while (lvl >= m_sequence.size()) {
        m_sequence.emplace_back(make_shared<frame>());
        m_blockCounter.emplace_back(0);
    }
    return m_sequence[lvl];
}


bool OverSequenceSet::IsBlockedByFrame(const cube &latches, int frameLevel) {
    auto f = GetFrame(frameLevel);
    // by for checking
    for (const auto &uc : *f) {
        if (Imply(uc, latches)) {
            return true;
        }
    }
    return false;
}


bool OverSequenceSet::IsBlockedByFrameLazy(const cube &latches, int frameLevel) {
    if (frameLevel >= m_sequence.size()) return false;

    int &counter = m_blockCounter[frameLevel];
    if (counter == -1) { // by sat
        bool sat = m_blockSolver->SolveFrame(make_shared<cube>(latches), frameLevel);
        return !sat;
    }

    if (m_sequence[frameLevel]->size() > 3000) {
        counter++;
    }
    // whether it's need to change the way of checking
    if (counter > 1000) {
        auto start_time = std::chrono::high_resolution_clock::now();
        m_blockSolver->SolveFrame(make_shared<cube>(latches), frameLevel);
        auto sat_end = std::chrono::high_resolution_clock::now();
        bool res = IsBlockedByFrame(latches, frameLevel);
        auto for_end = std::chrono::high_resolution_clock::now();
        auto sat_time = std::chrono::duration_cast<std::chrono::microseconds>(sat_end - start_time).count();
        auto for_time = std::chrono::duration_cast<std::chrono::microseconds>(for_end - sat_end).count();
        if (sat_time > for_time)
            counter = 0;
        else
            counter = -1;
        return res;
    }

    return IsBlockedByFrame(latches, frameLevel);
}


void OverSequenceSet::GetBlockers(const cube &latches, int framelevel, vector<cube> &b) {
    auto f = GetFrame(framelevel);
    // by imply checking
    for (const auto &uc : *f) {
        if (Imply(uc, latches)) {
            b.emplace_back(uc);
        }
    }
}


string OverSequenceSet::FramesInfo() {
    string res;
    res += "Frames " + to_string(m_sequence.size() - 1) + "\n";
    for (int i = 0; i < m_sequence.size(); ++i) {
        res += to_string(m_sequence[i]->size()) + " ";
    }
    return res;
}


string OverSequenceSet::FramesDetail() {
    string res;
    for (int i = 0; i < m_sequence.size(); ++i) {
        res += "Frame " + to_string(i) + "\n";
        if (i != 0) {
            for (auto uc : *m_sequence[i]) {
                for (auto j : uc) {
                    res += to_string(j) + " ";
                }
                res += "\n";
            }
        }
        res += "size: " + to_string(m_sequence[i]->size()) + "\n";
    }
    return res;
}


int State::numInputs = -1;
int State::numLatches = -1;


string State::GetLatchesString() {
    string result = "";
    result.reserve(numLatches);
    int j = 0;
    for (int i = 0; i < numLatches; ++i) {
        if (j >= latches->size() || numInputs + i + 1 < abs(latches->at(j))) {
            result += "x";
        } else {
            result += (latches->at(j) > 0) ? "1" : "0";
            ++j;
        }
    }
    return result;
}


string State::GetInputsString() {
    string result = "";
    result.reserve(numInputs);
    int j = 0;
    for (int i = 1; i <= numInputs; ++i) {
        if (j >= inputs->size() || i < abs(inputs->at(j))) {
            result += "x";
        } else {
            result += (inputs->at(j) > 0) ? "1" : "0";
            ++j;
        }
    }
    return result;
}


} // namespace car