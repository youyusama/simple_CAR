#include "OverSequenceSet.h"

namespace car {

static bool _cmp(int a, int b) {
    if (abs(a) != abs(b))
        return abs(a) < abs(b);
    else
        return a < b;
}


void OverSequenceSet::add_uc_to_frame(const cube *uc, frame &f) {
    frame tmp;
    for (auto f_uc : f) {
        if (!is_imply(*uc, *f_uc))
            tmp.emplace(f_uc);
    }
    tmp.emplace(uc);
    f.swap(tmp);
}

// ================================================================================
// @brief: if a->b
// @input:
// @output:
// ================================================================================
bool OverSequenceSet::is_imply(cube a, cube b) {
    if (a.size() >= b.size())
        return false;
    if (includes(b.begin(), b.end(), a.begin(), a.end(), _cmp))
        return true;
    else
        return false;
}

bool OverSequenceSet::Insert(shared_ptr<cube> uc, int index) {
    m_blockSolver->AddUnsatisfiableCore(*uc, index);
    if (index >= m_sequence.size()) {
        m_sequence.emplace_back(frame());
        m_block_counter.emplace_back(0);
    }
    auto res = Ucs.insert(*uc);
    add_uc_to_frame(&*res.first, m_sequence[index]);
    return true;
}


// ================================================================================
// @brief: init frame 0
// @input: init state
// @output:
// ================================================================================
void OverSequenceSet::Init_Frame_0(shared_ptr<cube> latches) {
    m_sequence.emplace_back(frame());
    m_block_counter.emplace_back(0);
    for (auto l : *latches) {
        shared_ptr<cube> puc(new vector<int>{-l});
        auto res = Ucs.insert(*puc);
        m_sequence[0].emplace(&*res.first);
        m_blockSolver->AddUnsatisfiableCore(*puc, 0);
    }
}


void OverSequenceSet::GetFrame(int frameLevel, vector<shared_ptr<vector<int>>> &out) {
    if (frameLevel >= m_sequence.size()) return;
    frame frame_i = m_sequence[frameLevel];
    vector<shared_ptr<vector<int>>> res;
    res.reserve(frame_i.size());
    for (auto i : frame_i) {
        shared_ptr<vector<int>> temp(new vector<int>());
        temp->resize(i->size());
        copy(i->begin(), i->end(), temp->begin());
        res.emplace_back(temp);
    }
    out = res;
    return;
}

bool OverSequenceSet::IsBlockedByFrame(vector<int> &latches, int frameLevel) {
    // by for checking
    int latch_index, num_inputs;
    num_inputs = m_model->GetNumInputs();
    for (auto uc : m_sequence[frameLevel]) { // for each uc
        bool blocked = true;
        for (int j = 0; j < uc->size(); j++) { // for each literal
            latch_index = abs(uc->at(j)) - num_inputs - 1;
            if (latches[latch_index] != uc->at(j)) {
                blocked = false;
                break;
            }
        }
        if (blocked) {
            return true;
        }
    }
    return false;
}


bool OverSequenceSet::IsBlockedByFrame_sat(vector<int> &latches, int frameLevel) {
    vector<int> assumption;
    assumption.reserve(latches.size());
    for (int l : latches) {
        assumption.emplace_back(l);
    }

    bool result = m_blockSolver->SolveWithAssumption(assumption, frameLevel);
    if (!result) {
        return true;
    } else {
        return false;
    }
}


bool OverSequenceSet::IsBlockedByFrame_lazy(vector<int> &latches, int frameLevel) {
    int &counter = m_block_counter[frameLevel];
    if (counter == -1) { // by sat
        vector<int> assumption;
        assumption.reserve(latches.size());
        for (int l : latches) {
            assumption.emplace_back(l);
        }

        bool result = m_blockSolver->SolveWithAssumption(assumption, frameLevel);
        if (!result) {
            return true;
        } else {
            return false;
        }
    }
    if (m_sequence[frameLevel].size() > 3000) {
        counter++;
    }
    // whether it's need to change the way of checking
    clock_t start_time, sat_time, for_time;
    if (counter > 1000) {
        start_time = clock();
        vector<int> assumption;
        assumption.reserve(latches.size());
        for (int l : latches) {
            assumption.emplace_back(l);
        }
        m_blockSolver->SolveWithAssumption(assumption, frameLevel);
        sat_time = clock();
    }
    // by imply checking
    for (auto uc : m_sequence[frameLevel]) { // for each uc
        if (includes(latches.begin(), latches.end(), uc->begin(), uc->end(), _cmp)) return true;
    }
    if (counter > 1000) {
        for_time = clock();
        if (sat_time - start_time > for_time - sat_time)
            counter = 0;
        else
            counter = -1;
    }
    return false;
}

int OverSequenceSet::GetLength() {
    return m_sequence.size();
}

void OverSequenceSet::set_solver(shared_ptr<CarSolver> slv) {
    m_mainSolver = slv;
}

vector<int> *OverSequenceSet::GetBlocker(shared_ptr<vector<int>> latches, int framelevel) {
    if (framelevel >= m_sequence.size()) return new vector<int>();
    // by imply checking
    for (auto uc : m_sequence[framelevel]) { // for each uc
        if (latches->size() < uc->size()) break;
        if (includes(latches->begin(), latches->end(), uc->begin(), uc->end(), _cmp)) return uc;
    }
    return new vector<int>();
}

vector<cube *> *OverSequenceSet::GetBlockers(shared_ptr<vector<int>> latches, int framelevel) {
    vector<cube *> *res = new vector<cube *>();
    if (framelevel >= m_sequence.size()) return res;
    int size = -1;
    // by imply checking
    for (auto uc : m_sequence[framelevel]) { // for each uc
        if (size != -1 && size < uc->size()) break;
        if (includes(latches->begin(), latches->end(), uc->begin(), uc->end(), _cmp)) {
            size = uc->size();
            res->emplace_back(uc);
        }
    }
    return res;
}

void OverSequenceSet::propagate(int level, shared_ptr<Branching> b) {
    // cout << "propagate " << level << endl;
    frame &fi = m_sequence[level];
    frame &fi_plus_1 = m_sequence[level + 1];
    set<cube *>::iterator iter;
    for (cube *uc : fi) {
        iter = fi_plus_1.find(uc);
        if (iter != fi_plus_1.end()) continue; // propagated

        vector<int> ass;
        ass.reserve(uc->size());
        if (isForward)
            for (auto i : *uc) {
                ass.emplace_back(m_model->GetPrime(i));
            }
        else
            for (auto i : *uc) {
                ass.emplace_back(i);
            }
        if (!m_mainSolver->SolveWithAssumption(ass, level)) {
            add_uc_to_frame(uc, fi_plus_1);
            b->update(uc);
            m_blockSolver->AddUnsatisfiableCore(*uc, level + 1);
            m_mainSolver->AddUnsatisfiableCore(*uc, level + 1);
        }
    }
    return;
}


// ================================================================================
// @brief: propagate the uc from lvl +1
// @input: uc already in lvl
// @output: uc cannot propagate to lvl
// ================================================================================
int OverSequenceSet::propagate_uc_from_lvl(shared_ptr<cube> uc, int lvl, shared_ptr<Branching> b) {
    while (lvl + 1 < m_sequence.size()) {
        frame &fi = m_sequence[lvl];
        frame &fi_plus_1 = m_sequence[lvl + 1];
        vector<int> ass;
        ass.reserve(uc->size());
        if (isForward)
            for (auto i : *uc) {
                ass.emplace_back(m_model->GetPrime(i));
            }
        else
            for (auto i : *uc) {
                ass.emplace_back(i);
            }
        if (!m_mainSolver->SolveWithAssumption(ass, lvl)) {
            auto res = Ucs.insert(*uc);
            add_uc_to_frame(&*res.first, fi_plus_1);
            b->update(&*res.first);
            m_blockSolver->AddUnsatisfiableCore(*uc, lvl + 1);
            m_mainSolver->AddUnsatisfiableCore(*uc, lvl + 1);
        } else {
            break;
        }
        lvl++;
    }
    return lvl + 1;
}

void OverSequenceSet::PrintFramesInfo() {
    m_log->PrintSth("Frames " + to_string(m_sequence.size() - 1) + "\n");
    for (int i = 0; i < m_sequence.size(); ++i) {
        m_log->PrintSth(to_string(m_sequence[i].size()) + " ");
    }
    m_log->PrintSth("\n");
}

void OverSequenceSet::PrintOSequence() {
    if (!m_log->IsDebug()) return;
    for (int i = 0; i < m_sequence.size(); ++i) {
        m_log->DebugPrintSth(to_string(m_sequence[i].size()) + " ");
    }
    m_log->DebugPrintSth("\n");
}


void OverSequenceSet::PrintOSequenceDetail() {
    if (!m_log->IsDebug()) return;
    for (int i = 0; i < m_sequence.size(); ++i) {
        m_log->DebugPrintSth("Frame " + to_string(i) + "\n");
        if (i != 0) {
            for (auto uc : m_sequence[i]) {
                for (auto j : *uc) {
                    m_log->DebugPrintSth(to_string(j) + " ");
                }
                m_log->DebugPrintSth("\n");
            }
        }
        m_log->DebugPrintSth("size: " + to_string(m_sequence[i].size()) + "\n");
    }
    m_log->DebugPrintSth("\n");
}

} // namespace car