#include "Log.h"

namespace car {
void Log::PrintSth(string s) {
    m_log << s;
}

void Log::DebugPrintSth(string s) {
    if (m_debug) m_debug << s;
}

bool Log::IsDebug() {
    if (m_settings.debug)
        return true;
    else
        return false;
}


void Log::PrintLitOrder(vector<float> order) {
    if (!m_debug) return;
    for (int i = 0; i < order.size(); i++) {
        m_debug << i << ": " << (order[i] ? to_string(order[i]) : "-") << "\t";
    }
    m_debug << endl;
}

// void Log::PrintFramesInfo(IOverSequence* sequence)
// {
//   m_log<<"Frame "<<sequence->GetLength()-1<<endl;
//   for (int i = 0; i < sequence->GetLength(); ++i)
//   {
//     vector<shared_ptr<vector<int> > > frame;
//     sequence->GetFrame(i, frame);
//     m_log<<"frame "<<i<<" size "<<frame.size()<<endl;
//     for (auto j: frame){
//       for (auto k: *j){
//         m_log<<k<<" ";
//       }
//       m_log<<endl;
//     }
//   }
//   m_log<<endl;
// }

void Log::PrintCounterExample(int badNo, bool isForward) {

    m_res << "1" << endl
          << "b" << badNo << endl;
    if (isForward) {
        if (lastState == nullptr) {
            for (int i = 0; i < m_model->GetNumLatches(); ++i) {
                m_res << "0";
            }
            m_res << endl;
            for (int i = 0; i < m_model->GetNumInputs(); ++i) {
                m_res << "0";
            }
            m_res << endl;
        } else {
            shared_ptr<State> state = lastState;
            m_res << state->GetValueOfLatches() << endl;
            m_res << state->GetValueOfInputs() << endl;
            while (state->preState != nullptr) {
                state = state->preState;
                m_res << state->GetValueOfInputs() << endl;
            }
        }
    } else {
        if (lastState == nullptr) {
            for (int i = 0; i < m_model->GetNumLatches(); ++i) {
                m_res << "0";
            }
            m_res << endl;
            for (int i = 0; i < m_model->GetNumInputs(); ++i) {
                m_res << "0";
            }
            m_res << endl;
        } else {
            stack<shared_ptr<State>> trace;
            shared_ptr<State> state = lastState;
            while (state != nullptr) {
                trace.push(state);
                state = state->preState;
            }
            m_res << trace.top()->GetValueOfLatches() << endl;
            // m_res << trace.top()->GetValueOfInputs()<<endl;
            trace.pop();
            while (!trace.empty()) {
                m_res << trace.top()->GetValueOfInputs() << endl;
                trace.pop();
            }
        }
    }
    m_res << "." << endl;
}

void Log::PrintSafe(int badNo) {
    m_res << "0" << endl
          << "b" << badNo << endl
          << "." << endl;
}


void Log::PrintInDebug(string str) {
    if (m_debug) m_debug << str;
}

void Log::DebugPrintVector(vector<int> &v, string text) {
    if (!m_debug) return;
    m_debug << text << endl;
    for (auto l : v)
        m_debug << l << " ";
    m_debug << endl;
}

void latches_vecotr_to_short_vector(vector<int> &node, vector<int> &latches) {
    int count = 0;
    int tempi = 0;
    for (int l : latches) {
        if (l > 0)
            tempi = (tempi << 1) + 1;
        else
            tempi <<= 1;
        count++;
        if (count == 32 || l == latches[latches.size() - 1]) {
            node.emplace_back(tempi);
            tempi = 0;
            count = 0;
        }
    }
}

void Log::PrintStateShort(shared_ptr<State> s) {
    if (!m_debug) return;
    vector<int> node;
    latches_vecotr_to_short_vector(node, *s->latches);
    for (int n : node) {
        m_debug << n;
    }
    m_debug << endl;
}


} // namespace car