#include "Log.h"

namespace car {
void Log::PrintSth(std::string s) {
  m_log << s << std::endl;
}

void Log::DebugPrintSth(std::string s) {
  if (m_debug) m_debug << s << std::endl;
}


void Log::PrintFramesInfo(OverSequenceNI *sequence) {
  m_log << "Frame " << sequence->GetLength() - 1 << std::endl;
  for (int i = 0; i < sequence->GetLength(); ++i) {
    std::vector<std::shared_ptr<std::vector<int>>> frame;
    sequence->GetFrame(i, frame);
    m_log << frame.size() << " ";
  }
  m_log << std::endl;
}


void Log::PrintFramesInfo(OverSequenceSet *sequence) {
  m_log << "Frame " << sequence->GetLength() - 1 << std::endl;
  for (int i = 0; i < sequence->GetLength(); ++i) {
    std::vector<std::shared_ptr<std::vector<int>>> frame;
    sequence->GetFrame(i, frame);
    m_log << frame.size() << " ";
  }
  m_log << std::endl;
}


void Log::PrintFramesInfo(IOverSequence *sequence) {
  m_log << "Frame " << sequence->GetLength() - 1 << std::endl;
  for (int i = 0; i < sequence->GetLength(); ++i) {
    std::vector<std::shared_ptr<std::vector<int>>> frame;
    sequence->GetFrame(i, frame);
    m_log << frame.size() << " ";
  }
  m_log << std::endl;
}

// void Log::PrintFramesInfo(IOverSequence* sequence)
// {
//   m_log<<"Frame "<<sequence->GetLength()-1<<std::endl;
//   for (int i = 0; i < sequence->GetLength(); ++i)
//   {
//     std::vector<std::shared_ptr<std::vector<int> > > frame;
//     sequence->GetFrame(i, frame);
//     m_log<<"frame "<<i<<" size "<<frame.size()<<std::endl;
//     for (auto j: frame){
//       for (auto k: *j){
//         m_log<<k<<" ";
//       }
//       m_log<<std::endl;
//     }
//   }
//   m_log<<std::endl;
// }

void Log::PrintCounterExample(int badNo, bool isForward) {

  m_res << "1" << std::endl
        << "b" << badNo << std::endl;
  if (isForward) {
    std::shared_ptr<State> state = lastState;
    m_res << state->GetValueOfLatches() << std::endl;
    m_res << state->GetValueOfInputs() << std::endl;
    while (state->preState != nullptr) {
      state = state->preState;
      m_res << state->GetValueOfInputs() << std::endl;
    }
  } else {
    if (lastState == nullptr) {
      for (int i = 0; i < m_model->GetNumLatches(); ++i) {
        m_res << "0";
      }
      m_res << std::endl;
      for (int i = 0; i < m_model->GetNumInputs(); ++i) {
        m_res << "0";
      }
      m_res << std::endl;
    } else {
      std::stack<std::shared_ptr<State>> trace;
      std::shared_ptr<State> state = lastState;
      while (state != nullptr) {
        trace.push(state);
        state = state->preState;
      }
      m_res << trace.top()->GetValueOfLatches() << std::endl;
      // m_res << trace.top()->GetValueOfInputs()<<std::endl;
      trace.pop();
      while (!trace.empty()) {
        m_res << trace.top()->GetValueOfInputs() << std::endl;
        trace.pop();
      }
    }
  }
  m_res << "." << std::endl;
}

void Log::PrintSafe(int badNo) {
  m_res << "0" << std::endl
        << "b" << badNo << std::endl
        << "." << std::endl;
}


void Log::PrintInDebug(std::string str) {
  if (m_debug) m_debug << str;
}

void Log::DebugPrintVector(std::vector<int> &v, std::string text) {
  if (!m_debug) return;
  m_debug << text << std::endl;
  for (auto l : v)
    m_debug << l << " ";
  m_debug << std::endl;
}

void latches_vecotr_to_short_vector(std::vector<int> &node, std::vector<int> &latches) {
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

void Log::PrintStateShort(std::shared_ptr<State> s) {
  if (!m_debug) return;
  std::vector<int> node;
  latches_vecotr_to_short_vector(node, *s->latches);
  for (int n : node) {
    m_debug << n;
  }
  m_debug << std::endl;
}

void Log::PrintOSequence(OverSequenceNI *sequence) {
  if (!m_debug) return;
  for (int i = 0; i < sequence->GetLength(); ++i) {
    std::vector<std::shared_ptr<std::vector<int>>> frame;
    sequence->GetFrame(i, frame);
    m_debug << frame.size() << " ";
  }
  m_debug << std::endl;
}

void Log::PrintOSequence(OverSequenceSet *sequence) {
  if (!m_debug) return;
  for (int i = 0; i < sequence->GetLength(); ++i) {
    std::vector<std::shared_ptr<std::vector<int>>> frame;
    sequence->GetFrame(i, frame);
    m_debug << frame.size() << " ";
  }
  m_debug << std::endl;
}

void Log::PrintOSequence(IOverSequence *sequence) {
  if (!m_debug) return;
  for (int i = 0; i < sequence->GetLength(); ++i) {
    std::vector<std::shared_ptr<std::vector<int>>> frame;
    sequence->GetFrame(i, frame);
    m_debug << frame.size() << " ";
  }
  m_debug << std::endl;
}

void Log::PrintUcNums(std::vector<int> &uc, OverSequenceNI *sequence) {
  if (!m_debug) return;
  m_debug << "SAT????????????, UNSAT" << std::endl
          << "???uc=";
  for (int i = 0; i < uc.size(); ++i) {
    m_debug << uc[i] << " ";
  }
  m_debug << std::endl
          << "Frame:\t";
  for (int i = 0; i < sequence->GetLength(); ++i) {
    std::vector<std::shared_ptr<std::vector<int>>> frame;
    sequence->GetFrame(i, frame);
    m_debug << frame.size() << " ";
  }
  m_debug << std::endl;
}

void Log::PrintUcNums(std::vector<int> &uc, IOverSequence *sequence) {
  if (!m_debug) return;
  m_debug << "SAT????????????, UNSAT" << std::endl
          << "???uc=";
  for (int i = 0; i < uc.size(); ++i) {
    m_debug << uc[i] << " ";
  }
  m_debug << std::endl
          << "Frame:\t";
  for (int i = 0; i < sequence->GetLength(); ++i) {
    std::vector<std::shared_ptr<std::vector<int>>> frame;
    sequence->GetFrame(i, frame);
    m_debug << frame.size() << " ";
  }
  m_debug << std::endl;
}

void Log::PrintSAT(std::vector<int> &vec, int frameLevel) {
  if (!m_debug) return;
  m_debug << "----------------------" << std::endl;
  m_debug << "??????SAT, frameLevel= " << frameLevel << std::endl
          << "assumption = ";
  for (int i = 0; i < vec.size(); ++i) {
    m_debug << vec[i] << " ";
  }
  m_debug << std::endl;
}


void Log::PrintPineInfo(std::shared_ptr<State> state, std::shared_ptr<std::vector<int>> uc) {
  if (!m_debug) return;
  if (state->pine_state_type != 1) {
    m_debug << "pine ?????????" << std::endl;
    return;
  }
  m_debug << "pine assumptions ??????:" << std::endl;
  m_debug << "??? " << state->pine_l_index->size() << " ???: ";
  for (auto i : *state->pine_l_index) m_debug << i << " ";
  m_debug << std::endl;
  int n = 0;
  for (auto u : *uc) {
    auto iter = std::find(state->pine_assumptions->begin(), state->pine_assumptions->end(), u);
    if (iter - state->pine_assumptions->begin() + 1 > n)
      n = iter - state->pine_assumptions->begin() + 1;
  }
  m_debug << n << std::endl;
  int r = 0;
  for (auto i : *state->pine_l_index) {
    if (n >= i)
      r++;
    else
      break;
  }
  m_debug << "uc ????????? " << r + 1 << " ???" << std::endl;
  if (state->pine_l_list_type == 0) {
    m_debug << "list ????????????" << std::endl;
  } else {
    m_debug << "list ???????????????" << std::endl;
  }
}


void Log::StatPineInfo(std::shared_ptr<State> state, std::shared_ptr<std::vector<int>> uc_pine, std::shared_ptr<std::vector<int>> uc) {
  m_pineCalled++;
  if (uc_pine->size() <= uc->size()) m_pineIsShort++;
  int n = 0;
  for (auto u : *uc) {
    auto iter = std::find(state->pine_assumptions->begin(), state->pine_assumptions->end(), u);
    if (iter - state->pine_assumptions->begin() + 1 > n)
      n = iter - state->pine_assumptions->begin() + 1;
  }
  int len = 0;
  for (auto i : *state->pine_l_index) {
    if (n < i) {
      len = i;
      break;
    }
  }
  if (len / (float)state->latches->size() < 0.2) m_pineL1isShort++;
}


} // namespace car