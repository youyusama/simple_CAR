#ifndef RESTART_H
#define RESTART_H

#include "IncrCheckerHelpers.h"
#include <Settings.h>
#include <memory>
#include <stack>
#include <vector>

namespace car {


struct Luby {
  public:
    Luby() {
        m_index = 0;
    }

    int GetLuby(int index) {

        int distance = index + 1 - m_luby.size();
        if (distance > 0) {
            PushLuby(distance);
        }
        return m_luby[index];
    }

    int GetNextLuby() {
        int distance = m_index + 1 - m_luby.size();
        if (distance > 0) {
            PushLuby(distance);
        }
        return m_luby[m_index++];
    }

    void PushLuby(int numbers = 1) {
        int size = m_luby.size();
        int k;
        for (int i = 0; i < numbers; ++i) {
            k = GetLog2(size + 2);
            if (size + 2 == GetPower2(k)) {
                m_luby.push_back(GetPower2(k - 1));
            } else {
                m_luby.push_back(m_luby[size - GetPower2(k) + 1]);
            }
            size++;
        }
    }

  private:
    int GetLog2(int num) {
        if (num < 1) {
            return -1;
        }
        int power = 0;
        while (num > 1) {
            num = num / 2;
            power++;
        }
        return power;
    }

    int GetPower2(int power) {
        int res = 1;
        while (power > 0) {
            res = res * 2;
            power--;
        }
        return res;
    }


    std::vector<int> m_luby;
    int m_index;
};


class Restart {
  public:
    Restart(Settings settings) {
        if (settings.restartLuby) {
            isLubyActived = true;
            m_luby.PushLuby(15);
        }
        m_baseThreshold = settings.restartThreshold;
        m_threshold = settings.restartThreshold;
        m_growthRate = settings.restartGrowthRate;
    }

    bool RestartCheck() {
        GLOBAL_LOG->L(3, "Restart Check: ", m_ucCounts, " > ", m_threshold);
        return m_ucCounts > m_threshold;
    }

    void UpdateThreshold() {
        if (isLubyActived) {
            m_threshold = m_luby.GetNextLuby() * m_baseThreshold;
        } else {
            m_threshold = m_threshold * m_growthRate;
        }
        GLOBAL_LOG->L(2, "Updated Restart Threshold: ", m_threshold);
    }

    void UcCountsPlus1() { m_ucCounts++; }

    void ResetUcCounts() { m_ucCounts = 0; }

  private:
    bool isLubyActived = false;
    int m_threshold;
    int m_baseThreshold;
    int m_ucCounts = 0;
    float m_growthRate = 1.5;
    Luby m_luby;
};


} // namespace car


#endif