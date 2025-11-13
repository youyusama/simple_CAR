#ifndef MINICORE_UTILS_H
#define MINICORE_UTILS_H


#include <chrono>
#include <cstddef>
#include <signal.h>
#if defined(_WIN32) || defined(_WIN64)
#include <psapi.h>
#include <windows.h>
#elif defined(__linux__)
#include <cstdio>
#include <cstring>
#elif defined(__APPLE__)
#include <mach/mach.h>
#include <unistd.h>
#endif

namespace minicore {

inline double cpuTime() {
    static const auto start_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> diff = now - start_time;
    return diff.count();
}


inline double memUsedPeak() {
    double peakMemoryMB = 0.0;

#if defined(_WIN32) || defined(_WIN64)
    PROCESS_MEMORY_COUNTERS pmc;
    if (GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc))) {
        peakMemoryMB = static_cast<double>(pmc.PeakWorkingSetSize) / (1024.0 * 1024.0);
    }
#elif defined(__linux__)
    FILE *file = fopen("/proc/self/status", "r");
    if (file) {
        char line[128];
        while (fgets(line, sizeof(line), file)) {
            if (strncmp(line, "VmPeak:", 7) == 0) {
                long kb;
                if (sscanf(line + 7, "%ld", &kb) == 1) {
                    peakMemoryMB = static_cast<double>(kb) / 1024.0;
                }
                break;
            }
        }
        fclose(file);
    }
#elif defined(__APPLE__)
    // Use mach_task_basic_info which exposes the current resident size.
    // Some macOS SDKs don't provide `resident_size_max` in the reported
    // `task_basic_info` type; using `mach_task_basic_info` and
    // `MACH_TASK_BASIC_INFO` is more portable across macOS versions.
    struct mach_task_basic_info info;
    mach_msg_type_number_t info_count = MACH_TASK_BASIC_INFO_COUNT;

    if (task_info(mach_task_self(), MACH_TASK_BASIC_INFO,
                  (task_info_t)&info, &info_count) == KERN_SUCCESS) {
        peakMemoryMB = static_cast<double>(info.resident_size) / (1024.0 * 1024.0);
    }
#endif

    return peakMemoryMB;
}


inline void sigTerm(void handler(int)) {
    signal(SIGINT, handler);
    signal(SIGTERM, handler);
}

} // namespace minicore

#endif