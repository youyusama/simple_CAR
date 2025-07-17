# 编译器设置
CXX := g++ # 或 clang++
CXXFLAGS := -std=c++17 -Wall -Wextra
LDFLAGS := -lstdc++fs # 某些环境需要这个库

ifndef DEBUG
	CXXFLAGS += -O3 -DNDEBUG
else
	CXXFLAGS += -g -ggdb
endif

# 项目文件
TARGET := minicore
SRCS := src/main.cpp src/solver.cpp
HDRS := src/types.h src/solver_types.h src/utils.hpp
OBJS := $(SRCS:.cpp=.o)

# 目标定义
all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

# 高级编译规则
%.o: %.cpp $(HDRS)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# 清理规则
clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean