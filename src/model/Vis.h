#ifndef VIS_H
#define VIS_H

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <map>
#include <assert.h>
#include "State.h"
#include "Settings.h"
#include "UnderSequence.h"
namespace car
{
  typedef struct VisNode{
    int id;
    std::string name;
  }VisNode;

  typedef struct VisEdgeG{
    std::string type;
    std::string arrow;
  }VisEdgeG;

  typedef struct VisEdge{
    int source;
    int target;
    VisEdgeG edgeGraphics;
  }VisEdge;

  typedef struct VisGraph{
    int directed;
    std::vector<VisNode> nodes;
    std::vector<VisEdge> edges;
  }VisGraph;

  typedef struct VisGML{
    std::string Creator;
    std::string Version;
    VisGraph graph;
  }VisGML;

class Vis
{
public:
    Vis(Settings settings);

    ~Vis();

    void addState(std::shared_ptr<State> state);

    void OutputGML();

private:
  VisGML m_gml;
  int tempId;
  Settings m_settings;
  std::map<std::shared_ptr<State>, int> stateMap;

  std::string GetFileName(std::string filePath)
	{
		auto startIndex = filePath.find_last_of("/");
		if (startIndex == std::string::npos)
		{
			startIndex = 0;
		}
		else
		{
			startIndex++;
		}
		auto endIndex = filePath.find_last_of(".");
		assert (endIndex != std::string::npos);
		return filePath.substr(startIndex, endIndex-startIndex);	
	}
};

}//namespace car

#endif