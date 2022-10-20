#ifndef VIS_H
#define VIS_H

#include "../model/AigerModel.h"
#include "../model/Settings.h"
#include "../model/State.h"
#include "../model/UnderSequence.h"
#include <assert.h>
#include <fstream>
#include <iostream>
#include <map>
#include <queue>
#include <sstream>
#include <string>
#include <vector>
extern "C" {
#include "aigsim_for_vis.h"
}

namespace car {

class Vis {
public:
  Vis(Settings settings, std::shared_ptr<AigerModel> model);

  ~Vis();

  void addState(std::shared_ptr<State> state);

  /* a gml file sample
Creator "vis"
Version 0.1
graph
[ directed  1
  node
  [ id  0
    graphics
    [ fill "#RRGGBB"
    ]
    LabelGraphics
    [ text  "0"
      color "#RRGGBB"
    ]
  ]
  edge
  [ source  1
    target  0
    graphics
    [ type "arc"
      arrow "last"
    ]
  ]
]
  */
  void OutputGML(bool is_timeout);
  bool isEnoughNodesForVis();

  void add_tree_node(std::vector<int> &cube, int level);

  void print_tree(int k);

  void clear_tree();

private:
  // search vis
  uint tempId;
  std::shared_ptr<AigerModel> m_model;
  Settings m_settings;
  std::map<std::vector<uint32_t>, uint> node_id_map;
  std::vector<std::pair<uint, uint>> m_edges;

  // proof vis
  struct uc_tree_node {
    std::vector<int> cube;
    std::vector<uc_tree_node *> children;
    bool is_blank;
    uc_tree_node() {
      cube.clear();
      children.clear();
      is_blank = false;
    }
  };
  uc_tree_node *uc_tree_root;

  std::string GetFileName(std::string filePath) {
    auto startIndex = filePath.find_last_of("/");
    if (startIndex == std::string::npos) {
      startIndex = 0;
    } else {
      startIndex++;
    }
    auto endIndex = filePath.find_last_of(".");
    assert(endIndex != std::string::npos);
    return filePath.substr(startIndex, endIndex - startIndex);
  }
};

} // namespace car

#endif