#include "Vis.h"
#include <stdio.h>

namespace car {
Vis::Vis(Settings settings, std::shared_ptr<AigerModel> model) : m_settings(settings), m_model(model) {
  node_id_map.clear();
  m_edges.clear();
  tempId = 0;
  uc_tree_root = new uc_tree_node();
}

Vis::~Vis() {
  node_id_map.clear();
  m_edges.clear();
  tempId = 0;
}

void latches_vecotr_to_short_vector(std::vector<uint32_t> &node, std::vector<int> &latches) {
  int count = 0;
  uint32_t tempi = 0;
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

void Vis::addState(std::shared_ptr<State> state) {
  uint temp_node_id = tempId++;
  std::vector<uint32_t> temp_node;
  latches_vecotr_to_short_vector(temp_node, *state->latches);
  // add node
  auto z = node_id_map.insert(std::pair<std::vector<uint32_t>, uint>(temp_node, temp_node_id));
  if (z.second == false) { // visit the same state repeatedly
    // std::cout<<"vis repeat"<<std::endl;
    tempId--;
    // std::cout<<z.first->second<<std::endl;
    // for (int i=0; i<z.first->first.size(); i++) std::cout<<z.first->first[i];
    // std::cout<<std::endl;
    // for (int i=0; i<temp_node.size(); i++) std::cout<<temp_node[i];
    // std::cout<<std::endl;
    std::map<std::vector<uint32_t>, uint>::iterator iter;
    iter = node_id_map.find(temp_node);
    if (iter != node_id_map.end()) {
      temp_node_id = iter->second;
    } else {
      // place holder
    }
  }

  // add edge
  if (state->preState) {
    // find prenode
    std::vector<uint32_t> pre_node;
    latches_vecotr_to_short_vector(pre_node, *state->preState->latches);
    std::map<std::vector<uint32_t>, uint>::iterator iter;
    iter = node_id_map.find(pre_node);
    if (iter != node_id_map.end()) {
      m_edges.push_back(std::pair<uint, uint>(iter->second, temp_node_id));
    } else {
      // place holder
    }
  }
}


std::string to_hex(int i) {
  std::stringstream ioss;
  std::string s;
  ioss << std::hex << i;
  ioss >> s;
  if (s.size() == 1) {
    s = "0" + s;
  } else if (s.size() > 2) {
    // place holder
  }
  return s;
}

std::string Vis_GetColor(int step, int n) {
  if (n == step - 1) {
    return "\"#DC143C\"";
  }
  // set gradual color
  // terminal 0,0,0 -> 15,155,15
  float r_a = 0, g_a = 0, b_a = 0;
  float r_b = 15, g_b = 155, b_b = 15;
  int r_n, g_n, b_n;
  r_n = (int)r_a + (r_b - r_a) / step * n;
  g_n = (int)g_a + (g_b - g_a) / step * n;
  b_n = (int)b_a + (b_b - b_a) / step * n;
  std::string res = "";
  res += to_hex(r_n);
  res += to_hex(g_n);
  res += to_hex(b_n);
  return "\"#" + res + "\"";
}


void write_node(std::ofstream &visFile, uint id, std::string text = "", std::string color = "\"#008000\"") {
  text = (text.length() == 0) ? std::to_string(id) : text;
  visFile << "node" << std::endl;
  visFile << "[" << std::endl;
  visFile << "id\t" << id << std::endl;
  visFile << "graphics" << std::endl;
  visFile << "[" << std::endl;
  visFile << "w\t" << 50 << std::endl;
  visFile << "h\t" << 160 << std::endl;
  visFile << "fill\t" << color << std::endl;
  visFile << "]" << std::endl;
  visFile << "LabelGraphics" << std::endl;
  visFile << "[" << std::endl;
  visFile << "text\t"
          << "\"" << text << "\"" << std::endl;
  visFile << "color\t"
          << "\"#333333\"" << std::endl;
  visFile << "]" << std::endl;
  visFile << "]" << std::endl;
}

void write_edge(std::ofstream &visFile, uint source, uint target) {
  visFile << "edge" << std::endl;
  visFile << "[" << std::endl;
  visFile << "source\t" << source << std::endl;
  visFile << "target\t" << target << std::endl;
  visFile << "graphics" << std::endl;
  visFile << "[" << std::endl;
  visFile << "type\t"
          << "\"arc\"" << std::endl;
  visFile << "arrow\t"
          << "\"last\"" << std::endl;
  visFile << "]" << std::endl;
  visFile << "]" << std::endl;
}


void Vis::OutputGML(bool is_timeout) {
  // add the cex info
  std::vector<std::vector<uint32_t>> cex_states_v;
  bool is_cex_valid = true;
  if (m_settings.cexFilePath.length() != 0) {
    // check cex file type
    std::string cex_file_type = m_settings.cexFilePath.substr(m_settings.cexFilePath.rfind("."));
    if (cex_file_type == ".cex") {
      // generate .res from .cex
      std::ifstream r_cex_file(m_settings.cexFilePath);
      std::ofstream w_res_file(m_settings.cexFilePath.substr(0, m_settings.cexFilePath.length() - 4) + ".res");
      w_res_file << 1 << std::endl
                 << "b0" << std::endl;
      std::string t;
      while (!r_cex_file.eof()) {
        r_cex_file >> t;
        if (t[t.length() - 1] == '#') {
          w_res_file << t.substr(0, t.length() - 1) << std::endl;
          break;
        } else {
          w_res_file << t << std::endl;
        }
      }
      w_res_file << '.' << std::endl;
      r_cex_file.close();
      w_res_file.close();
      m_settings.cexFilePath = m_settings.cexFilePath.substr(0, m_settings.cexFilePath.length() - 4) + ".res";
    } else if (cex_file_type == ".res") {
      // nothing to do
    } else {
      std::cout << "illegal cex file" << std::endl;
      is_cex_valid = false;
    }

    // read the res
    std::ifstream r_cex_file(m_settings.cexFilePath);
    int step_num = 0;
    if (r_cex_file.is_open()) {
      while (!r_cex_file.eof()) {
        std::string t;
        r_cex_file >> t;
        if (t != "1" && t != "b0" && t != "." && t.length() != 0) {
          step_num++;

          if (t.length() != m_model->GetNumInputs() && t.length() != m_model->GetNumLatches()) {
            std::cout << "illegal cex file [wrong inputs]" << std::endl;
            is_cex_valid = false;
          }
        }
      }
    }
    step_num--; // not count the initial line

    // call aigsim to fill cex_ss
    uint32_t **cex_states = new uint32_t *[step_num];
    int arrlen = m_model->GetNumLatches() % 32 == 0 ? m_model->GetNumLatches() / 32 : m_model->GetNumLatches() / 32 + 1;
    for (int cex_i = 0; cex_i < step_num; cex_i++) {
      cex_states[cex_i] = new uint32_t[arrlen];
    }
    if (is_cex_valid) aigsim_for_vis(cex_states, m_settings.aigFilePath.c_str(), m_settings.cexFilePath.c_str(), step_num);
    for (int i = 0; i < step_num; i++) {
      // for (int j=0; j<arrlen; j++) std::cout<<cex_states[i][j];
      // std::cout<<std::endl;
      std::vector<uint32_t> temp_i(cex_states[i], cex_states[i] + arrlen);
      cex_states_v.push_back(temp_i);
    }
  }


  // io
  std::string outPath = m_settings.outputDir + GetFileName(m_settings.aigFilePath);
  std::ofstream visFile;
  visFile.open(outPath + "_vis.gml");
  // header
  if (is_timeout)
    visFile << "Creator\t"
            << "\"car visualization (time out)\"" << std::endl;
  else
    visFile << "Creator\t"
            << "\"car visualization\"" << std::endl;
  visFile << "Version\t" << 0.1 << std::endl;
  visFile << "graph" << std::endl;
  visFile << "[" << std::endl;
  // graph info
  visFile << "directed\t" << 1 << std::endl;
  // search nodes
  std::map<std::vector<uint32_t>, uint>::iterator iter;
  for (iter = node_id_map.begin(); iter != node_id_map.end(); iter++) {
    bool is_cex_node = false;
    if (is_cex_valid) {
      for (std::vector<uint32_t> st : cex_states_v) {
        if (iter->first == st) is_cex_node = true;
      }
    }
    if (is_cex_node) {
      // skip
    } else {
      write_node(visFile, iter->second, Vis_GetColor(tempId, iter->second));
    }
  }
  // cex nodes and edges
  bool is_first_cex_step = true;
  uint last_cex_step_id;
  for (std::vector<uint32_t> st : cex_states_v) {
    std::map<std::vector<uint32_t>, uint>::iterator iter;
    iter = node_id_map.find(st);
    if (iter != node_id_map.end()) {
      write_node(visFile, iter->second, "\"#0000CD\"");
      if (is_first_cex_step) {
        is_first_cex_step = false;
      } else {
        m_edges.push_back(std::pair<uint, uint>(last_cex_step_id, iter->second));
      }
      last_cex_step_id = iter->second;
    } else {
      int node_id = tempId++;
      write_node(visFile, node_id, "\"#0000CD\"");
      if (is_first_cex_step) {
        is_first_cex_step = false;
      } else {
        m_edges.push_back(std::pair<uint, uint>(last_cex_step_id, node_id));
      }
      last_cex_step_id = node_id;
    }
  }
  // edges
  for (auto e : m_edges) {
    write_edge(visFile, e.first, e.second);
  }
  visFile << "]" << std::endl;
  visFile.close();
}


bool Vis::isEnoughNodesForVis() {
  if (node_id_map.size() >= 8000)
    return true;
  else
    return false;
}


bool _LitComp(int &l1, int &l2) {
  return l1 < l2;
}

// ================================================================================
// @brief: whether cube1 -> cube2
// @input:
// @output:
// ================================================================================
bool _cube_imply(std::vector<int> cube1, std::vector<int> cube2) {
  // if (cube1.size() == 0) return false;
  if (std::includes(cube2.begin(), cube2.end(), cube1.begin(), cube1.end(), _LitComp))
    return true;
  else
    return false;
}

// ================================================================================
// @brief: whether cube1 == cube2
// @input:
// @output:
// ================================================================================
bool _cube_same(std::vector<int> cube1, std::vector<int> cube2) {
  if (cube1.size() != cube2.size()) return false;

  for (int i = 0; i < cube1.size(); i++) {
    if (cube1[i] != cube2[i]) return false;
  }
  return true;
}

// ================================================================================
// @brief: clear the tree
// @input:
// @output:
// ================================================================================
void Vis::clear_tree() {
  std::queue<uc_tree_node *> node_queue;
  node_queue.push(uc_tree_root);
  while (!node_queue.empty()) {
    uc_tree_node *f_node = node_queue.front();
    node_queue.pop();
    for (auto c : f_node->children) {
      node_queue.push(c);
    }
    delete f_node;
  }
  uc_tree_root = new uc_tree_node();
}

// ================================================================================
// @brief: add node to the frame tree
// @input: the cube, frame level
// @output:
// ================================================================================
void Vis::add_tree_node(std::vector<int> &cube, int level) {
  // new node
  uc_tree_node *temp_node = new uc_tree_node();
  temp_node->cube = cube;
  if (level == 1) {
    bool has_same = false;
    for (uc_tree_node *bro : uc_tree_root->children) {
      if (_cube_same(bro->cube, temp_node->cube)) {
        has_same = true;
        break;
      }
    }
    if (!has_same) uc_tree_root->children.emplace_back(temp_node);
  } else {
    int depth = 0, node_num = 1, nl_node_num = 0;
    std::queue<uc_tree_node *> node_queue;
    node_queue.push(uc_tree_root);
    while (!node_queue.empty() && depth < level - 1) {
      uc_tree_node *f_node = node_queue.front();
      node_queue.pop();
      for (auto node : f_node->children) {
        if (_cube_imply(node->cube, temp_node->cube)) {
          node_queue.push(node);
          nl_node_num++;
        }
      }
      node_num--;
      if (node_num == 0) {
        depth++;
        node_num = nl_node_num;
        nl_node_num = 0;
      }
    }
    while (!node_queue.empty()) {
      uc_tree_node *f_node = node_queue.front();
      node_queue.pop();
      if (!f_node->is_blank) {
        bool has_same = false;
        for (auto bro : f_node->children) {
          if (_cube_same(bro->cube, temp_node->cube)) {
            has_same = true;
            break;
          }
        }
        if (!has_same) {
          f_node->children.emplace_back(temp_node);
        }
        return;
      } else {
        if (_cube_same(f_node->children[0]->cube, temp_node->cube)) return;
      }
    }
    depth = 1;
    uc_tree_node *temp_anc = uc_tree_root;
    while (depth < level) {
      uc_tree_node *blank_node = new uc_tree_node();
      blank_node->is_blank = true;
      temp_anc->children.emplace_back(blank_node);
      temp_anc = blank_node;
      depth++;
    }
    temp_anc->children.emplace_back(temp_node);
  }
  return;
}

std::string _cube_to_string(std::vector<int> cube) {
  std::string res = "";
  for (auto l : cube) {
    res += std::to_string((l > 0) ? (l * 2) : (-l * 2 + 1)) + "\n";
  }
  return res;
}

void Vis::print_tree(int k) {
  // io
  std::ofstream visFile;
  visFile.open("frame_vis/k" + std::to_string(k) + ".gml");
  // header
  visFile << "Creator\t"
          << "\"car visualization\"" << std::endl;
  visFile << "Version\t" << 0.1 << std::endl;
  visFile << "graph" << std::endl;
  visFile << "[" << std::endl;
  // graph info
  visFile << "directed\t" << 1 << std::endl;
  // traverse tree
  uint tree_id = 0;
  std::queue<uint> id_queue;
  std::queue<uc_tree_node *> node_queue;
  id_queue.push(tree_id);
  node_queue.push(uc_tree_root);
  write_node(visFile, tree_id, "root");
  tree_id++;

  while (!node_queue.empty()) {
    uc_tree_node *temp_node = node_queue.front();
    node_queue.pop();
    uint temp_id = id_queue.front();
    id_queue.pop();
    for (auto node : temp_node->children) {
      id_queue.push(tree_id);
      node_queue.push(node);
      if (node->is_blank) {
        write_node(visFile, tree_id, " ", "\"#ffffff\"");
      } else {
        write_node(visFile, tree_id, _cube_to_string(node->cube));
      }
      write_edge(visFile, temp_id, tree_id);
      tree_id++;
    }
  }
  visFile << "]" << std::endl;
  visFile.close();
}


} // namespace car