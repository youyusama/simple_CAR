#include "Vis.h"

namespace car{
  Vis::Vis(Settings settings):m_settings(settings){
    m_gml.Creator = "\"CAR visualization\"";
    m_gml.Version = "0.1";
    VisGraph newgraph;
    m_gml.graph = newgraph;
    m_gml.graph.directed = 1;
    tempId = 0;
  }

  Vis::~Vis(){
    stateMap.clear();
  }

  void Vis::addState(std::shared_ptr<State> state){
    VisNode newNode;
    newNode.id = tempId;
    newNode.name = std::to_string(newNode.id);
    tempId++;
    stateMap.insert(std::pair<std::shared_ptr<State>, int>(state, newNode.id));
    //add node
    m_gml.graph.nodes.push_back(newNode);
    //add edge
    if (state->preState){
      VisEdge newEdge;
      newEdge.target = newNode.id;
      std::map<std::shared_ptr<State>, int>::iterator iter;
      iter = stateMap.find(state->preState);
      if (iter != stateMap.end()){
        newEdge.source = iter->second;
      }else{
        //place holder
      }
      m_gml.graph.edges.push_back(newEdge);
    }
  }


  std::string to_hex(int i){
    std::stringstream ioss;
    std::string s;
    ioss<<std::hex<<i;
    ioss>>s;
    if (s.size()==1){
      s = "0" + s;
    }else if (s.size()>2){
      //place holder
    }
    return s;
  }

  std::string Vis_GetColor(int step, int n){
    if (n == step - 1){
      return "\"#DC143C\"";
    }
    //set gradual color
    //terminal 0,0,0 -> 15,155,15
    float r_a=0,g_a=0,b_a=0;
    float r_b=15,g_b=155,b_b=15;
    int r_n,g_n,b_n;
    r_n = (int) r_a + (r_b-r_a)/step *n;
    g_n = (int) g_a + (g_b-g_a)/step *n;
    b_n = (int) b_a + (b_b-b_a)/step *n;
    std::string res = "";
    res += to_hex(r_n);
    res += to_hex(g_n);
    res += to_hex(b_n);
    return "\"#" + res + "\"";
  }


  void Vis::OutputGML(){
    //io
    std::string outPath = m_settings.outputDir + GetFileName(m_settings.aigFilePath);
    std::ofstream visFile;
    visFile.open(outPath + "_vis.gml");
    //header
    visFile<<"Creator\t"<<m_gml.Creator<<std::endl;
    visFile<<"Version\t"<<m_gml.Version<<std::endl;
    visFile<<"graph"<<std::endl;
    visFile<<"["<<std::endl;
      //graph info
      visFile<<"directed\t"<<m_gml.graph.directed<<std::endl;
      //nodes
      for (VisNode n:m_gml.graph.nodes){
        visFile<<"node"<<std::endl;
        visFile<<"["<<std::endl;
          visFile<<"id\t"<<n.id<<std::endl;
          visFile<<"graphics"<<std::endl;
          visFile<<"["<<std::endl;
            visFile<<"fill\t"<<Vis_GetColor(tempId, n.id)<<std::endl;
          visFile<<"]"<<std::endl;
          visFile<<"LabelGraphics"<<std::endl;
          visFile<<"["<<std::endl;
            visFile<<"text\t"<<"\""<<n.id<<"\""<<std::endl;
            visFile<<"color\t"<<"\"#F8F8FF\""<<std::endl;
          visFile<<"]"<<std::endl;
        visFile<<"]"<<std::endl;
      }
      //edges
      for (VisEdge e:m_gml.graph.edges){
        visFile<<"edge"<<std::endl;
        visFile<<"["<<std::endl;
          visFile<<"source\t"<<e.source<<std::endl;
          visFile<<"target\t"<<e.target<<std::endl;
          visFile<<"graphics"<<std::endl;
          visFile<<"["<<std::endl;
            visFile<<"type\t"<<"\"arc\""<<std::endl;
            visFile<<"arrow\t"<<"\"last\""<<std::endl;
          visFile<<"]"<<std::endl;
        visFile<<"]"<<std::endl;
      }
    visFile<<"]"<<std::endl;
    visFile.close();
  }
}