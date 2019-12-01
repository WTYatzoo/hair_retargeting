#ifndef _OBJECT_
#define _OBJECT_
#include "head.h"
#include "vertex.h"
#include "quad.h"


struct v
{
  double a,b,c;
  
};

struct curve
{
  std::string name;
  std::vector<vertex > curve_vertex;
};
struct e
{
  int v1,v2;
  double c0,c1,c2;
};

class object
{
 public:
  std::vector<vertex > myvertexs;
  std::vector<quad > myquads;
	
  int num_vertex;
  int num_face;
  std::vector<vertex > myvertexs_target;
  std::vector<quad > myquads_target;

  int num_vertex_target;
  int num_face_target;

  std::vector<vertex > myvertexs_head; // hair & head
  std::vector<int > myvertexs_head_index;
  int num_vertex_head;
  int num_vertex_hair_head; // all

  std::vector<vertex > myvertexs_hair;
  std::vector<e > myedges_hair;
  int num_vertex_hair;
  int num_edge_hair;
  
  //std::vector<v > myvertexs_hair_head;
  //std::vector<int > myvertexs_hair_head_index;
  //int num_vertex_hair_head;

  std::vector<curve > mycurves;
  int num_curve;
  
  object();
  ~object();
  void getObjData();
  void testdraw();
  void deform(std::vector< vertex >& myvertexs_head,double h, std::vector<std::pair< myvector, myvector> >& controlPointsPair);
};
#endif
