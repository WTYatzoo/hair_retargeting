#ifndef _GRID_
#define _GRID_

#include "myvector.h"
#include "tetrahedron.h"
class grid
{
 public:
  int index_vertex[2][2][2];
  tetrahedron mytet[5];
  int kind;
  grid(){;}
  ~grid(){;}
  
};
#endif
