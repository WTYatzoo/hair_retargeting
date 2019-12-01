#ifndef _VERTEX_GRID_
#define _VERTEX_GRID_
#include "myvector.h"
class vertex_grid
{
 public:
  myvector location;
  myvector location_deform;
  
  vertex_grid();
  vertex_grid(const myvector location);
  ~vertex_grid();
};
#endif
