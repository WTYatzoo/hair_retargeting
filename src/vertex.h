#ifndef _VERTEX_
#define _VERTEX_
#include "myvector.h"
class vertex
{
 public:
  myvector location;
  myvector location_deform;
  int index_tet[4];
  vertex();
  vertex(const myvector location);
  ~vertex();
};

#endif
