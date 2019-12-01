#include "vertex.h"
vertex::vertex()
{
  int i;
  for(i=0;i<4;i++)
    {
      this->index_tet[i]=-1;
    }
}

vertex::vertex(const myvector location)
{
  this->location_deform=location;
  this->location=location;
  int i;
  for(i=0;i<4;i++)
    {
      this->index_tet[i]=-1;
    }
}

vertex::~vertex()
{
  ;
}

