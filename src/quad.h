#ifndef _QUAD_
#define _QUAD_

class quad
{
 public:
  int index_vertex[4];
  quad()
    {
      ;
    }
	
  quad(int index0,int index1,int index2, int index3)
    {
      index_vertex[0]=index0; index_vertex[1]=index1; index_vertex[2]=index2; index_vertex[3]=index3;
    }
  quad(const int (&index_vertex)[4])
    {
      int i;
      for(i=0;i<4;++i)
	{
	  this->index_vertex[i]=index_vertex[i];
	}
      return;
    }
  ~quad()
    {

    }
};
#endif
