#ifndef  _TETRAHEDRON_
#define _TETRAHEDRON_
#include "head.h"
class tetrahedron
{
 public:
  int index_vertex[4];
  Eigen::Matrix<double,3,3> X_inverse;
  Eigen::Matrix<double,3,3> P;
  Eigen::Matrix<double,9,12> A;
  int get_P;
  double vol;
  tetrahedron();
  ~tetrahedron();
};
#endif

