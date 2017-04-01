#ifndef TF_H_
#define TF_H_

#include <Eigen.h>        // Calls main Eigen matrix class library
#include <Eigen/LU>       // Calls inverse, determinant, LU decomp., etc.

#include <math.h>

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

namespace open_manipulator
{
class TF
{
 public:

 public:
  TF();
  ~TF();

  Eigen::Matrix3f skew(Eigen::Vector3f v);
  Eigen::Matrix3f calcRodrigues(Eigen::Vector3f axis, float angle);
};
}

#endif // TF_H_
