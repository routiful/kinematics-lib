#include "tf.h"
using namespace open_manipulator;

TF::TF()
{
}

TF::~TF()
{

}

Eigen::Matrix3f TF::skew(Eigen::Vector3f v)
{
  Eigen::Matrix3f skew_symmetric_matrix = Eigen::Matrix3f::Zero();

  skew_symmetric_matrix <<     0,  -v(2),  v(1),
                           v(2),      0, -v(0),
                          -v(1),   v(0),     0;

  return skew_symmetric_matrix;
}

Eigen::Matrix3f TF::calcRodrigues(Eigen::Vector3f axis, float angle)
{
  Eigen::Matrix3f skew_symmetric_matrix = Eigen::Matrix3f::Zero();
  Eigen::Matrix3f rotation_matrix = Eigen::Matrix3f::Zero();
  Eigen::Matrix3f Identity_matrix = Eigen::Matrix3f::Identity();

  skew_symmetric_matrix = skew(axis);
  rotation_matrix = Identity_matrix + skew_symmetric_matrix * sin(angle) + skew_symmetric_matrix * skew_symmetric_matrix * (1 - cos(angle));

  return rotation_matrix;
}
