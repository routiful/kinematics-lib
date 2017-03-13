#include <Eigen.h>        // Calls main Eigen matrix class library
#include <Eigen/LU>       // Calls inverse, determinant, LU decomp., etc.
using namespace Eigen;    // Eigen related statement; simplifies syntax for declaration of matrices

float inByte = 0.0;

void setup()
{
  Serial.begin(57600);
}

void loop()
{
  Serial.write('S');
  delay(1000);
}
