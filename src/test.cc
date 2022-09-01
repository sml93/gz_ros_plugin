#include <iostream>
#include <math.h>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

double alpha = 60*(M_PI/180);

int main()
{
  MatrixXf matrixA(3,3);
  matrixA.setZero();
  // cout << "\n" << matrixA << endl;


  matrixA(0, 0) = cos(alpha);
  matrixA(0, 2) = sin(alpha);
  matrixA(1, 1) = 1;
  matrixA(2, 0) = -sin(alpha);
  matrixA(2, 2) = cos(alpha);
  cout << "\n" << matrixA << endl;

  MatrixXf matrixB(3,1);
  matrixB(0, 0) = 0;
  matrixB(1, 0) = 0;
  matrixB(2, 0) = 22;
  cout << "\n" << matrixB << endl;

  MatrixXf mat(3, 1);
  mat = matrixA * matrixB;
  cout << "\n" << mat << endl;
}