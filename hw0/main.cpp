#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

using namespace Eigen;
using namespace std;

int main(void) {
  const float PI = 4 * atan(1);
  cout << "Defined PI:\n" << PI << endl;

  cout << "Given a point P:\n";
  Vector3f p(2.0, 1.0, 1.0);
  cout << p << endl;

  cout << "Rotation matrix R:\n";
  Matrix3f r;
  r << cos(PI / 4), -sin(PI / 4), 0,
       sin(PI / 4),  cos(PI / 4), 0,
       0, 0, 1;
  cout << r << endl;

  cout << "Translation matrix T:\n";
  Matrix3f t;
  t << 1, 0, 1,
       0, 1, 2,
       0, 0, 1;
  cout << t << endl;

  cout << "Composed matrix RT:\n";
  Matrix3f rt;
  rt << cos(PI / 4), -sin(PI / 4), 1,
       sin(PI / 4),  cos(PI / 4), 2,
       0, 0, 1;
  cout << rt << endl;

  cout << "Transformed only by R:\n" << r * p << endl;

  cout << "Transformed only by T:\n" << t * p << endl;

  cout << "Transformed by R then T:\n" << t * r * p << endl;
  
  cout << "Transformed by RT:\n" << rt * p << endl;

  return 0;
}
