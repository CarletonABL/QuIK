//
//  hgtInv.cpp
// 	Computes the inverse of a 4x4 homogenious transformation matrix
//  Much faster than actually inverting it since the computations are easy
//  QuIK
//
//  Created by Steffan Lloyd on 2021-09-12.
//

#include "hgtInv.hpp"
#include "Eigen/Dense"

using namespace Eigen;

Matrix4d hgtInv( const Matrix4d& T ){
	Matrix4d Tinv;
	Tinv.topLeftCorner<3,3>() = T.topLeftCorner<3,3>().transpose();
	Tinv.topRightCorner<3,1>() = -Tinv.topLeftCorner<3,3>()*T.topRightCorner<3,1>();
	Tinv.bottomLeftCorner<1,3>().fill(0);
	Tinv(3,3) = 1;
	return Tinv;
}
