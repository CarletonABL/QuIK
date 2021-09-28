//
//  hgtInv.hpp
// 	Computes the inverse of a 4x4 homogenious transformation matrix
//  Much faster than actually inverting it since the computations are easy
//  QuIK
//
//  Created by Steffan Lloyd on 2021-09-12.
//

#ifndef hgtInv_hpp
#define hgtInv_hpp
#include "Eigen/Dense"

using namespace Eigen;

Matrix4d hgtInv( const Matrix4d& T );

#include "hgtInv.cpp"

#endif /* hgtInv_hpp */
