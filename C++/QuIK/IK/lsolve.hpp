//
//  lsolve.hpp
//  QuIK
//
// 	Subroutine for solving a linear system with damping.
//
//  Created by Steffan Lloyd on 2021-08-21.
//

#ifndef lsolve_hpp
#define lsolve_hpp

#include "Eigen/Dense"
#include "IKOptions.hpp"

using namespace Eigen;

template <int DOF>
void lsolve( 	const Matrix<double,6,DOF>& A,
				const Vector<double,6>& b,
				const Vector<double,DOF>& x,
				const IKOptions& opt);

#include "lsolve.cpp"

#endif /* lsolve_hpp */
