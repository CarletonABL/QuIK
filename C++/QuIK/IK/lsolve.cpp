//
//  lsolve.cpp
//  QuIK
//
// 	Subroutine for solving a linear system with damping.
//
//  Created by Steffan Lloyd on 2021-08-21.
//
#include "lsolve.hpp"

using namespace Eigen;

template <int DOF>
void lsolve( 	const Matrix<double,6,DOF>& A,
				const Vector<double,6>& b,
				Vector<double,DOF>& x,
			    const IKOptions& opt){
	
	// Form covariance matrix
	Matrix<double, 6, 6> Astar = A*A.transpose();
	
	// If a damping term is given, add it to the diagonals
	if (opt.lambda2 > 0) Astar.diagonal().array() += opt.lambda2;
	
	// Do LLT decomposition
	LLT<Matrix<double,6,6>> Astar_llt(Astar);
		
	// Solve
	x = A.transpose() * ( Astar_llt.solve(b) );
	
}
