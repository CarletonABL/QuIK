//
//  IK.cpp
//  QuIK
//
// IK A basic IK implementation of the QuIK, NR and BFGS algorithms.
//
// Inputs:
//   	- Robot& R: A Robot object to perform the IK on.
//		- Matrix<double,4*N,4>& Twt: Tall matrix of all the desired robot poses
//			vertically stacked.
//   	- Matrix<double,DOF,N>& Q0: Initial guesses of the joint angles
//   	- IKOPtions& opt: An options array for the inverse kinematics algorithm.
//
// Outputs:
//   	- Matrix<double,DOF,N>& Qstar: [DOFxN] The solved joint angles
//		- Matrix<double,6,N>&e: The pose errors at the final solution.
//		- Vector<int,N> iter: The number of iterations the algorithm took.
//		- Vector<int,N> breakReason: The reason the algorithm stopped.
//               1 - Tolerance reached
//               2 - Min step size reached
//               3 - Max iterations reached
//               4 - Grad fails reached
//
//  Created by Steffan Lloyd on 2021-08-21.
//

#ifndef IK_hpp
#define IK_hpp

#include "Eigen/Dense"
#include "Robot.hpp"
#include "IKOptions.hpp"

using namespace Eigen;

template <int DOF=Dynamic>
void IK( 	const Robot<DOF>& R,
			const Matrix<double,Dynamic,4>& Twt,
			const Matrix<double,DOF,Dynamic>& Q0,
			const IKOptions& opt,
			Matrix<double,DOF,Dynamic>& Q_star,
			Matrix<double,6,Dynamic>& e_star,
			VectorXi& iter,
			VectorXi& breakReason);

#include "IK.cpp"

#endif /* IK_hpp */
