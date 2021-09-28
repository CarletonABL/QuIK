//
//  example.cpp
//  Example code showing how the IK functions can be called.
//
//  Created by Steffan Lloyd on 2021-08-20.
//

#include <iostream>
#include "Eigen/Dense"
#include <math.h>
#include "Robot.hpp"
#include "IKOptions.hpp"
#include "IK.hpp"

using namespace std;
using namespace Eigen;

// Define manipulator.
// This is the DH parameters for the KUKA KR6 robot
const Robot<6> R(
	// Given as DOFx4 table, in the following order: a_i, alpha_i, d_i, theta_i.
	(Matrix<double, 6, 4>() <<
	  1./40,	-M_PI/2, 	183./1000,	0,
	 -63./200,	0,        	0,			0,
	 -7./200,	M_PI/2,		0,			0,
	 0,			-M_PI/2,	73./200,	0,
	 0,  		M_PI/2,		0,			0,
	 0,  		0,			2./25,		0).finished(),
					  
	// Second argument is a list of joint types
	// true is prismatic, false is revolute
	// KUKA KR6 only has revolute joints
	(Vector<bool,6>() << false, false, false, false, false, false).finished(),

	// Third agument is a list of joint directions
	// Allows you to change the sign (direction) of
	// the joints.
	(Vector<double,6>(6) << 1, 1, 1, 1, 1, 1 ).finished(),

	// Fourth and fifth arguments are the base and tool transforms, respectively
	Matrix4d::Identity(4,4),
	Matrix4d::Identity(4,4)
);

// Define the IK options
const IKOptions opt = IKOptions( 	200, // max number of iterations
									0, // algorithm (0 = QuIK, 1 = Newton-Raphson, 2 = BFGS)
									1e-12, // Exit tolerance
									1e-14, // Minimum step tolerance
									0.05, // iteration-to-iteration improvement tolerance (0.05 = 5% relative improvement)
									10, // max consequitive gradient fails
									80, // Max gradient fails
									1e-10, // lambda2 (lambda^2, the damping parameter for DQuIK and DNR)
									0.34, // Max linear error step
									1, // Max angular error step
									1e-5, // Sigma value for armijo rule in BFGS line search
									0.5 // beta value for armijo rule in BFGS line search
								);

int main()
{
	
	// Initilize variables
	int N = 5; // Number of poses to generate
	int DOF = R.dof;
	Matrix<double,6,Dynamic>	Q(DOF, N),	// True joint angles
								Q0(DOF, N),	// Initial guess of joint angles
								Q_star(DOF, N);	// Solver's solution
	Matrix<double,6,Dynamic> e_star(6,N);	// Error at solver pose
	Vector<int,Dynamic> iter(N),	// Store number of iterations of algorithm
						breakReason(N);	// Store break out reason
	Matrix4d 	T,		// True forward kinematics transform
				T_star;	// Forward kinematics at solver solution
	
	Matrix<double,Dynamic,4> Tn(N*4,4); 	// 4N * 4 matrix of vertically stacked transforms to be solved.
							// This is just a convenient way of sending in an array of transforms.
	
	// Generate some random joint configurations for the robot
	Q.setRandom(DOF, N);
	
	// Perturb true answers slightly to get initial "guess"
	Q0 = Q.array() + 0.1;
		
	// Do forward kinematics of each Q sample and store in the "tall" matrix
	// of transforms, Tn
	for (int i = 0; i < N; i++){
		R.FKn( Q.col(i), T );
		Tn.middleRows<4>(i*4) = T;
	}
		
	// Solve using the IK function, store results in Q_star, e_star, iter and breakreason
	IK(R, Tn, Q0, opt, Q_star, e_star, iter, breakReason);
	
	// Print out results
	cout << "Joint angles (start):" << endl;
	cout << Q0 << endl << endl;
	cout << "Joint angles (true):" << endl;
	cout << Q << endl << endl;
	cout << "The final joint angles are: " << endl;
	cout << Q_star << endl << endl;
	cout << "Final error is: " << endl << e_star.array().square().colwise().sum().sqrt() << endl << endl;
	cout << "Break reason is: " << endl  << breakReason.transpose() << endl << endl;
	cout << "Number of iterations: " << endl  << iter.transpose() << endl << endl;
	cout << "Program finished!" << endl;
	
	return 0;
}
