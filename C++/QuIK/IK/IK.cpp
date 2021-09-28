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
//               4 - Grad fails reached (consecutive)
//				 5 - rcond limit reached, solution is likely singular
//				 6 - Grad fails reached (total)
//
//  Created by Steffan Lloyd on 2021-08-21.
//


#include "IK.hpp"
#include "hgtDiff.hpp"
#include "hgtInv.hpp"
#include "Eigen/Dense"
#include "Robot.hpp"
#include "IKOptions.hpp"
#include "clampMag.hpp"
#include "lsolve.hpp"
#include <iostream>

using namespace Eigen;
using namespace std;

template <int DOF>
void IK( 	const Robot<DOF>& R,
			const Matrix<double,Dynamic,4>& Twt,
			const Matrix<double,DOF,Dynamic>& Q0,
			const IKOptions& opt,
			Matrix<double,DOF,Dynamic>& Q_star,
			Matrix<double,6,Dynamic>& e_star,
			VectorXi& iter,
			VectorXi& breakReason){

	// Get size of problem
	int N = (int) Q0.cols();
	
	// Define function variables
	Vector<double,DOF> Q_i, dQ_i, s0;
	constexpr int DOF4 = DOF>0 ? (DOF+1)*4 : -1;
	Matrix<double,DOF4,4> T_i((R.dof+1)*4, 4);
	Matrix4d Twt_i;
	Vector<double,6> e_i, Hg_i;
	Vector<double,DOF>  grad_i, grad_ip1, y;
	Matrix<double,6,DOF> J_i(6, R.dof), A_i;
	Matrix<double,DOF,DOF> H_i;
	int 	iter_i,
			breakReason_i,
			grad_fail_counter,
			grad_fail_counter_total;
	double 	e_i_norm = 0,
			e_i_prev_norm,
			error_relImprovement = 0,
			cost_i = 1e10,
			cost_ip1 = 1e10,
			gamma,
			rho,
			delta;
	

	// Start iterations over samples
	for (int i = 0; i < N; i++){
	
		// Start solver
		Q_i = Q0.col(i);
		e_i.fill(0);
		dQ_i.fill(0);
		iter_i = opt.iterMax;
		breakReason_i = 3;
		e_i_prev_norm = 1e10;
		grad_fail_counter = 0;
		grad_fail_counter_total = 0;
		Twt_i = Twt.middleRows<4>(4*i);
		
		// Start IK iterations
		for (int i = 0; i < opt.iterMax; i++){
			
			// Get error, forward kinematics and jacobian
			// Only do this for Newton and QuIK, or on first iteration
			if (opt.algorithm != 2 || i == 0){
				// Forward kinematics
				R.FK( Q_i, T_i );
				
				// Get jacobian (needed for all algorithms)
				R.jacobian(T_i, J_i, true);
				
				// Get error
				hgtDiff( T_i.template bottomRows<4>(), Twt_i, e_i, opt );
			}
			
			// Calculate norm
			e_i_norm = e_i.norm();

			// Break, if exit tolerance has been reached
			if (e_i_norm < opt.exitTol){
				breakReason_i = 1; // Tolerance reached
				iter_i = i;
				break;
			}
			
			// Check relative improvement in error
			// We break if the relative improvement fails opt.maxGradFails times in a row, or if
			// it fails opt.maxGradFailsTotal total
			error_relImprovement = (e_i_prev_norm - e_i_norm) / e_i_prev_norm;
			if (error_relImprovement < opt.relImprovementTol){
				// If relative improvement is below threshold, increment counters
				grad_fail_counter++;
				grad_fail_counter_total++;
				if (grad_fail_counter > opt.maxGradFails) {
					breakReason_i = 4; // Grad consecutive fails reached
					iter_i = i;
					break;
				}
				if (grad_fail_counter_total > opt.maxGradFailsTotal) {
					breakReason_i = 6; // Grad fails reached
					iter_i = i;
					break;
				}
			}else{
				grad_fail_counter = 1;
			}

			// Store prev value
			e_i_prev_norm = e_i_norm;
			
			// Clamp error
			clampMag(e_i, opt);
			
			// Go to switch statement to do work of each individual algorithm
			switch (opt.algorithm){
					
					
				case 0:
					// Halley's method
					
					// First, store the newton step in dQ_i (note, it's negative)
					lsolve( J_i, e_i, dQ_i, opt);
					
					// Then, negate it and divide by two
					dQ_i *= -0.5;
					
					// Assign jacobian to A_i so that it gets added to it
					A_i = J_i;

					// Get gradient product
					R.hessianProduct( J_i, dQ_i, A_i );
										
					// Resolve
					lsolve(A_i, e_i, dQ_i, opt);
					dQ_i *= -1;
					
					break;
					
					
					
				case 1:
					// Newton's method
					lsolve( J_i, e_i, dQ_i, opt);
					dQ_i *= -1;
					break;
					
					
					
				case 2:
					// BFGS
					// On first iteration, initialize some variables
					if (i == 0){
						H_i = Matrix<double,DOF,DOF>::Identity(R.dof,R.dof);
						grad_i = J_i.transpose() * e_i;
						cost_i = 0.5*e_i.array().square().sum();
					}
					
					// Get initial step
					s0 = -H_i*grad_i;
					
					// Initialize line search
					gamma = 1;
					
					// Recalculate cost and error
					R.FK( Q_i + gamma*s0, T_i );
					hgtDiff( T_i.template bottomRows<4>(), Twt_i, e_i, opt );
					cost_ip1 = 0.5*e_i.array().square().sum();
					
					// Do line search
					while ((cost_i - cost_ip1) < -opt.armijoRuleSigma * grad_i.transpose()*(gamma*s0)){
						// Reduce gamma
						gamma = opt.armijoRuleBeta * gamma;
						
						// Break if step size is too small (prevents infinite loops too)
						if (gamma < opt.minStepSize) break;
						
						// Recalculate cost
						R.FK( Q_i + gamma*s0, T_i );
						hgtDiff( T_i.template bottomRows<4>(), Twt_i, e_i, opt );
						cost_ip1 = 0.5*e_i.array().square().sum();
					}
					
					// Break out if step size is too small
					if (gamma < opt.minStepSize){
						breakReason_i = 2; // Grad fails reached
						iter_i = i;
						break;
					}
					
					// Take step
					dQ_i = gamma*s0;
					
					// Update gradient (T_i and e_i are already updated)
					R.jacobian(T_i, J_i);
					grad_ip1 = J_i.transpose() * e_i;
					
					// Update gradient
					y = grad_ip1 - grad_i;
					rho = dQ_i.transpose() * y;
					delta = y.transpose() * H_i * y;
					if (rho > delta && rho > numeric_limits<double>::epsilon())
						H_i = H_i + ( (1 + delta/rho) * dQ_i*dQ_i.transpose() - dQ_i*y.transpose()*H_i - H_i*y*dQ_i.transpose())/rho;
					else if (delta > numeric_limits<double>::epsilon() && rho > numeric_limits<double>::epsilon())
						H_i = H_i + (dQ_i*dQ_i.transpose())/rho - H_i*(y*y.transpose())*H_i/delta;
					
					// Update variables for next time
					grad_i = grad_ip1;
					cost_i = cost_ip1;
					
					break;

					
				default:
					// invalid input
					cout << "Invalid algorithm specified!" << endl;
					dQ_i.fill(0);
					
					
			} // end of algorithm switch statement
						
			// Apply change
			Q_i += dQ_i;
			
			// Check grad tolerance, break if necessary
			if (dQ_i.array().square().sum() < opt.minStepSize * opt.minStepSize){
				breakReason_i = 2; // minimum step sized reached
				iter_i = i;
				break;
			}
			
		} // End of IK loop
		
		// Store solutions
		Q_star.col(i) = Q_i;
		e_star.col(i) = e_i;
		iter(i) = iter_i;
		breakReason(i) = breakReason_i;
		
	} // End of sample loop
	
} // End of function
