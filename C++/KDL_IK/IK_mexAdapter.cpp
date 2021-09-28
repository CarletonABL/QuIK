/* IK_mexAdapter
 *
 * MEX adapter for matlab to compute the inverse kinematics using KDL's solvers
 *
 * Inputs:
 * 		DHx: 6 x 6 (a alpha d theta linktype Qsign),
 *		Tbase: 4 x 4 (base transform)
 * 		Ttool: 4 x 4 (tool transform)
 * 		Twt: 4N x 4 (vertically stacked samples)
 *		Q0: 6 x n, Initial guesses
 * 		opt: struct with fields
 * 			- iterMax
 * 			- algorithm
 * 			- exitTol
 * 			- gradExitTol
 * 			- relImprovement
 * 			- maxGradFails
 * 			- lambda2
 * 			- linAlg
 * 			- maxLinearErrorStep
 * 			- maxAngularErrorStep
 * Outputs:
 * 		Q_star: 6 x N, solved angles
 *		e_star: double, N, error at solved angles
 *		breakReason: int, N, reason for breaking.
 */

#include <mex.h>
#include "Eigen/Dense"
#include "chainiksolverpos_lma.hpp"
#include "chainfksolverpos_recursive.hpp"
#include "chainiksolverpos_nr.hpp"
#include "chainiksolvervel_pinv.hpp"
#include "chain.hpp"
#include "frames_io.hpp"
#include <stdio.h>
#include <iostream>
using namespace Eigen;
using namespace std;

void assignFrame( const Matrix4d& T, KDL::Frame& F);

void mexFunction(int numOutputs, mxArray *outputs[],
                 int numInputs, const mxArray *inputs[])
{
	
	
	// Basic input checking
	if(numInputs!=6)
		 mexErrMsgIdAndTxt("Quik:wronginputs","Wrong number of inputs.");
	
	// Initialize variables
	int N = (int) mxGetN(inputs[4]); // Number of columns in Q
	int DOF = (int) mxGetM(inputs[4]); // Number of rows in Q
	
	// Double check that inputs are double
	for (int k=0;k<4;k++){
		if ( (!mxIsDouble(inputs[k]) || mxIsComplex(inputs[k]) ))
			mexErrMsgIdAndTxt("Quik:notDouble","Input matrices must be type double.");
	}

	// Double check sizes
	if (mxGetM(inputs[0]) != DOF || mxGetN(inputs[0]) != 6)
		mexErrMsgIdAndTxt("Quik:badDHx","DHx table must be DOFx6");
	if (mxGetM(inputs[1]) != 4 || mxGetN(inputs[1]) != 4)
		mexErrMsgIdAndTxt("Quik:badTbase","Tbase must be 4x4");
	if (mxGetM(inputs[2]) != 4 || mxGetN(inputs[2]) != 4)
		mexErrMsgIdAndTxt("Quik:badTtool","Ttool must be 4x4");
	if (mxGetM(inputs[3]) != N*4 || mxGetN(inputs[3]) != 4)
		mexErrMsgIdAndTxt("Quik:badTwt","Twt table must be DOF*Nx4");
	if (mxGetM(inputs[4]) != DOF)
		mexErrMsgIdAndTxt("Quik:badQ0","Q0 must be 6xN");
	
	// Assign inputs
	MatrixXd Q0(DOF,N), Q_star(DOF,N);
	MatrixXd Twt(4*N, 4);
	Array<double,Dynamic,6> DHx;
	DHx = Map<Matrix<double,Dynamic,6>>(mxGetDoubles(inputs[0]), DOF, 6);
	Matrix4d Tbase = Map<Matrix4d>(mxGetDoubles(inputs[1]));
	Matrix4d Ttool = Map<Matrix4d>(mxGetDoubles(inputs[2]));

	// Parse robot
	Matrix<double,Dynamic,4> DH = DHx.leftCols<4>();
	
	// Base transform
	KDL::Chain chain;
	KDL::Frame Tbase_f = KDL::Frame();
	assignFrame(Tbase, Tbase_f);
	chain.addSegment( KDL::Segment("base", KDL::Joint(KDL::Joint::None), Tbase_f) );
	
	// Add links
	for (int i = 0; i<DOF; i++){
		chain.addSegment( KDL::Segment( 	KDL::Joint(KDL::Joint::RotZ),
											KDL::Frame::DH( DH(i,0), DH(i,1), DH(i,2), DH(i,3) ) ) );
	}
	
	// Add tool transform
	KDL::Frame Ttool_f = KDL::Frame();
	assignFrame(Ttool, Ttool_f);
	chain.addSegment( KDL::Segment("tool", KDL::Joint(KDL::Joint::None), Ttool_f) );
	
	// Parse numeric inputs
	Twt = Map<Matrix<double,Dynamic,4>>(mxGetDoubles(inputs[3]), 4*N, 4);
	Q0 = Map<MatrixXd>(mxGetDoubles(inputs[4]), DOF, N);

	// Parse options
	const mxArray* iterMaxPtr = mxGetField(inputs[5], 0, "iterMax");
	if ( iterMaxPtr == NULL || mxGetClassID(iterMaxPtr) != mxINT32_CLASS)
		mexErrMsgIdAndTxt("Quik:badField","iterMax must be included and must be int32");

	const mxArray* algorithmPtr = mxGetField(inputs[5], 0, "algorithm");
	if ( algorithmPtr == NULL || mxGetClassID(algorithmPtr) != mxINT32_CLASS)
		mexErrMsgIdAndTxt("Quik:badField","algorithm must be included and must be int32");

	const mxArray* exitTolPtr = mxGetField(inputs[5], 0, "exitTol");
	if ( exitTolPtr == NULL || mxGetClassID(exitTolPtr) != mxDOUBLE_CLASS)
		mexErrMsgIdAndTxt("Quik:badField","exitTol must be included and must be double");

	const mxArray* minStepSizePtr = mxGetField(inputs[5], 0, "minStepSize");
	if ( minStepSizePtr == NULL || mxGetClassID(minStepSizePtr) != mxDOUBLE_CLASS)
		mexErrMsgIdAndTxt("Quik:badField","minStepSize must be included and must be double");
	
	const mxArray* maxLinearErrorStepPtr = mxGetField(inputs[5], 0, "maxLinearErrorStep");
	if ( maxLinearErrorStepPtr == NULL || mxGetClassID(maxLinearErrorStepPtr) != mxDOUBLE_CLASS)
		mexErrMsgIdAndTxt("Quik:badField","maxLinearErrorStep must be included and must be double");

	const mxArray* maxAngularErrorStepPtr = mxGetField(inputs[5], 0, "maxAngularErrorStep");
	if ( maxAngularErrorStepPtr == NULL || mxGetClassID(maxAngularErrorStepPtr) != mxDOUBLE_CLASS)
		mexErrMsgIdAndTxt("Quik:badField","maxAngularErrorStep must be included and must be double");

	// Init outputs
	VectorXd e_star(N);
	VectorXi iter(N), breakReason(N);
	
	// Assign outputs
	double *Q_star_o;
	double *e_star_o;
	int *breakReason_o;
	
	// Initialize intermediate vars
	Matrix4d Ti;
	KDL::Frame Ti_f = KDL::Frame();
	KDL::JntArray 	q0 = KDL::JntArray(DOF),
					q_star = KDL::JntArray(DOF);
	
	// Initialize solvers
	
	// Auto-LMA
	KDL::ChainIkSolverPos_LMA solver_lma(chain,
		Eigen::Vector<double,6>::Ones(6,1), // weights
		*mxGetDoubles(exitTolPtr), // exitTol
		*mxGetInt32s(iterMaxPtr), // itermax
		*mxGetDoubles(minStepSizePtr) // min step size
	);
	
	// NR
	KDL::ChainFkSolverPos_recursive fk_solver(chain); // Forward kin. solver
	KDL::ChainIkSolverVel_pinv vik_solver(chain); // PseudoInverse vel solvers
	KDL::ChainIkSolverPos_NR solver_nr(	chain,
										fk_solver,
										vik_solver,
										*mxGetInt32s(iterMaxPtr),
										*mxGetDoubles(exitTolPtr)); // Joint
	
	
	// Run IK
	for (int i = 0; i<N; i++){
		// translate data to KDL joint array format
		q0.data = Q0.col(0);
		q_star.data.fill(0);
		Ti = Twt.middleRows<4>(i*4);
		assignFrame( Ti, Ti_f );
		
		// Run IK
		switch (*mxGetInt32s(algorithmPtr)){
			case 1:
				breakReason(i) = solver_nr.CartToJnt(q0, Ti_f, q_star);
				break;
			case 3:
				// Do the IK
				breakReason(i) = solver_lma.CartToJnt(q0, Ti_f, q_star);
				break;
			default:
				mexErrMsgIdAndTxt("Quik:badalgorithm","algorithm must be either 1 (NR) or 3 (LMA)");
		}
		
		// Assign outputs
		Q_star.col(i) = q_star.data;
		e_star(i) = 0; // solver_nr.lastDifference;
	}

	// Assign outputs
	if (numOutputs >= 1){
		outputs[0] = mxCreateNumericMatrix(DOF, N, mxDOUBLE_CLASS, mxREAL);
		Q_star_o = mxGetDoubles(outputs[0]);
		memcpy( Q_star_o, Q_star.data(), sizeof( double )*N*DOF);
	}
	if (numOutputs >= 2){
		outputs[1] = mxCreateNumericMatrix(N, 1, mxDOUBLE_CLASS, mxREAL);
		e_star_o = mxGetDoubles(outputs[1]);
		memcpy( e_star_o, e_star.data(), sizeof( double )*N);
	}
	if (numOutputs >= 3){
		outputs[2] = mxCreateNumericMatrix(N, 1, mxINT32_CLASS, mxREAL);
		breakReason_o = mxGetInt32s(outputs[2]);
		memcpy( breakReason_o, breakReason.data(), sizeof( int )*N);
	}
}

// Helper function to assign an Eigen Matrix4d to a frame F
void assignFrame( const Matrix4d& T, KDL::Frame& F){
	F.M = KDL::Rotation( T(0,0), T(0,1), T(0,2),
						 T(1,0), T(1,1), T(1,2),
						T(2,0), T(2,1), T(2,2));
	F.p = KDL::Vector ( T(0,3), T(1,3), T(2,3) );
}
