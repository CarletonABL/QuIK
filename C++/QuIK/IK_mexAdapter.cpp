/* IK_mexAdapter
 *
 * Computes the inverse kinematics.
 * Inputs:
 * 		DHx: 6 x 6 (a alpha d theta linktype Qsign),
 *		Tbase: 4 x 4 (base transform)
 * 		Ttool: 4 x 4 (tool transform)
 * 		Twt: 4N x 4 (vertically stacked samples)
 *		Q0: DOF x n, Initial guesses
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
 * 		Q_star: DOF x N, solved angles
 *		e_star: 6 x N, error at solved angles
 *		iter: int, N, number of iterations required
 *		breakReason: int, N, reason for breaking.
 */

#include "Robot.hpp"
#include "IK.hpp"
#include "IKOptions.hpp"
#include <mex.h>
#include "Eigen/Dense"
#include <iostream>
using namespace Eigen;
using namespace std;


template <int DOF=Dynamic>
void runFixedIK( 	const Array<double,Dynamic,4>& DH,
					const Vector<bool, Dynamic>& lt,
					const VectorXd& Qsign,
					const Matrix4d& Tbase,
					const Matrix4d& Ttool,
					const Matrix<double,Dynamic,4>& Twt,
					const Matrix<double,Dynamic,Dynamic>& Q0,
					const IKOptions& opt,
					MatrixXd& Q_star,
					Matrix<double,6,Dynamic>& e_star,
					VectorXi& breakReason,
					VectorXi& iter){
	
	// Init robot
	const Robot<DOF> R( DH.topRows<DOF>(), lt.topRows<DOF>(), Qsign.topRows<DOF>(), Tbase, Ttool);
		
	// Make place to store results
	Matrix<double,DOF,Dynamic> Q_star_fixed(DOF,Q_star.cols());
	
	// Call IK
	IK<DOF>(R, Twt, Q0.topRows<DOF>(), opt, Q_star_fixed, e_star, iter, breakReason );
	
	// Store results
	Q_star = Q_star_fixed;
}



void mexFunction(int numOutputs, mxArray *outputs[],
                 int numInputs, const mxArray *inputs[])
{
	// Initialize variables
	int N = (int) mxGetN(inputs[4]); // Number of columns in Q
	int DOF = (int) mxGetM(inputs[4]); // Number of rows in Q
	
	// Basic input checking
	if(numInputs!=6)
		 mexErrMsgIdAndTxt("Quik:wronginputs","Wrong number of inputs.");

	for (int k=0;k<4;k++){
		if ( (!mxIsDouble(inputs[k]) || mxIsComplex(inputs[k]) ))
			mexErrMsgIdAndTxt("Quik:notDouble","Input matrices must be type double.");
	}

	if (mxGetM(inputs[0]) != DOF || mxGetN(inputs[0]) != 6)
		mexErrMsgIdAndTxt("Quik:badDHx","DHx table must be DOFx6");
	if (mxGetM(inputs[1]) != 4 || mxGetN(inputs[1]) != 4)
		mexErrMsgIdAndTxt("Quik:badTbase","Tbase must be 4x4");
	if (mxGetM(inputs[2]) != 4 || mxGetN(inputs[2]) != 4)
		mexErrMsgIdAndTxt("Quik:badTtool","Ttool must be 4x4");
	if (mxGetM(inputs[3]) != N*4 || mxGetN(inputs[3]) != 4)
		mexErrMsgIdAndTxt("Quik:badTwt","Twt table must be DOF*Nx4");
	if (mxGetM(inputs[4]) != DOF)
		mexErrMsgIdAndTxt("Quik:badQ0","Q0 must be DOFxN");
	
	// Assign inputs
	MatrixXd Q0(DOF,N), Q_star(DOF,N);
	Matrix<double,Dynamic,4> Twt(4*N, 4);
	Array<double,Dynamic,6> DHx;
	DHx = Map<Matrix<double,Dynamic,6>>(mxGetDoubles(inputs[0]), DOF, 6);
	Matrix4d Tbase = Map<Matrix4d>(mxGetDoubles(inputs[1]));
	Matrix4d Ttool = Map<Matrix4d>(mxGetDoubles(inputs[2]));

	// Parse robot
	Array<double,Dynamic,4> DH = DHx.leftCols<4>();
	VectorXd Qsign = DHx.rightCols<1>();
	Vector<bool,Dynamic> lt = DHx.col(4) > 0.5;

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
	
	const mxArray* relImprovementTolPtr = mxGetField(inputs[5], 0, "relImprovementTol");
	if ( relImprovementTolPtr == NULL || mxGetClassID(relImprovementTolPtr) != mxDOUBLE_CLASS)
		mexErrMsgIdAndTxt("Quik:badField","relImprovementTol must be included and must be double");
	
	const mxArray* maxGradFailsPtr = mxGetField(inputs[5], 0, "maxGradFails");
	if ( maxGradFailsPtr == NULL || mxGetClassID(maxGradFailsPtr) != mxINT32_CLASS)
		mexErrMsgIdAndTxt("Quik:badField","maxGradFails must be included and must be int32");
	
	const mxArray* maxGradFailsTotalPtr = mxGetField(inputs[5], 0, "maxGradFailsTotal");
	if ( maxGradFailsPtr == NULL || mxGetClassID(maxGradFailsTotalPtr) != mxINT32_CLASS)
		mexErrMsgIdAndTxt("Quik:badField","maxGradFailsTotal must be included and must be int32");

	const mxArray* lambda2Ptr = mxGetField(inputs[5], 0, "lambda2");
	if ( lambda2Ptr == NULL || mxGetClassID(lambda2Ptr) != mxDOUBLE_CLASS)
		mexErrMsgIdAndTxt("Quik:badField","lambda2 must be included and must be double");

	const mxArray* maxLinearErrorStepPtr = mxGetField(inputs[5], 0, "maxLinearErrorStep");
	if ( maxLinearErrorStepPtr == NULL || mxGetClassID(maxLinearErrorStepPtr) != mxDOUBLE_CLASS)
		mexErrMsgIdAndTxt("Quik:badField","maxLinearErrorStep must be included and must be double");

	const mxArray* maxAngularErrorStepPtr = mxGetField(inputs[5], 0, "maxAngularErrorStep");
	if ( maxAngularErrorStepPtr == NULL || mxGetClassID(maxAngularErrorStepPtr) != mxDOUBLE_CLASS)
		mexErrMsgIdAndTxt("Quik:badField","maxAngularErrorStep must be included and must be double");

	const mxArray* armijoRuleSigmaPtr = mxGetField(inputs[5], 0, "armijoRuleSigma");
	if ( armijoRuleSigmaPtr == NULL || mxGetClassID(armijoRuleSigmaPtr) != mxDOUBLE_CLASS)
		mexErrMsgIdAndTxt("Quik:badField","armijoRuleSigma must be included and must be double");

	const mxArray* armijoRuleBetaPtr = mxGetField(inputs[5], 0, "armijoRuleBeta");
	if ( armijoRuleBetaPtr == NULL || mxGetClassID(armijoRuleBetaPtr) != mxDOUBLE_CLASS)
		mexErrMsgIdAndTxt("Quik:badField","armijoRuleBeta must be included and must be double");
	
	// Initialize options object
	IKOptions opt = IKOptions( 	*mxGetInt32s(iterMaxPtr),
								*mxGetInt32s(algorithmPtr),
								*mxGetDoubles(exitTolPtr),
								*mxGetDoubles(minStepSizePtr),
								*mxGetDoubles(relImprovementTolPtr),
								*mxGetInt32s(maxGradFailsPtr),
								*mxGetInt32s(maxGradFailsTotalPtr),
								*mxGetDoubles(lambda2Ptr),
								*mxGetDoubles(maxLinearErrorStepPtr),
								*mxGetDoubles(maxAngularErrorStepPtr),
								*mxGetDoubles(armijoRuleSigmaPtr),
								*mxGetDoubles(armijoRuleBetaPtr));
	
	// Init outputs
	Matrix<double,6,Dynamic> e_star(6, N);
	VectorXi iter(N), breakReason(N);
	
	// Robot

	// Run IK
	// Because the code runs faster as fixed-size, we compile fixed-size templates\
	// for the most common manipulator sizes. If the size is not in this list, variable-
	// sized code is run instead (usually about 10% slower)
	switch (DOF) {
			
		case 6: runFixedIK<6>( DH, lt, Qsign, Tbase, Ttool, Twt, Q0, opt, Q_star, e_star, breakReason, iter );
			break;
			
		case 7: runFixedIK<7>( DH, lt, Qsign, Tbase, Ttool, Twt, Q0, opt, Q_star, e_star, breakReason, iter ); break;
			
		case 8: runFixedIK<8>( DH, lt, Qsign, Tbase, Ttool, Twt, Q0, opt, Q_star, e_star, breakReason, iter ); break;
			
		case 9: runFixedIK<9>( DH, lt, Qsign, Tbase, Ttool, Twt, Q0, opt, Q_star, e_star, breakReason, iter ); break;
			
		case 10: runFixedIK<10>( DH, lt, Qsign, Tbase, Ttool, Twt, Q0, opt, Q_star, e_star, breakReason, iter ); break;
			
		case 16: runFixedIK<16>( DH, lt, Qsign, Tbase, Ttool, Twt, Q0, opt, Q_star, e_star, breakReason, iter ); break;
			
		default:{
			// Run with variable sized
			 const Robot<Dynamic> R( DH, lt, Qsign, Tbase, Ttool);
			 IK<Dynamic>(R, Twt, Q0, opt, Q_star, e_star, iter, breakReason );
		}
	}
	
	// Assign outputs
	double *Q_star_o;
	double *e_star_o;
	int *iter_o;
	int *breakReason_o;

	// Assign outputs
	if (numOutputs >= 1){
		outputs[0] = mxCreateNumericMatrix(DOF, N, mxDOUBLE_CLASS, mxREAL);
		Q_star_o = mxGetDoubles(outputs[0]);
		memcpy( Q_star_o, Q_star.data(), sizeof( double )*N*DOF);
	}
	if (numOutputs >= 2){
		outputs[1] = mxCreateNumericMatrix(6, N, mxDOUBLE_CLASS, mxREAL);
		e_star_o = mxGetDoubles(outputs[1]);
		memcpy( e_star_o, e_star.data(), sizeof( double )*N*6);
	}
	if (numOutputs >= 3){
		outputs[2] = mxCreateNumericMatrix(N, 1, mxINT32_CLASS, mxREAL);
		iter_o = mxGetInt32s(outputs[2]);
		memcpy( iter_o, iter.data(), sizeof( int )*N);
	}
	if (numOutputs >= 4){
		outputs[3] = mxCreateNumericMatrix(N, 1, mxINT32_CLASS, mxREAL);
		breakReason_o = mxGetInt32s(outputs[3]);
		memcpy( breakReason_o, breakReason.data(), sizeof( int )*N);
	}
}
