//
//  IKOptions.hpp
//  QuIK
//
// IKOPTIONS Builds a structure that can be used as an input to the
// inverse kinematics solver.
//
//       * iterMax [int]: Maximum number of iterations of the
//           algorithm. Default: 100
//       * algorithm [int]: The algorithm to use
//           0 - QuIK
//           1 - Newton-Raphson or Levenberg-Marquardt
//           2 - BFGS
//           Default: 0.
//       * exitTol [double]: The exit tolerance on the norm of the
//           error. Default: 1e-12.
//       * minStepSize [double]: The minimum joint angle step size
//           (normed) before the solver exits. Default: 1e-14.
//       * relImprovementTol [double]: The minimum relative
//           iteration-to-iteration improvement. If this threshold isn't
//           met, a counter is incremented. If the threshold isn't met
//           [maxGradFails] times in a row, then the algorithm exits.
//           For example, 0.05 represents a minimum of 5// relative
//           improvement. Default: 0.05.
//       * maxGradFails [int]: The maximum number of relative
//           improvement fails before the algorithm exits. Default:
//           20.
//       * lambda2 [double]: The square of the damping factor, lambda.
//           Only applies to the NR and QuIK methods. If given, these
//           methods become the DNR (also known as levenberg-marquardt)
//           or the DQuIK algorithm. Ignored for BFGS algorithm.
//           Default: 0.
//       * maxLinearErrorStep [double]: An upper limit of the error step
//           in a single step. Ignored for BFGS algorithm. Default: 0.3.
//       * maxAngularErrorStep [double]: An upper limit of the error step
//           in a single step. Ignored for BFGS algorithm. Default: 1.
//       * armijoRuleSigma [double]: The sigma value used in armijo's
//           rule, for line search in the BFGS method. Default: 1e-5
//       * armijoRuleBeta [double]: The beta value used in armijo's
//           rule, for line search in the BFGS method. Default: 0.5
//
//  Created by Steffan Lloyd on 2021-08-21.
//

#include "IKOptions.hpp"
#include <iostream>
#include "Eigen/Dense"

using namespace std;
using namespace Eigen;

IKOptions::IKOptions(	int _iterMax,
						int _algorithm,
						double _exitTol,
						double _minStepSize,
						double _relImprovementTol,
						int _maxGradFails,
						int _maxGradFailsTotal,
						double _lambda2,
						double _maxLinearErrorStep,
						double _maxAngularErrorStep,
					    double _armijoRuleSigma,
					    double _armijoRuleBeta){
	iterMax = _iterMax;
	algorithm = _algorithm;
	exitTol = _exitTol;
	minStepSize = _minStepSize;
	relImprovementTol = _relImprovementTol;
	maxGradFails = _maxGradFails;
	maxGradFailsTotal = _maxGradFailsTotal;
	lambda2 = _lambda2;
	maxLinearErrorStep = _maxLinearErrorStep;
	maxAngularErrorStep = _maxAngularErrorStep;
	armijoRuleSigma = _armijoRuleSigma;
	armijoRuleBeta = _armijoRuleBeta;
	
}

void IKOptions::print() const{
	cout << "opt.iterMax: " << iterMax << endl;
	cout << "opt.algorithm: " << algorithm << endl;
	cout << "opt.exitTol: " << exitTol << endl;
	cout << "opt.minStepSize: " << minStepSize << endl;
	cout << "opt.relImprovementTol: " << relImprovementTol << endl;
	cout << "opt.maxGradFails: " << maxGradFails << endl;
	cout << "opt.lambda2: " << lambda2 << endl;
	cout << "opt.maxLinearErrorStep: " << maxLinearErrorStep << endl;
	cout << "opt.maxAngularErrorStep: " << maxAngularErrorStep << endl;
	cout << "opt.armijoRuleSigma: " << armijoRuleSigma << endl;
	cout << "opt.armijoRuleBeta: " << armijoRuleBeta << endl;
}
