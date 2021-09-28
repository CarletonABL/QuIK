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

#ifndef IKOptions_hpp
#define IKOptions_hpp

#include "Eigen/Dense"
using namespace Eigen;

class IKOptions{
public:
	int iterMax;
	int algorithm;
	double exitTol;
	double minStepSize;
	double relImprovementTol;
	int maxGradFails;
	int maxGradFailsTotal;
	double lambda2;
	double maxLinearErrorStep;
	double maxAngularErrorStep;
	double armijoRuleSigma;
	double armijoRuleBeta;
	
	// Constructor
	IKOptions(	int _iterMax = 100,
				int _algorithm = 0,
				double _exitTol = 1e-12,
				double _minStepSize = 1e-14,
				double _relImprovementTol = 0.05,
				int _maxGradFails = 5,
				int _maxGradFailsTotal = 20,
				double _lambda2 = 0,
				double _maxLinearErrorStep = .34,
				double _maxAngularErrorStep = 1,
				double _armijoRuleSigma = 1e-5,
			    double _armijoRuleBeta = 0.5 );
	
	// Prints out the options
	void print() const;
};

#include "IKOptions.cpp"

#endif /* IKOptions_hpp */
