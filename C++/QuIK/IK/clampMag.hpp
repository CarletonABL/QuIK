//
//  clampMag.cpp
//  QuIK
//
// 	Saturates the magnitude of the error vector before being
// 	sent to the rest of the algorithm.
//
// 	Implemented as defined here:
// 	[1] S. R. Buss, “Introduction to Inverse Kinematics
//	with Jacobian Transpose, Pseudoinverse and Damped Least
//	Squares methods,” 2009.
// 	https://www.math.ucsd.edu/~sbuss/ResearchWeb/ikmethods/iksurvey.pdf
//
//  Created by Steffan Lloyd on 2021-08-21.
//

#ifndef clampMag_hpp
#define clampMag_hpp

#include "Eigen/Dense"
#include "IKOptions.hpp"

using namespace Eigen;

void clampMag(Vector<double,6>& e, const IKOptions& opt);

#include "clampMag.cpp"
#endif /* clampMag_hpp */
