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
#include "Eigen/Dense"
#include "clampMag.hpp"
#include <math.h>

using namespace Eigen;

void clampMag(Vector<double,6>& e, const IKOptions& opt){
	
	// Break out early if we're doing BFGS, this isn't relevant
	if (opt.algorithm == 2) return;
	
	// Calculate the squared normed error
	// This avoids doing the sqrt unless necessary
	double ei_lin_norm2 = e.head<3>().array().square().sum();
	double ei_ang_norm2 = e.tail<3>().array().square().sum();

	// If either limit is greater than the square of the threshold, then rescale
	// the appropriate part of the error
	if (ei_lin_norm2 > (opt.maxLinearErrorStep * opt.maxLinearErrorStep))
		e.head<3>() *= opt.maxLinearErrorStep / sqrt(ei_lin_norm2);
	
	if (ei_ang_norm2 > (opt.maxAngularErrorStep * opt.maxAngularErrorStep))
		e.tail<3>() *= opt.maxAngularErrorStep / sqrt(ei_ang_norm2);
}
