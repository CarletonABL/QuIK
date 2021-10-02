//
//  hgtDiff.cpp
//  QuIK
//
//  Calculates the error between two homogeneous transforms.
//  Algorithm used is as described in
//  [1] T. Sugihara, “Solvability-Unconcerned Inverse Kinematics
//  by the Levenberg–Marquardt Method,” IEEE Trans. Robot.,
//	vol. 27, no. 5, pp. 984–991, Oct. 2011.
//
//  Created by Steffan Lloyd on 2021-08-21.
//

#include "hgtDiff.hpp"
#include "Eigen/Dense"
#include "IKOptions.hpp"
#include <math.h>

using namespace Eigen;
using namespace std;

void hgtDiff(const Matrix4d& T1, const Matrix4d& T2, Vector<double,6>& e, const IKOptions& opt){
	Matrix3d R1, R2, Re;
	Vector3d d1, d2, eps;
	double eps_norm, t;
	
	// Break out values
	R1 = T1.topLeftCorner<3,3>();
	R2 = T2.topLeftCorner<3,3>();
	d1 = T1.topRightCorner<3,1>();
	d2 = T2.topRightCorner<3,1>();
	
	// Orientation error
	Re = R1*R2.transpose();
	
	// Assign linear error
	e.head<3>() = d1 - d2;
	
	// Extract diagonal and trace
	t = Re.trace();
	
	// Build l variable, and calculate norm
	eps <<	Re(2,1)-Re(1,2),
			Re(0,2)-Re(2,0),
			Re(1,0)-Re(0,1);
	eps_norm = eps.norm();

	// Different behaviour if rotations are near pi or not.
	if (t > -.99 || eps_norm > 1e-10){
		// Matrix is normal or near zero (not near pi)
		// If the eps_norm is small, then the first-order taylor
		// expansion results in no error at all
		if (eps_norm < 1e-3){
			// atan2( eps_norm, t - 1 ) / eps_norm ~= 0.5 - (t-3)/12
			// Should have zero machine precision error when eps_norm < 1e-3.
			//
			// w ~= theta/(2*sin(theta)) = acos((t-1)/2)/(2*sin(theta))
			//
			// taylor expansion of theta/(2*theta) ~= 1/2 + theta^2/12 (3rd
			// order)
			// taylor expansion of (acos(t-1)/2)^2 is (3-t) (2nd order).
			//
			// Subtituting:
			// w ~= (1/2 + (3-t)/12) * eps = (0.75 - t/12)*eps.
			e.tail<3>() = (0.75 - t/12) * eps;
		}else{
			// Just use normal formula
			e.tail<3>() = (atan2(eps_norm, t - 1) / eps_norm) * eps;
		}
	}else{
		// If we get here, the trace is either nearly -1, and the error is
		// close to zero.
		// This combination is only possible if R is nearly a rotation of pi
		// radians about the x, y, or z axes.
		//
		// Since at this point, any rotation vector will do since we could
		// rotate in any direction. However, we use the approximation below.
		e.tail<3>() = 1.570796326794897 * (Re.diagonal().array() + 1);
		
	}
	
}
