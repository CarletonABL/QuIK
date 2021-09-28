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
	Vector3d d1, d2, re, w, l;
	double l_norm;
	
	// Break out values
	R1 = T1.topLeftCorner<3,3>();
	R2 = T2.topLeftCorner<3,3>();
	d1 = T1.topRightCorner<3,1>();
	d2 = T2.topRightCorner<3,1>();
	
	// Orientation error
	Re = R1*R2.transpose();
	
	// Assign linear error
	e.head<3>() = d1 - d2;
	
	// Extract diagonal
	re = Re.diagonal();

	if (Re.isDiagonal( std::numeric_limits<double>::epsilon() )){
		// Need to check if this is the identity or not
		if ((re.array() > 0).all()){
			// matrix is identity
			w = Vector3d::Zero(3,1);
		}else{
			w = 1.570796326794897 * (re.array() + 1);
		}
	}else{
		// Build l variable, and calculate norm
		l <<	Re(2,1)-Re(1,2),
				Re(0,2)-Re(2,0),
				Re(1,0)-Re(0,1);
		l_norm = l.norm();
		
		// If the l_norm is small, then the first-order taylor
		// expansion results in no error at all
		if (l_norm < 1e-12){
			w = 1/(re.sum() - 1) * l;
		}else{
			w = atan2(l_norm, re.sum() - 1) / l_norm * l;
		}
	}

	// Store in last 3 positions of e
	e.tail<3>() = w;

	// Set "ignored" error values to zero
	// for (int i = 0; i<opt.not_m; i++) e( opt.nErrorMask(i) ) = 0;
}
