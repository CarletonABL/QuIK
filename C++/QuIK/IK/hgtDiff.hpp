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

#ifndef hgtDiff_hpp
#define hgtDiff_hpp

#include "Eigen/Dense"
#include "IKOptions.hpp"

using namespace Eigen;

void hgtDiff(const Matrix4d& T1, const Matrix4d& T2, Vector<double,6>& e, const IKOptions& opt);

#include "hgtDiff.cpp"

#endif /* hgtDiff_hpp */
