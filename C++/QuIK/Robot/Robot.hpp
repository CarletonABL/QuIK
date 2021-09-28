//
//  Robot.cpp
//  QuIK
//
// 	A simple class allowing for kinematic analysis of a serial chain.
//
//	Properties:
//		- Matrix<double,DOF,4> DH: A DOFx4 array of the DH params in the following order:
//         [a_1  alpha_1    d_1   theta_1;
//          :       :        :       :        :         :
//          an   alpha_n    d_n   theta_n ];
//
//      - Vector<bool,DOF> linkTypes: A vector of link types. Should be true if joint is a
//       prismatic joint, false otherwise.
//
//      - Vector<double,6> Qsign: A vector of link direction (-1 or 1). Allows you to
//       change the sign of the joint variable.
//
//		- Matrix4d Tbase: The base transform of the robot (between world frame and first
//		 DH frame)
//
//		- Matrix4d Ttool: The tool transform of the robot (between DOF'th frame and tool
//		 frame)
//
// Methods:
//
//		void FK( const MatrixQ& Q, MatrixTi& T) const;
//		Computes the forward kinematics of the manipulator for a given set of joint angles.
//		Result is stored in a Matrix<double,DOF*4,4> matrix, with each frame transform
//		stacked vertically ontop of each other.
//
//		void FKn( const MatrixQ& Q, Matrix4d& T) const;
//		Computes the forward kinematics for a given set of joint angles, but only returns
//		the final frame as a Matrix<double,4,4>.
//
//   	void jacobian( MatrixTi& T, MatrixJ& J) const;
//		Computes the geometric jacobian of the manipulator, based on the stacked forward
//		kinematics transforms given by the FK function.
//
//   	void hessianProduct( const MatrixJ& J, const MatrixQ& dQ, MatrixJ& A) const
//		Computes the product H*dQ, where H is the geometric hessian of the robot. This
//		function never computes H explicitely, and instead just reduces the computed values
//	 	through multiplication as they are computed. The result is added to A.
//		Note, A must be pre-initialized with 0's before calling this function!
//
//  Created by Steffan Lloyd on 2021-08-20.
//

#ifndef robot_hpp
#define robot_hpp

#include "Eigen/Dense"

using namespace Eigen;

template<int DOF=Dynamic>
class Robot {
public:
    
	// DH parameters, as a DOF x 4 array in the following order:
	// a_i, alpha_i, d_i, theta_i
    Array<double,DOF,4> DH;
	
	// Base transform
	Matrix4d Tbase;
	
	// Ttool transform
	Matrix4d Ttool;
    
	Vector<bool,DOF> linkTypes;
    
	Vector<double,DOF> Qsign;
	
	int dof;
	
	Robot( Array<double,DOF,4> _DH,
		  Vector<bool,DOF> _linkTypes,
		  Vector<double,DOF> _Qsign,
		  Matrix4d _Tbase = Matrix4d::Identity(4,4),
		  Matrix4d _Ttool = Matrix4d::Identity(4,4));
	
	void print() const;
    
    void FK( const Vector<double,DOF>& Q, Matrix<double,(DOF>0?4*(DOF+1):-1),4>& T) const;
	
	void FKn( const Vector<double,DOF>& Q, Matrix4d& T) const;
	
	void jacobian( Matrix<double,(DOF>0?4*(DOF+1):-1),4>& T, Matrix<double,6,DOF>& J, bool includeTool = true) const;
	
	void hessianProduct( const Matrix<double,6,DOF>& J, const Vector<double,DOF>& dQ, Matrix<double,6,DOF>& A) const;
	
};

#include "Robot.cpp"

#endif /* robot_hpp */
