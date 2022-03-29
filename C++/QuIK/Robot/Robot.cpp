/**
 *   Robot.cpp
 *   QuIK
 *
 *  	A simple class allowing for kinematic analysis of a serial chain.
 *
 * 	Properties:
 * 		- Matrix<double,DOF,4> DH: A DOFx4 array of the DH params in the following order:
 *          [a_1  alpha_1    d_1   theta_1;
 *           :       :        :       :        :         :
 *           an   alpha_n    d_n   theta_n ];
 *
 *       - Vector<bool,DOF> linkTypes: A vector of link types. Should be true if joint is a
 *        prismatic joint, false otherwise.
 *
 *       - Vector<double,6> Qsign: A vector of link direction (-1 or 1). Allows you to
 *        change the sign of the joint variable.
 *
 * 		- Matrix4d Tbase: The base transform of the robot (between world frame and first
 * 		 DH frame)
 *
 * 		- Matrix4d Ttool: The tool transform of the robot (between DOF'th frame and tool
 * 		 frame)
 *
 *  Methods:
 *
 * 		void FK( const MatrixQ& Q, MatrixTi& T) const;
 * 		Computes the forward kinematics of the manipulator for a given set of joint angles.
 * 		Result is stored in a Matrix<double,DOF*4,4> matrix, with each frame transform
 * 		stacked vertically ontop of each other.
 *
 * 		void FKn( const MatrixQ& Q, Matrix4d& T) const;
 * 		Computes the forward kinematics for a given set of joint angles, but only returns
 * 		the final frame as a Matrix<double,4,4>.
 *
 *    	void jacobian( MatrixTi& T, MatrixJ& J) const;
 * 		Computes the geometric jacobian of the manipulator, based on the stacked forward
 * 		kinematics transforms given by the FK function.
 *
 *    	void hessianProduct( const MatrixJ& J, const MatrixQ& dQ, MatrixJ& A) const
 * 		Computes the product H*dQ, where H is the geometric hessian of the robot. This
 * 		function never computes H explicitely, and instead just reduces the computed values
 * 	 	through multiplication as they are computed. The result is added to A.
 * 		Note, A must be pre-initialized with 0's before calling this function!
 *
 *   Created by Steffan Lloyd on 2021-08-20.
 *
*/

#include "Robot.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <math.h>

using namespace Eigen;
using namespace std;

/**
 * Robot constructor
 */
template<int DOF>
Robot<DOF>::Robot( 	Array<double,DOF,4> _DH,
					Vector<bool,DOF> _linkTypes,
					Vector<double,DOF> _Qsign,
					Matrix4d _Tbase,
					Matrix4d _Ttool){
	DH = _DH;
	Tbase = _Tbase;
	Ttool = _Ttool;
	linkTypes = _linkTypes;
	Qsign = _Qsign;
	dof = (int) DH.rows();
	
}

template<int DOF>
void Robot<DOF>::print() const{
	cout << "R.DH: " << endl << DH << endl;
	cout << "R.Tbase: " << endl << Tbase << endl;
	cout << "R.Ttool: " << endl << Ttool << endl;
	cout << "R.linkTypes: " << linkTypes.transpose() << endl;
	cout << "R.Qsign: " << Qsign.transpose() << endl;
	cout << "R.dof: " << dof << endl;
}

/**
 * FK Computes the forward kinematics of the manipulator for a given set of joint angles.
 * Result is stored in a Matrix<double,DOF*4,4> matrix, with each frame transform
 * stacked vertically ontop of each other.
 *
 * @param[in] Q The joint angles Q, in a DOF-vector
 * @param[out] T A matrix of the forward kinematics of all the frames of the robot, vertically stacked
 * into a 4*DOFx4 matrix.
 */
template<int DOF>
void Robot<DOF>::FK( const Vector<double,DOF>& Q, Matrix<double,(DOF>0?4*(DOF+1):-1),4>& T) const{
	
	// Initialize data
	Array<double,1,4> DH_k;
	Matrix4d Ak;
	double stk, ctk, sak, cak, ak, dk;
		
	// Iterate over joints
	for (int k=0; k<dof; k++){
		
		// Get DH variables for row
		DH_k = DH.row(k);
		if (linkTypes(k)) DH_k(2) += Q(k) * Qsign(k);
		else DH_k(3) += Q(k) * Qsign(k);
		
		// break out sin and cos
		stk = sin(DH_k(3));
		ctk = cos(DH_k(3));
		sak = sin(DH_k(1));
		cak = cos(DH_k(1));
		ak = DH_k(0);
		dk = DH_k(2);
		
		// Assign frame transform
		Ak << 	ctk, 	-stk*cak, 	stk*sak, 	ak*ctk,
				stk, 	ctk*cak, 	-ctk*sak, 	ak*stk,
				0,		sak,		cak,		dk,
				0, 		0,			0,			1;
		
		// Collect transforms
		if (k==0){
			T.template middleRows<4>(0) = Tbase * Ak;
		}else{
			T.template middleRows<4>(4*k) = T.template middleRows<4>(4*(k-1)) * Ak;
		}
			  
	}
	
	// Apply tool transform
	T.template bottomRows<4>() = ((T.template middleRows<4>(4*(dof-1))) * Ttool).eval();
	
}

/**
 * FKn Computes the forward kinematics for a given set of joint angles, but only returns
 * the final frame as a Matrix<double,4,4>.
 *
 * @param[in] Q The joint angles Q, in a DOF-vector
 * @param[out] Tn A matrix of the forward kinematics of the end-effector of the robot, as a Matrix4d
 */
template<int DOF>
void Robot<DOF>::FKn(const Vector<double,DOF> &Q, Matrix4d& Tn) const{
	constexpr int DOF4 = (DOF>0) ? (DOF+1)*4 : -1;
	Matrix<double,DOF4,4> Ti((dof+1)*4,4);
	FK( Q, Ti );
	Tn = Ti.template bottomRows<4>();
}

/**
 *	Computes the geometric jacobian of the manipulator, based on the stacked forward
 *	kinematics transforms given by the FK function. The result is stored in J.
 *
 * @param[in] T The forward kinematics of the robot in a given pose, in a 4*DOFx4 matrix
 * @param[out] J The Jacobian of the manipulator in a given pose
 */
template<int DOF>
void Robot<DOF>::jacobian( Matrix<double,(DOF>0?4*(DOF+1):-1),4>& T, Matrix<double,6,DOF>& J, bool includeTool) const{
	
	// Initialize some variables
	Vector3d z_im1, o_im1, o_n;
	
	// Get position of end effector
	o_n = T.template block<3,1>( 4*(dof-1+int(includeTool)), 3);
	// o_n = T.template block<3,1>( 4*(dof-1), 3);

	// Loop through joints
	for (int i = 0; i < dof; i++){
		
		// Get z_{i-1}, o_{i-1}
		// z_{i-1} is the unit vector along the z-axis of the previous joint
		// o_{i-1} is the position of the center of the previous joint frame
		if (i > 0){
			z_im1 = T.template block<3,1>(4*(i-1), 2);
			o_im1 = T.template block<3,1>(4*(i-1), 3);
		}else{
			z_im1 = Tbase.block<3,1>(0,2);
			o_im1 = Tbase.block<3,1>(0,3);
		}
		
		// Assign the appropriate blocks of the Jacobian
		// These formulas are from Spong.
		if (linkTypes(i)){
			// Prismatic joint
			J.template block<3,1>(0, i) = z_im1;
			J.template block<3,1>(3, i).fill(0);
		}else{
			// Revolute joint
			J.template block<3,1>(0, i) = z_im1.cross( o_n - o_im1 );
			J.template block<3,1>(3, i) = z_im1;
		}
	}
	
}


/**
 *  Computes the product H*dQ, where H is the geometric hessian of the robot. This
 *	function never computes H explicitely, and instead just reduces the computed values
 * 	through multiplication as they are computed. The result is added to A.
 *	Note, A must be pre-initialized with 0's before calling this function!
 *
 * @param[in] J The Jacobian of the manipulator in a given pose
 * @param[in] dQ The vector dQ
 * @param[out] A The matrix A to store the hessian product
 */
template <int DOF>
void Robot<DOF>::hessianProduct( const Matrix<double,6,DOF>& J, const Vector<double,DOF>& dQ, Matrix<double,6,DOF>& A) const{
	
	Vector3d cp, jvk, jwk, Aw_k_sum;
	// A.fill(0);
	
	if (!linkTypes.any()){
		// Special code for revolute joint robots only (most common, can
		// skip branched coding which is
		
		for (int k = 0; k < dof; k++){
			// First, iterate over off-diagonal terms
			jvk = J.template block<3,1>(0,k);
			jwk = J.template block<3,1>(3,k);
            
            // Rotational terms can be simplified by avoiding half the cross products
			// Initiate summer, then sum in loop before doing cross product.
            Aw_k_sum << 0,0,0;
            
			for (int i = 0; i < k; i++){
				// A(4:6, k) += jwi x jwk * dQi
				//           += (jwi * dQi) x jwk
				// Can sum the first term in cross product first before computing cross product.
                Aw_k_sum += J.template block<3,1>(3,i) * dQ(i);
                
				// A(1:3, k) += jwi x jvk*dQi
				cp = J.template block<3,1>(3, i).cross( jvk );
				A.template block<3,1>(0, k) += cp * dQ(i);
				A.template block<3,1>(0, i) += cp * dQ(k); // Symmetry
			}
            
            // Do final cross product for rotational term
            A.template block<3,1>(3, k) += Aw_k_sum.cross( jwk );
			
			// For diagonal entries, can skip the omega term since jwk x jwk = 0
			// A(4:6, k) += jwi x jwk * dQi = 0
			//
			// A(1:3, k) += jwi x jvk*dQi
			A.template block<3,1>(0, k) += jwk.cross( jvk ) * dQ(k);
		}
		
	}else{
		// General case
		
		for (int k = 0; k < dof; k++){
			// First, iterate over off-diagonal terms
			jvk = J.template block<3,1>(0,k);
			jwk = J.template block<3,1>(3,k);
			
			for (int i = 0; i < k; i++){
				if (! linkTypes(i)){ // link i is revolute
					if (! linkTypes(k)){	// link k is revolute
						// A(4:6, k) += jwi x jwk * dQi
						A.template block<3,1>(3, k) += (J.template block<3,1>(3, i)).cross( jwk ) * dQ(i);
					}
					
					// A(1:3, k) += jwi x jvk*dQi
					cp = J.template block<3,1>(3, i).cross( jvk );
					A.template block<3,1>(0, k) += cp * dQ(i);
					A.template block<3,1>(0, i) += cp * dQ(k); // Symmetry
				}
				
				if (! linkTypes(k)){
					// For diagonal entries, can skip the omega term since jwk x jwk = 0
					// A(4:6, k) += jwi x jwk * dQi = 0
					//
					// A(1:3, k) += jwi x jvk*dQi
					A.template block<3,1>(0, k) += jwk.cross( jvk ) * dQ(k);
				}
			} // end of inner for loop
		} // end of outer for loop
	} // end of branched if
}
