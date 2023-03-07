
# QuIK Codebase

This repository contains code to implement the QuIK algorithm, and others. It is provided as-is, with no guarantees of performance. The code in this repository is more full explained in the paper.

S. Lloyd, R. Irani, and M. Ahmadi, "Fast and Robust Inverse Kinematics for Serial Robots using Halley’s Method," IEEE Transactions on Robotics, vol. 38, no. 5, pp. 2768–2780, Oct. 2022. doi: [10.1109/TRO.2022.3162954](http://dx.doi.org/10.1109/TRO.2022.3162954). A preprint of this paper can be found in the current repository, [SLloydEtAl2022_QuIK_preprint.pdf](SLloydEtAl2022_QuIK_preprint.pdf).

The codebase features a C++ and Matlab implementation of the QuIK method. It also includes code to reproduce the benchmarks in the paper above.and

## Codebase file structure

 * **C++**: A folder containing all C++ code
	 * **KDL_IK**: The codebase containing the KDL libraries
		 * *IK_mexAdapter.cpp*: A C++ mex-adapter, allowing the KDL IK libraries to be called from Matlab.
		 * **KDL-1.5.0**: A copy of the KDL codebase source files.
	 * **QuIK**: The codebase for the C++ implementation of the QuIK algorithm
		 * *example.cpp*: An example function demonstrating the usage of the function.
		 * *IK_mexAdapter.cpp*: A C++ mex-adapter function, allowing the QuIK C++ libraries to be called from Matlab.
		 * **IK**: Header and C++ files for the inverse kinematics.
		 * **Robot**: Header and C++ files for Robot definition, forward kinematics, jacobian computation, etc..
 * **matlab**: A folder containing all matlab code. To run the matlab code, you should add this directory to your path.
	 * **+benchmark**: A matlab package containing functions that run the benchmarks from the paper.
	 * **+examples**: A directory with several examples showing how the packages can be used, and allowing benchmarks from the paper to be reproduced.
	 * **+IK_matlab**: A matlab package that allows the *Matlab robotics toolbox* functions to be compiled and run, using a mex adapter, in c-code.
	 * **+KDL_IK**: A matlab package that allows the KDL library (from the C++) folder to be compiled into a mex-file, and run from matlab.
	 * **+QuIK**: A matlab implementation of the QuIK algorithm. Provided for demonstration perposes - the matlab code is much slower than the C++ implementation.
	 * **+QuIK_cpp**: A matlab package that allows the C++ QuIK library from the C++ directory to be compiled into a mex-file, and run from Matlab.
	 * **+utils**: A matlab package with various helper functions
	 * *config.m*: A configuration file. It is necessary to edit this files before MEX-files can be generated, as the codebases all rely on an Eigen installation.

## Requirements
### C++ Requirements
All code is given as simple cpp and hpp files. Code is arranged as a header-only library, so simply include the header files for the code you'd like to use. All functions rely on the [Eigen 3.4](https://eigen.tuxfamily.org) linear algebra library, so you will also need to link to an appropriate library.

### Matlab Requirements
The matlab code given in the ``/matlab`` directory has only been tested on Matlab R2021a, and is not guaranteed to work on earlier versions. Additionally, benchmarking requires use of the Matlab Coder, Parallel Computing Toolbox and Robotics Toolbox add-ons.

## Minimal Usage

### C++ QuIK library

A minimal example is given in ``C++/QuIK/example.cpp``. This code defines a robot object from a DH table, generates 5 random poses for the manipulator, and then demonstrates inverse kinematics on these poses.

### Matlab Library

The matlab code provides functionality to perform inverse kinematics (as a matlab function) in the ``+QuIK`` package. This code is primarily given for educational purposes - the C++ package is much faster. This C++ package can be run from Matlab by using the ``+QuIK_cpp`` package.

All Matlab functions are well documented and well commented. To know the calling syntax, call ``help`` on the desired function. A minimal example is given in the examples directory. We believe these fully cover most use cases.

 * **ex01_sample_IK**: Demonstrates using the matlab inverse kinematics library to solve a sample inverse kinematics problem using a KUKA KR6 kinematic chain.
 * **ex02_sample_IK_compiled**: Demonstrates using a compiled mex-version of the C++ QuIK library, through a mex adapter function. The mex'ed version is typically 100-200x faster.
 * **ex03_benchmark1**: The example reproduces the first benchmark found in the paper "Fast and Robust Inverse Kinematics using Halley's method", by Steffan Lloyd, Rishad Irani, and Mojtaba Ahmadi. It performs a benchmarking comparison of the performance and reliability of the QuIK/NR/BFGS algorithms at different perturbation amounts.
 *  **ex04_benchmark2**: This example reproduces the second benchmark found in the paper above, performing a comparison of the singularity robustness of the algorithms.
 * **ex05_benchmark3**: This example reproduces the third benchmark found in the paper above. It performs a benchmarking comparison of the performance and reliability of the QuIK/NR/BFGS algorithms with other algorithms, such as KDL and Matlab.

### Robot Definitions

This codebase defines several robots, for benchmarking, in the ``+robots`` package. Each robot is coded as a matlab function that returns a structure of DH parameters, base transform, and tool transform.

## Citations

If you use our work, please reference our publication below. Recommended citation:

[1] [S. Lloyd, R. A. Irani, and M. Ahmadi, “Fast and Robust Inverse Kinematics of Serial Robots Using Halley’s Method,” IEEE Transactions on Robotics, vol. 38, no. 5, pp. 2768–2780, Oct. 2022.](SLloydEtAl2022_QuIK_preprint.pdf) doi: [10.1109/TRO.2022.3162954](http://dx.doi.org/10.1109/TRO.2022.3162954).
