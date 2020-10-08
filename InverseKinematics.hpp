#pragma once
#include <vector>
#include "bvh-loader/Segment.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/QR> 
#define PI 3.14159265

std::vector<std::vector<Segment *>> getEndSites(Segment * root);
Eigen::Matrix3d rotationMatrix(Eigen::Vector3d angle);
Eigen::MatrixXd pseudoInverse(Eigen::MatrixXd input);
void getGlobalPosAndOri(const std::vector<Segment *> *joints,
	std::vector<Eigen::Vector3d> *jointOriX,
	std::vector<Eigen::Vector3d> *jointOriY,
	std::vector<Eigen::Vector3d> *jointOriZ,
	std::vector<Eigen::Vector3d> *jointPos);
Eigen::MatrixXd computeJacobian(std::vector<Segment *> const& joints, Eigen::Vector3d pos, Eigen::Vector3d ori);
Eigen::VectorXd leastSquareDirection(std::vector<Segment *> const& joints, Eigen::Vector3d goalOri, Eigen::Vector3d goalPos);