#include "InverseKinematics.hpp"
#include <cmath>

// euler method for energy function, try to get direction for each step
Eigen::Matrix3d rotationMatrix(Eigen::Vector3d angle){
	/*
		Create rotation matrix, in form of ZXY rotation matrix
	*/
	Eigen::Matrix3d zRot, xRot, yRot;
	angle /= -(180.0/PI);
	zRot << cos(angle[2]), sin(angle[2]), 0, -sin(angle[2]), cos(angle[2]), 0, 0, 0, 1;
	xRot << 1, 0, 0, 0, cos(angle[0]), sin(angle[0]), 0, -sin(angle[0]), cos(angle[0]);
	yRot << cos(angle[1]), 0, -sin(angle[1]), 0, 1, 0, sin(angle[1]), 0, cos(angle[1]);

	/*
	for (int i = 0; i < zRot.size(); i++){
		std::cout << *(zRot.data() + i) << "  ";
		std::cout << std::endl << std::endl;
	}
	*/

	return zRot * xRot * yRot;
}

// function converting local rotation change to global rotation change

Eigen::MatrixXd pseudoInverse(Eigen::MatrixXd input){
	return input.completeOrthogonalDecomposition().pseudoInverse();
}

void getGlobalPosAndOri(const std::vector<Segment *> *joints,
	std::vector<Eigen::Vector3d> *jointOriX,
	std::vector<Eigen::Vector3d> *jointOriY,
	std::vector<Eigen::Vector3d> *jointOriZ,
	std::vector<Eigen::Vector3d> *jointPos){

	for(int i = joints->size()-2; i >= 0; i--){
		// apply rotation
		Eigen::Matrix3d rot = rotationMatrix((*joints)[i]->getRot());
		for(int jCnt = 0; jCnt < (*jointPos).size(); jCnt++){
			(*jointOriX)[jCnt].applyOnTheLeft(rot);
			(*jointOriY)[jCnt].applyOnTheLeft(rot);
			(*jointOriZ)[jCnt].applyOnTheLeft(rot);
			(*jointPos)[jCnt].applyOnTheLeft(rot);
		}

		// apply translation (only for pos)
		Eigen::Vector3d dpos = (*joints)[i]->getOffset() + (*joints)[i]->getTrans();
		for(int jCnt = 0; jCnt < (*jointPos).size(); jCnt++){
			(*jointPos)[jCnt] += dpos;
		}

		(*jointPos).push_back(dpos);
		(*jointOriX).push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
		(*jointOriY).push_back(Eigen::Vector3d(0.0, 1.0, 0.0));
		(*jointOriZ).push_back(Eigen::Vector3d(0.0, 0.0, 1.0));
	}
}

Eigen::MatrixXd computeJacobian(std::vector<Segment *> const& joints, Eigen::Vector3d pos, Eigen::Vector3d ori){
	/*
		Get global position(pos) and orientation(ori) of the end site,
		orientation of each joint axis(jointOris)
	*/
	std::vector<Eigen::Vector3d> jointOriX;
	std::vector<Eigen::Vector3d> jointOriY;
	std::vector<Eigen::Vector3d> jointOriZ;
	std::vector<Eigen::Vector3d> jointPos;

	getGlobalPosAndOri(&joints, &jointOriX, &jointOriY, &jointOriZ, &jointPos);
	/* 
	// for now, ignore cases where some joint has limited freedom
	int dof = 0;
	for (auto joint : joints){
		if (joint -> getName() != "End Site") dof += joint->numChannels();
	}
	*/
	Eigen::MatrixXd posFactor(3, 3 * (joints.size()-1));
	Eigen::MatrixXd oriFactor(3, 3 * (joints.size()-1));

	for(int i = jointPos.size()-2; i >= 0; i--){
		Eigen::Vector3d relPos = (pos - jointPos[i]);
		Eigen::Vector3d posFactorX = jointOriX[i].cross(relPos);
		Eigen::Vector3d posFactorY = jointOriY[i].cross(relPos);
		Eigen::Vector3d posFactorZ = jointOriZ[i].cross(relPos);

		posFactor.col(3 * (joints.size() - 2) - 3 * i + 0) = posFactorX;
		posFactor.col(3 * (joints.size() - 2) - 3 * i + 1) = posFactorY;
		posFactor.col(3 * (joints.size() - 2) - 3 * i + 2) = posFactorZ;
		oriFactor.col(3 * (joints.size() - 2) - 3 * i + 0) = jointOriX[i];
		oriFactor.col(3 * (joints.size() - 2) - 3 * i + 1) = jointOriY[i];
		oriFactor.col(3 * (joints.size() - 2) - 3 * i + 2) = jointOriZ[i];
	}

	Eigen::MatrixXd jacobian(6, 3 * joints.size() - 1);
	jacobian.block(0, 0, 3, 3 * (joints.size() - 1)) = posFactor;
	jacobian.block(3, 0, 3, 3 * (joints.size() - 1)) = oriFactor;
	
	return jacobian;
}

Eigen::VectorXd leastSquareDirection(std::vector<Segment *> const& joints, Eigen::Vector3d goalOri, Eigen::Vector3d goalPos){
	// calculate position and orientation of end site in global coordination first
	Eigen::Vector3d pos = joints[joints.size() - 1]->getOffset();
	Eigen::Vector3d ori = joints[joints.size() - 1]->getOffset().normalized();

	for(int i = joints.size()-2; i >= 0; i--){
		// apply rotation
		Eigen::Matrix3d rot = rotationMatrix(joints[i]->getRot());
		ori.applyOnTheLeft(rot);
		pos.applyOnTheLeft(rot);

		// apply translation (only for pos)
		Eigen::Vector3d dpos = joints[i]->getOffset() + joints[i]->getTrans();
		pos += dpos;
	}

	Eigen::MatrixXd jacobian = computeJacobian(joints, pos, ori);

	Eigen::VectorXd dgoal(6);
	dgoal.block(0, 0, 3, 1) = goalPos - pos;
	dgoal.block(3, 0, 3, 1) = ori.cross(goalOri);

    dgoal.applyOnTheLeft(pseudoInverse(jacobian));
	
	return dgoal;
}