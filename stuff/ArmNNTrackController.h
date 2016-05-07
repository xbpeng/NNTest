#pragma once

#include "ArmNNController.h"

class cArmNNTrackController : public cArmNNController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cArmNNTrackController();
	virtual ~cArmNNTrackController();

	virtual void Init(cSimCharacter* character);
	virtual void Reset();

	virtual int GetPoliStateSize() const;
	virtual void BuildNNInputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;

	virtual void SetTargetPoseVel(const Eigen::VectorXd& tar_pose, const Eigen::VectorXd& tar_vel);

protected:
	Eigen::VectorXd mTargetPose;
	Eigen::VectorXd mTargetVel;

	virtual void InitTargetPoseVel();
	virtual void UpdatePoliState();
};