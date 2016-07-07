#pragma once

#include "ArmNNTrackController.h"
#include "sim/MusculotendonUnit.h"

class cArmNNTrackMuscularController : public cArmNNTrackController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cArmNNTrackMuscularController();
	virtual ~cArmNNTrackMuscularController();

	virtual void Init(cSimCharacter* character, const std::string& char_file);
	virtual void Clear();
	virtual void Update(double time_step);
	virtual void Reset();

	virtual int GetPoliStateSize() const;
	virtual int GetPoliActionSize() const;
	virtual void BuildNNInputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildNNOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	
	virtual int GetNumMTUs() const;
	virtual const cMusculotendonUnit& GetMTU(int id) const;

	virtual void HandlePoseReset();
	virtual void HandleVelReset();

protected:

	std::vector<cMusculotendonUnit> mMTUs;

	virtual void BuildMTUs(const std::string& char_file);
	virtual void UpdateMTUs(double time_step);
	virtual void ResetMTUs();
	virtual void ResetCEState();

	virtual void UpdatePoliState();
	virtual void ApplyPoliAction(double time_step, const tAction& action);
	virtual int GetMTUStateSize() const;
};