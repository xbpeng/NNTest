#pragma once

#include "ScenarioArmEval.h"
#include "anim/KinCharacter.h"

class cScenarioArmImitateEval : public cScenarioArmEval
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioArmImitateEval();
	virtual ~cScenarioArmImitateEval();

	virtual void ParseArgs(const cArgParser& parser);
	virtual void Init();
	virtual void Reset();
	virtual void Clear();

	virtual const std::shared_ptr<cKinCharacter>& GetKinChar() const;

	virtual std::string GetName() const;

protected:
	std::string mMotionFile;
	std::shared_ptr<cKinCharacter> mKinChar;

	virtual void BuildKinCharacter();
	virtual void SyncCharacter();
	virtual void ResetKinChar();

	virtual void UpdateCharacter(double time_step);
	virtual void UpdateArmTrackController();
	virtual double CalcError() const;

	virtual void RandReset();
	virtual void ApplyRandPose();
	virtual void RandResetKinChar();
	virtual void GetRandPoseMinMaxTime(double& out_min, double& out_max) const;
};