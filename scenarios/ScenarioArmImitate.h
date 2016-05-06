#pragma once
#include "ScenarioArmTrain.h"
#include "anim/KinCharacter.h"

class cScenarioArmImitate : public cScenarioArmTrain
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioArmImitate();
	virtual ~cScenarioArmImitate();

	virtual void ParseArgs(const cArgParser& parser);
	virtual void Init();
	virtual void Reset();
	virtual void Clear();

	virtual const std::shared_ptr<cKinCharacter>& GetKinChar() const;
	virtual void ToggleTraining();

	virtual std::string GetName() const;

protected:
	std::string mMotionFile;
	std::shared_ptr<cKinCharacter> mKinChar;

	virtual void BuildKinCharacter();
	virtual void InitTrainer();

	virtual void UpdateCharacter(double time_step);
	virtual void RandReset();
	virtual void ResetKinChar();
	virtual void RandResetKinChar();
	virtual void ApplyRandPose();
	virtual void SyncCharacter();

	virtual double CalcReward() const;
	virtual void GetRandPoseMinMaxTime(double& out_min, double& out_max) const;
};