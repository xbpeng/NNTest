#pragma once

#include "scenarios/ScenarioArmRL.h"

class cScenarioArmRLPretrain : public cScenarioArmRL
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioArmRLPretrain();
	virtual ~cScenarioArmRLPretrain();
	
	virtual void Init();
	virtual void Reset();

	virtual void Update(double time_elapsed);

	virtual const tVector& GetPredTarget() const;
	virtual std::string GetName() const;
	
protected:
	tVector mPredTarget;

	virtual bool BuildController(std::shared_ptr<cCharController>& out_ctrl);

	virtual void SetupScale();
	virtual bool NeedCtrlUpdate() const;

	virtual void RecordState(Eigen::VectorXd& out_state) const;
	virtual void RecordAction(Eigen::VectorXd& out_action) const;

	virtual bool NeedViewBuffer() const;
	virtual void SetNNViewFeatures();
	virtual void Train();
	virtual void UpdatePredTarget();

	virtual void SyncCharacters();

	virtual void PrintInfo() const;
	virtual void GetRandTargetMinMaxTime(double& out_min, double& out_max) const;
	virtual void GetRandPoseMinMaxTime(double& out_min, double& out_max) const;
};