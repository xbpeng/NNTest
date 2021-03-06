#pragma once

#include <string>
#include "util/MathUtil.h"
#include "scenarios/ScenarioArm.h"
#include "stuff/ArmNNController.h"
#include "stuff/ArmNNPixelController.h"
#include "learning/ExpTuple.h"
#include "learning/CaclaTrainer.h"
#include "render/TextureDesc.h"
#include "render/camera.h"


class cScenarioArmTrain : public cScenarioArm
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioArmTrain();
	virtual ~cScenarioArmTrain();

	virtual void Init();
	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Reset();
	virtual void Clear();

	virtual void Update(double time_elapsed);

	virtual void ToggleTraining();
	virtual bool EnableTraining() const;

	virtual void SaveNet(const std::string& out_file) const;
	virtual std::string GetName() const;

protected:
	bool mEnableTraining;
	bool mPretrain;
	bool mValidSample;

	double mExpRate;
	double mInitExpRate;
	double mExpTemp;
	double mInitExpTemp;
	int mNumAnnealIters;

	std::shared_ptr<cNeuralNetTrainer> mTrainer;
	std::shared_ptr<cNeuralNetLearner> mLearner;

	cNeuralNetTrainer::tParams mTrainerParams;
	std::vector<tExpTuple> mTupleBuffer;
	tExpTuple mCurrTuple;
	int mNumTuples;

	virtual void SetCtrlTargetPos(const tVector& target);

	virtual void ApplyRandPose();
	virtual void RandReset();
	virtual void SetRandTarget();

	virtual int GetStateSize() const;
	virtual int GetActionSize() const;

	virtual void RecordTuple(const tExpTuple& tuple);
	virtual void RecordState(Eigen::VectorXd& out_state) const;
	virtual void RecordAction(Eigen::VectorXd& out_action) const;
	virtual double CalcReward() const;
	virtual bool CheckFail() const;

	virtual void ClearFlags(tExpTuple& out_tuple) const;
	virtual void RecordFlagsBeg(tExpTuple& out_tuple) const;
	virtual void RecordFlagsEnd(tExpTuple& out_tuple) const;

	virtual void UpdateCharacter(double time_step);
	virtual void GetRandTargetMinMaxTime(double& out_min, double& out_max) const;
	virtual void GetRandPoseMinMaxTime(double& out_min, double& out_max) const;

	virtual void InitTupleBuffer();
	virtual void BuildTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer) const;
	virtual void InitTrainer();
	virtual void InitLearner();

	virtual void SetupActionBounds();
	virtual void BuildActionBounds(Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const;

	virtual void SetupScale();
	virtual void SetupActorScale();
	virtual void SetupCriticScale();

	virtual void UpdatePolicy();
	
	virtual void Train();
	virtual int GetIter() const;
	virtual double CalcExpRate() const;
	virtual double CalcExpTemp() const;
	virtual bool CheckOffPolicy() const;

	virtual std::shared_ptr<cArmNNController> GetController() const;
	virtual void PrintInfo() const;
};