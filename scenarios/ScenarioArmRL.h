#pragma once

#include <string>
#include "util/MathUtil.h"
#include "scenarios/ScenarioArm.h"
#include "stuff/ArmNNController.h"
#include "stuff/ArmQPController.h"
#include "stuff/ArmPDQPController.h"
#include "stuff/ArmPDNNController.h"
#include "stuff/ArmNNPixelController.h"
#include "learning/ExpTuple.h"
#include "learning/NeuralNetTrainer.h"
#include "render/TextureDesc.h"
#include "render/camera.h"


class cScenarioArmRL : public cScenarioArm
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioArmRL();
	virtual ~cScenarioArmRL();

	virtual void Init();
	virtual void ParseArgs(const cArgParser& parser);
	virtual void Reset();
	virtual void Clear();

	virtual void Update(double time_elapsed);

	virtual void ToggleTraining();
	virtual bool EnableTraining() const;

	virtual const std::shared_ptr<cSimCharacter>& GetCoach() const;

	virtual void SaveNet(const std::string& out_file) const;
	virtual std::string GetName() const;

protected:
	bool mEnableTraining;
	bool mPretrain;

	eCtrlType mCoachType;
	std::shared_ptr<cSimCharacter> mCoach;

	std::string mSolverFile;

	cNeuralNetTrainer mTrainer;
	std::shared_ptr<cNeuralNetLearner> mLearner;

	cNeuralNetTrainer::tParams mTrainerParams;
	std::vector<tExpTuple> mTupleBuffer;
	int mNumTuples;

	virtual bool BuildCoachController(std::shared_ptr<cCharController>& out_ctrl);
	virtual void BuildCoach();

	virtual void UpdateCoach(double time_step);
	virtual void SetCtrlTargetPos(const tVector& target);

	virtual void ApplyRandPose();

	virtual int GetStateSize() const;
	virtual int GetActionSize() const;

	virtual void RecordState(Eigen::VectorXd& out_state) const;
	virtual void RecordAction(Eigen::VectorXd& out_action) const;
	virtual void RecordTuple();

	virtual void UpdateCharacter(double time_step);

	virtual void InitTupleBuffer();
	virtual void InitTrainer();
	virtual void InitLearner();
	virtual void SetupScale();

	virtual void Train();
	virtual int GetIter() const;

	virtual std::shared_ptr<cArmQPController> GetCoachController() const;
	virtual std::shared_ptr<cArmNNController> GetStudentController() const;
	virtual void SyncCharacters();
	virtual bool EnableSyncCharacters() const;

	virtual void PrintInfo() const;
};