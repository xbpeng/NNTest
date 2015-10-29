#pragma once

#include <string>
#include "util/MathUtil.h"
#include "scenarios/ScenarioSimChar.h"
#include "stuff/ArmNNController.h"
#include "stuff/ArmQPController.h"
#include "learning/ExpTuple.h"
#include "learning/NeuralNetTrainer.h"
#include "render/TextureDesc.h"
#include "render/camera.h"


class cScenarioArmRL : public cScenarioSimChar
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

	virtual bool EnabledAutoTarget() const;
	virtual void EnableAutoTarget(bool enable);

	virtual void SetTargetPos(const tVector& target);
	virtual const tVector& GetTargetPos() const;

	virtual void DrawCharacter() const;
	virtual void DrawTarget() const;

	virtual const std::unique_ptr<cTextureDesc>& GetViewRT() const;
	virtual const std::shared_ptr<cSimCharacter>& GetCoach() const;

	virtual void SaveNet(const std::string& out_file) const;
	virtual std::string GetName() const;
	
protected:
	bool mEnableAutoTarget;
	bool mEnableTraining;
	bool mPretrain;

	double mTargetCounter;
	std::shared_ptr<cSimCharacter> mCoach;

	std::string mSolverFile;
	std::string mNetFile;
	std::string mModelFile;

	cNeuralNetTrainer mTrainer;
	std::vector<tExpTuple> mTupleBuffer;
	int mNumTuples;

	tVector mTargetPos;

	cCamera mRTCam;
	std::unique_ptr<cTextureDesc> mRenderTarget;
	
	virtual void BuildWorld();
	virtual bool BuildController(std::shared_ptr<cCharController>& out_ctrl);
	virtual bool BuildCoachController(std::shared_ptr<cCharController>& out_ctrl);
	virtual void BuildGround();

	virtual void CreateCharacter(std::shared_ptr<cSimCharacter>& out_char) const;
	virtual void InitCharacterPos(std::shared_ptr<cSimCharacter>& out_char) const;
	virtual void BuildCoach();

	virtual void UpdateCoach(double time_step);
	virtual void UpdateGround();
	virtual void ResetGround();
	virtual void SetCtrlTargetPos(const tVector& target);

	virtual bool HasExploded() const;
	virtual void RandReset();
	virtual void ApplyRandPose();
	virtual void SetRandTarget();
	virtual void ResetTargetCounter();
	virtual void UpdateTargetCounter(double time_step);

	virtual int GetStateSize() const;
	virtual int GetActionSize() const;

	virtual void RecordState(Eigen::VectorXd& out_state) const;
	virtual void RecordAction(Eigen::VectorXd& out_action) const;
	virtual void RecordTuple();

	virtual void UpdateCharacter(double time_step);

	virtual void InitTupleBuffer();
	virtual void InitTrainer();
	virtual void SetupScale();

	virtual void Train();
	virtual int GetIter() const;

	virtual void InitCam();
	virtual void UpdateViewBuffer();
	virtual void InitRenderResources();
	virtual bool NeedCtrlUpdate() const;

	virtual std::shared_ptr<cArmQPController> GetCoachController() const;
	virtual std::shared_ptr<cArmNNController> GetStudentController() const;
	virtual void SyncCoach();
};