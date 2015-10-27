#pragma once

#include <string>
#include "util/MathUtil.h"
#include "scenarios/ScenarioSimChar.h"
#include "learning/ExpTuple.h"
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

	virtual void SetTargetPos(const tVector& target);
	virtual const tVector& GetTargetPos() const;

	virtual void DrawCharacter() const;
	virtual void DrawTarget() const;

	virtual const std::unique_ptr<cTextureDesc>& GetViewRT() const;

	virtual void SaveNet(const std::string& out_file) const;
	virtual std::string GetName() const;
	
protected:
	bool mEnableTraining;

	int mIter;
	std::string mSolverFile;
	std::string mNetFile;
	std::string mModelFile;

	std::vector<tExpTuple> mTupleBuffer;
	tExpTuple mCurrTuple;
	int mNumTuples;

	tVector mTargetPos;

	cCamera mRTCam;
	std::unique_ptr<cTextureDesc> mRenderTarget;
	
	virtual bool BuildController(std::shared_ptr<cCharController>& out_ctrl);
	virtual void BuildGround();

	virtual void CreateCharacter(std::shared_ptr<cSimCharacter>& out_char) const;
	virtual void InitCharacterPos(std::shared_ptr<cSimCharacter>& out_char) const;

	virtual void UpdateGround();
	virtual void ResetGround();
	virtual void SetCtrlTargetPos(const tVector& target);

	virtual int GetStateSize() const;
	virtual int GetActionSize() const;

	virtual void RecordState(Eigen::VectorXd& out_state) const;
	virtual void RecordAction(Eigen::VectorXd& out_action) const;
	virtual double CalcReward(const tExpTuple& tuple) const;
	virtual bool CheckFail() const;

	virtual void UpdateCharacter(double time_step);

	virtual void InitTupleBuffer();
	virtual void InitTrainer();

	virtual void Train();

	virtual void InitCam();
	virtual void UpdateViewBuffer();
	virtual void InitRenderResources();
	virtual bool NeedCtrlUpdate() const;
};