#pragma once

#include <string>
#include "util/MathUtil.h"
#include "scenarios/Scenario.h"
#include "stuff/PenaltyGround.h"
#include "stuff/Ball.h"
#include "learning/ExpTuple.h"
#include "learning/QNetTrainer.h"

class cScenarioBallRL : public cScenario
{
public:
	cScenarioBallRL();
	virtual ~cScenarioBallRL();

	virtual void Init();
	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Reset();
	virtual void Clear();

	virtual void Update(double time_elapsed);

	virtual const cBall& GetBall() const;
	virtual const cPenaltyGround& GetGround() const;

	virtual const tVector& GetBallPos() const;
	virtual void ToggleTraining();
	virtual bool EnableTraining() const;

	virtual double GetSuccRate() const;

	virtual void SaveNet(const std::string& out_file) const;

	virtual std::string GetName() const;
	
protected:
	cBall mBall;
	cPenaltyGround mGround;
	cPenaltyGround::tParams mGroundParams;

	cNeuralNetTrainer::tParams mTrainerParams;

	bool mFirstCycle;
	int mNumExpAnnealIters;

	cBallController::tExpParams mInitExpParams;
	cBallController::tExpParams mExpParams;
	double mCtrlNoise;
	bool mEnableTraining;

	int mNumTuples;
	std::vector<tExpTuple> mTupleBuffer;
	tExpTuple mCurrTuple;

	std::shared_ptr<cTrainerInterface> mTrainer;
	std::shared_ptr<cNeuralNetLearner> mLearner;

	virtual void SetupController();
	virtual void BuildController(std::shared_ptr<cBallController>& out_ctrl);
	virtual void UpdateGround();

	virtual int GetStateSize() const;
	virtual int GetActionSize() const;

	virtual void NewCycleUpdate();

	virtual void ClearFlags(tExpTuple& out_tuple) const;
	virtual void RecordBegFlags(tExpTuple& out_tuple) const;
	virtual void RecordEndFlags(tExpTuple& out_tuple) const;
	virtual void RecordState(Eigen::VectorXd& out_state) const;
	virtual void RecordAction(Eigen::VectorXd& out_action) const;
	virtual double GetActionLogp() const;
	virtual double CalcReward(const tExpTuple& tuple) const;
	virtual bool CheckFail() const;

	virtual void ApplyRandAction();

	virtual void InitTupleBuffer();
	virtual void InitTrainer();
	virtual void InitLearner();
	virtual void InitGround();

	virtual void SetupTrainerOutputOffsetScale();
	
	virtual int GetIter() const;
	virtual void Train();
	virtual void CalcExpParams(cBallController::tExpParams& out_params) const;
};