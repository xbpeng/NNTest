#pragma once
#include "util/MathUtil.h"
#include "PenaltyGround.h"
#include "learning/NeuralNet.h"

class cBall;

class cBallController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	struct tAction
	{
		int mID;
		Eigen::VectorXd mParams;
		double mDist;
		double mLikelihood;

		tAction();
		tAction(int id, double dist, double likelihood);
	};

	struct tExpParams
	{
		double mRate;
		double mTemp;
		double mNoise;
		double mInternNoise;

		tExpParams();
	};

	static const double gMinDist;
	static const double gMaxDist;

	cBallController(cBall& ball);
	virtual ~cBallController();

	virtual void Update(double time_step);
	virtual void Reset();

	virtual bool IsNewCycle() const;
	virtual void SetGround(cPenaltyGround* ground);
	virtual void SetCtrlNoise(double noise);
	virtual const Eigen::VectorXd& GetGroundSamples() const;
	virtual tVector GetGroundSamplePos(int s) const;
	virtual int GetNumGroundSamples() const;

	virtual bool LoadNet(const std::string& net_file);
	virtual void LoadModel(const std::string& model_file);
	virtual int GetNetInputSize() const;
	virtual int GetNetOutputSize() const;

	virtual int GetStateSize() const;
	virtual int GetActionSize() const;
	virtual const tAction& GetAction(int a) const;
	virtual const tAction& GetCurrAction() const;
	virtual double GetActionLikelihood() const;
	virtual tAction BuildActionFromParams(const Eigen::VectorXd& action_params) const;

	virtual void RecordState(Eigen::VectorXd& out_state) const;
	virtual void RecordAction(Eigen::VectorXd& out_action) const;

	virtual bool IsOffPolicy() const;

	virtual void ApplyRandAction();

	virtual void CopyNet(const cNeuralNet& net);
	virtual void SaveNet(const std::string& out_file) const;

	virtual double GetDistTravelled() const;

	virtual void SetExpParams(const tExpParams& params);
	virtual const tExpParams& GetExpParams() const;
	virtual void EnableExp(bool enable);

	virtual void BuildNNInputOffsetScaleTypes(std::vector<cNeuralNet::eOffsetScaleType>& out_types) const;
	virtual void BuildNNOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildActionBounds(Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const;
	virtual cNeuralNet& GetNet();

	virtual void RecordActionDist(bool enable);
	virtual bool RecordActionDist() const;
	virtual const Eigen::MatrixXd& GetActionDistSamples() const;

	virtual const tVector& GetPosBeg() const;

protected:
	cBall& mBall;
	cPenaltyGround* mGround;
	double mCtrlNoise;

	bool mEnableExp;
	tExpParams mExpParams;

	Eigen::VectorXd mGroundSamples;
	Eigen::VectorXd mPoliState;

	cNeuralNet mNet;

	double mPhase;
	tVector mPosBeg;
	tVector mPosEnd;
	double mDistTravelled;

	tAction mCurrAction;
	bool mOffPolicy;

	bool mRecordActionDist;
	Eigen::MatrixXd mActionDistSamples;

	virtual void LoadNetIntern(const std::string& net_file);

	virtual void UpdateAction();
	virtual double CalcDeltaPhase() const;
	virtual tVector CalcBallPos() const;
	virtual void SampleGround(Eigen::VectorXd& out_samples) const;
	virtual bool HasGround() const;

	virtual bool HasNet() const;
	virtual bool ShouldExplore() const;
	virtual void ExploitPolicy(tAction& out_action);
	virtual void ExploreAction(tAction& out_action);

	virtual void DecideAction(tAction& out_action);
	virtual void CalcActionNet(tAction& out_action);
	virtual void GetRandomAction(tAction& out_action);
	virtual void GetRandomActionDiscrete(tAction& out_action) const;

	virtual void BuildPoliState(Eigen::VectorXd& state) const;
	virtual void ApplyAction(int a);
	virtual void ApplyAction(const tAction& action);

	virtual int GetNumActionDistSamples() const;
	virtual void UpdateDistTravelled();

	virtual void SampleActionDist(int num_samples, Eigen::MatrixXd& out_samples);
};