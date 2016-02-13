#pragma once
#include "BallController.h"
#include "learning/MACETrainer.h"

class cBallControllerMACE: public cBallController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cBallControllerMACE(cBall& ball);
	virtual ~cBallControllerMACE();

	virtual void Reset();

	virtual int GetActionSize() const;

	virtual int GetNumActionFrags() const;
	virtual int GetActionFragSize() const;
	virtual int GetNetOutputSize() const;

	virtual void RecordAction(Eigen::VectorXd& out_action) const;
	virtual tAction BuildActionFromParams(const Eigen::VectorXd& action_params) const;

	virtual bool IsExpCritic() const;
	virtual bool IsExpActor() const;

	virtual void BuildNNOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;

protected:
	int mNumActionFrags;
	std::vector<double> mBoltzmannBuffer;
	bool mExpCritic;
	bool mExpActor;

	virtual void LoadNetIntern(const std::string& net_file);

	virtual void CalcActionNetCont(tAction& out_action);
	virtual void GetRandomActionFrag(tAction& out_action);
	virtual void ApplyExpNoise(tAction& out_action);
	virtual void AddExpActionNoise(tAction& out_action);
	virtual void AddExpStateNoise(tAction& out_action);

	virtual void UpdateAction();
	virtual void DecideAction(tAction& out_action);
	virtual void DecideActionBoltzmann(tAction& out_action);
	virtual void ExploitPolicy(tAction& out_action);
	virtual void ExploreAction(tAction& out_action);

	virtual void UpdateFragParams();
	virtual void BuildActorAction(const Eigen::VectorXd& params, int a_id, tAction& out_action) const;
	virtual void DebugPrintAction(const tAction& action, const Eigen::VectorXd& params) const;

	virtual int GetMaxFragIdx(const Eigen::VectorXd& params) const;
	virtual double GetMaxFragVal(const Eigen::VectorXd& params) const;
	virtual void GetFrag(const Eigen::VectorXd& params, int a_idx, Eigen::VectorXd& out_action) const;
	virtual void SetFrag(const Eigen::VectorXd& frag, int a_idx, Eigen::VectorXd& out_params) const;
	virtual double GetVal(const Eigen::VectorXd& params, int a_idx) const;
	virtual void SetVal(double val, int a_idx, Eigen::VectorXd& out_params) const;
};