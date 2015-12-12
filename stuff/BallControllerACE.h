#pragma once
#include "BallController.h"
#include "learning/ACETrainer.h"

class cBallControllerACE: public cBallController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cBallControllerACE(cBall& ball);
	virtual ~cBallControllerACE();

	virtual void Reset();
	virtual bool LoadNet(const std::string& net_file);

	virtual int GetActionSize() const;
	virtual void ApplyRandAction();

	virtual int GetNumActionFrags() const;
	virtual int GetActionFragSize() const;

	virtual void RecordAction(Eigen::VectorXd& out_action) const;
	virtual tAction BuildActionFromParams(const Eigen::VectorXd& action_params) const;

	virtual bool IsExpCritic() const;
	virtual bool IsExpActor() const;

protected:
	int mNumActionFrags;

	bool mExpCritic;
	bool mExpActor;

	virtual void UpdateAction();
	virtual tAction CalcActionNetCont();
	virtual tAction GetRandomActionFrag();
	virtual void AddExpNoise(tAction& out_action);

	virtual void UpdateFragParams();

	virtual int GetMaxFragIdx(const Eigen::VectorXd& params) const;
	virtual double GetMaxFragVal(const Eigen::VectorXd& params) const;
	virtual void GetFrag(const Eigen::VectorXd& params, int a_idx, Eigen::VectorXd& out_action) const;
	virtual void SetFrag(const Eigen::VectorXd& frag, int a_idx, Eigen::VectorXd& out_params) const;
	virtual void SetVal(double val, int a_idx, Eigen::VectorXd& out_params) const;

	virtual int GetTupleMaxFragIdx(const Eigen::VectorXd& params) const;
	virtual void GetTupleFrag(const Eigen::VectorXd& params, int a_idx, Eigen::VectorXd& out_action) const;
	virtual void SetTupleFrag(const Eigen::VectorXd& frag, int a_idx, Eigen::VectorXd& out_params) const;
	virtual void SetTupleVal(double val, int a_idx, Eigen::VectorXd& out_params) const;
};