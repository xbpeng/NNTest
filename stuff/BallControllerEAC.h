#pragma once
#include "BallController.h"
#include "learning/EACTrainer.h"

class cBallControllerEAC: public cBallController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	static int GetMaxFragIdx(const Eigen::VectorXd& params);
	static double GetMaxFragVal(const Eigen::VectorXd& params);
	static void GetFrag(const Eigen::VectorXd& params, int a_idx, Eigen::VectorXd& out_action);
	static void SetFrag(const Eigen::VectorXd& frag, int a_idx, Eigen::VectorXd& out_params);
	
	cBallControllerEAC(cBall& ball);
	virtual ~cBallControllerEAC();

	virtual void Reset();

	virtual int GetActionSize() const;
	virtual void ApplyRandAction();

	virtual int GetNumActionFrags() const;
	virtual int GetActionFragSize() const;

	virtual void RecordAction(Eigen::VectorXd& out_action) const;
	virtual tAction BuildActionFromParams(const Eigen::VectorXd& action_params) const;

	virtual bool IsExploring() const;

protected:
	bool mExploring;

	virtual void UpdateAction();
	virtual tAction CalcActionNetCont();
	virtual tAction GetRandomActionFrag();
	virtual tAction GetRandomActionNoise();
	virtual void AddExpNoise(tAction& out_action);
};