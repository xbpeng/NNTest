#pragma once
#include "BallControllerDPG.h"

class cBallControllerMACEDPG: public cBallControllerDPG
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cBallControllerMACEDPG(cBall& ball);
	virtual ~cBallControllerMACEDPG();

	virtual void Reset();

	virtual int GetNumActionFrags() const;
	virtual int GetActionFragSize() const;
	virtual int GetNetOutputSize() const;

	virtual int GetActorOutputSize() const;
	virtual int GetCriticInputSize() const;

	virtual void BuildActorOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	
protected:
	int mNumActionFrags;
	Eigen::VectorXd mBoltzmannBuffer;
	bool mExpCritic;
	bool mExpActor;

	virtual void LoadNetIntern(const std::string& net_file);
	virtual void UpdateFragParams();

	virtual void CalcCriticVals(const Eigen::VectorXd& state, const Eigen::VectorXd& actions, Eigen::VectorXd& out_vals);
	virtual void BuildActorAction(const Eigen::VectorXd& actions, int a_id, tAction& out_action) const;

	virtual void UpdateAction();
	virtual void DecideAction(tAction& out_action);
	virtual void DecideActionBoltzmann(tAction& out_action);
};