#pragma once
#include "BallControllerACE.h"
#include "learning/MACETrainer.h"

class cBallControllerMACE: public cBallControllerACE
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cBallControllerMACE(cBall& ball);
	virtual ~cBallControllerMACE();

	virtual bool LoadCriticNet(const std::string& net_file);
	virtual void LoadCriticModel(const std::string& model_file);
	virtual void LoadCriticScale(const std::string& scale_file);
	virtual void CopyCriticNet(const cNeuralNet& net);
	virtual void SaveCriticNet(const std::string& out_file) const;

	virtual bool HasNet() const;
	virtual int GetNetOutputSize() const;
	virtual int GetCriticNetOutputSize() const;

protected:
	cNeuralNet mCriticNet;

	virtual void UpdateFragParams();

	virtual tAction CalcActionNetCont();
	virtual tAction GetRandomActionFrag();

	virtual int GetMaxFragIdx(const Eigen::VectorXd& params) const;
	virtual double GetMaxFragVal(const Eigen::VectorXd& params) const;
	virtual void GetFrag(const Eigen::VectorXd& params, int a_idx, Eigen::VectorXd& out_action) const;
	virtual void SetFrag(const Eigen::VectorXd& frag, int a_idx, Eigen::VectorXd& out_params) const;
};