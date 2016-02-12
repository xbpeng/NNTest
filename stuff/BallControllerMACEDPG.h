#pragma once
#include "BallControllerDPG.h"

class cBallControllerMACEDPG: public cBallControllerDPG
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cBallControllerMACEDPG(cBall& ball);
	virtual ~cBallControllerMACEDPG();

	virtual int GetNumActionFrags() const;
	virtual int GetActionFragSize() const;
	virtual int GetNetOutputSize() const;

	virtual bool ValidCritic() const;
	virtual bool LoadCriticNet(const std::string& net_file);
	virtual void LoadCriticModel(const std::string& model_file);

	virtual const cNeuralNet& GetActor() const;
	virtual cNeuralNet& GetActor();
	virtual const cNeuralNet& GetCritic() const;
	virtual cNeuralNet& GetCritic();

	virtual void CopyActorNet(const cNeuralNet& net);
	virtual void CopyCriticNet(const cNeuralNet& net);
	
	virtual int GetActorInputSize() const;
	virtual int GetActorOutputSize() const;
	virtual int GetCriticInputSize() const;
	virtual int GetCriticOutputSize() const;

	virtual void BuildNNOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildActorOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildCriticOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	
protected:
	cNeuralNet mCriticNet;
	int mNumActionFrags;
	Eigen::VectorXd mBoltzmannBuffer;

	virtual void LoadNetIntern(const std::string& net_file);
	virtual void UpdateFragParams();

	virtual void CalcActionNet(tAction& out_action);
	virtual void CalcCriticVals(const Eigen::VectorXd& state, const Eigen::VectorXd& actions, Eigen::VectorXd& out_vals);
	virtual void BuildActorAction(const Eigen::VectorXd& actions, int a_id, tAction& out_action) const;
};