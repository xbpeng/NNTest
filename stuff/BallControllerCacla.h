#pragma once
#include "BallController.h"

class cBallControllerCacla : public cBallController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cBallControllerCacla(cBall& ball);
	virtual ~cBallControllerCacla();

	virtual int GetActionSize() const;
	virtual void ApplyRandAction();

	virtual void RecordAction(Eigen::VectorXd& out_action) const;
	virtual tAction BuildActionFromParams(const Eigen::VectorXd& action_params) const;
	
	virtual bool ValidCritic() const;
	virtual bool LoadCriticNet(const std::string& net_file);
	virtual void LoadCriticModel(const std::string& model_file);

	virtual const cNeuralNet& GetActor() const;
	virtual cNeuralNet& GetActor();
	virtual const cNeuralNet& GetCritic() const;
	virtual cNeuralNet& GetCritic();

	virtual void CopyActorNet(const cNeuralNet& net);
	virtual void CopyCriticNet(const cNeuralNet& net);

	virtual int GetNetOutputSize() const;
	virtual int GetActorInputSize() const;
	virtual int GetActorOutputSize() const;
	virtual int GetCriticInputSize() const;
	virtual int GetCriticOutputSize() const;

	virtual void BuildNNOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildActorOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildCriticOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildCriticInputOffsetScaleTypes(std::vector<cNeuralNet::eOffsetScaleType>& out_types) const;
	virtual void BuildActorInputOffsetScaleTypes(std::vector<cNeuralNet::eOffsetScaleType>& out_types) const;

	virtual void BuildActionExpCovar(Eigen::VectorXd& out_covar) const;

	virtual void SetExpNoise(double exp);

protected:
	cNeuralNet mCriticNet;
	double mExpNoiseStd;

	virtual void CalcActionNet(tAction& out_action);
	virtual void GetRandomAction(tAction& out_action);
	virtual void GetRandomActionCont(tAction& out_action);
	virtual void ApplyExpNoise(tAction& out_action);

	virtual void SampleActionDist(int num_samples, Eigen::MatrixXd& out_samples);
};