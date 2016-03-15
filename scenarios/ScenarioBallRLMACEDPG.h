#pragma once

#include "scenarios/ScenarioBallRLDPG.h"

class cScenarioBallRLMACEDPG : public cScenarioBallRLDPG
{
public:
	cScenarioBallRLMACEDPG();
	virtual ~cScenarioBallRLMACEDPG();

	virtual void ParseArgs(const cArgParser& parser);
	virtual std::string GetName() const;
	
protected:
	virtual void InitTrainer();
	virtual void SetupController();
	virtual void BuildController(std::shared_ptr<cBallController>& out_ctrl);

	virtual void BuildCriticOutputOffsetScale(const std::shared_ptr<cNeuralNetTrainer>& trainer,
											Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildActorOutputOffsetScale(const std::shared_ptr<cNeuralNetTrainer>& trainer,
											Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;

	virtual void Train();
};