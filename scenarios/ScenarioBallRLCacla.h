#pragma once

#include "scenarios/ScenarioBallRL.h"
#include "stuff/BallControllerCacla.h"
#include "learning/CaclaTrainer.h"

class cScenarioBallRLCacla : public cScenarioBallRL
{
public:
	cScenarioBallRLCacla();
	virtual ~cScenarioBallRLCacla();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);

	virtual void InitLearner();

	virtual void SaveCriticNet(const std::string& filename) const;
	virtual void SaveActorNet(const std::string& filename) const;

	virtual std::string GetName() const;
	
protected:
	
	virtual void BuildController(std::shared_ptr<cBallController>& out_ctrl);
	virtual void InitTrainer();
	virtual void BuildCriticOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildActorOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildCriticOutputOffsetScaleType(std::vector<cNeuralNet::eOffsetScaleType>& out_types) const;
	virtual void BuildActorOutputOffsetScaleType(std::vector<cNeuralNet::eOffsetScaleType>& out_types) const;

	virtual void RecordBegFlags(tExpTuple& out_tuple) const;
	virtual bool CheckOffPolicy() const;

	virtual void BuildActionBounds(Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const;
};