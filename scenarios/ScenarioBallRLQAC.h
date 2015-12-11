#pragma once

#include "scenarios/ScenarioBallRL.h"
#include "stuff/BallControllerQAC.h"
#include "learning/QACTrainer.h"

class cScenarioBallRLQAC : public cScenarioBallRL
{
public:
	cScenarioBallRLQAC();
	virtual ~cScenarioBallRLQAC();

	virtual void ParseArgs(const cArgParser& parser);

	virtual std::string GetName() const;
	
protected:

	virtual void BuildController(std::shared_ptr<cBallController>& out_ctrl);
	virtual void InitTrainer();
	virtual void BuildOutputOffsetScale(const std::shared_ptr<cNeuralNetTrainer>& trainer,
										Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;

	virtual void RecordBegFlags(tExpTuple& out_tuple) const;
	virtual bool CheckOffPolicy() const;
	virtual bool CheckExplore() const;

	virtual int GetNumActionFrags() const;
	virtual int GetActionFragSize() const;

	virtual std::shared_ptr<cBallControllerQAC> GetQACCtrl() const;
};