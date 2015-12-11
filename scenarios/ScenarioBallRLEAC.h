#pragma once

#include "scenarios/ScenarioBallRL.h"
#include "stuff/BallControllerEAC.h"
#include "learning/EACTrainer.h"

class cScenarioBallRLEAC : public cScenarioBallRL
{
public:
	cScenarioBallRLEAC();
	virtual ~cScenarioBallRLEAC();

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

	virtual std::shared_ptr<cBallControllerEAC> GetEACCtrl() const;
};