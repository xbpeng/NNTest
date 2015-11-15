#pragma once

#include "scenarios/ScenarioBallRL.h"
#include "stuff/BallControllerCont.h"
#include "learning/CaclaTrainer.h"

class cScenarioBallRLCacla : public cScenarioBallRL
{
public:
	cScenarioBallRLCacla();
	virtual ~cScenarioBallRLCacla();

	virtual void ParseArgs(const cArgParser& parser);

	virtual std::string GetName() const;
	
protected:
	std::string mCriticSolverFile;
	std::string mCriticNetFile;

	virtual void BuildController(std::shared_ptr<cBallController>& out_ctrl);
	virtual void InitTrainer();
	virtual void BuildOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;

	virtual void RecordBegFlags(tExpTuple& out_tuple) const;
	virtual bool CheckOffPolicy() const;
};