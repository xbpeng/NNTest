#pragma once

#include "scenarios/ScenarioBallRLACE.h"
#include "stuff/BallControllerMACE.h"
#include "learning/MACETrainer.h"

class cScenarioBallRLMACE : public cScenarioBallRLACE
{
public:
	cScenarioBallRLMACE();
	virtual ~cScenarioBallRLMACE();

	virtual void ParseArgs(const cArgParser& parser);

	virtual std::string GetName() const;
	
protected:
	std::string mCriticNetFile;
	std::string mCriticSolverFile;

	virtual void SetupController();
	virtual void BuildController(std::shared_ptr<cBallController>& out_ctrl);
	virtual void InitTrainer();
	virtual void SetupTrainerOutputOffsetScale();

	virtual std::shared_ptr<cBallControllerMACE> GetMACECtrl() const;
	virtual std::shared_ptr<cMACETrainer> GetMACETrainer() const;

	virtual void BuildCriticOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildActorOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
};