#include "DrawScenarioBallRLMACEDPG.h"
#include "scenarios/ScenarioBallRLMACEDPG.h"
#include "stuff/BallControllerMACEDPG.h"
#include "render/DrawUtil.h"

cDrawScenarioBallRLMACEDPG::cDrawScenarioBallRLMACEDPG(cCamera& cam)
	: cDrawScenarioBallRLMACE(cam)
{
}

cDrawScenarioBallRLMACEDPG::~cDrawScenarioBallRLMACEDPG()
{
}

void cDrawScenarioBallRLMACEDPG::ParseArgs(const cArgParser& parser)
{
	cDrawScenarioBallRL::ParseArgs(parser);
	parser.ParseString("critic_output_net_file", mCriticOutputNetFile);
}

void cDrawScenarioBallRLMACEDPG::BuildScene()
{
	mScene = std::shared_ptr<cScenarioBallRLDPG>(new cScenarioBallRLMACEDPG());
}

void cDrawScenarioBallRLMACEDPG::SaveNet() const
{
	std::shared_ptr<cScenarioBallRLMACEDPG> scene = std::static_pointer_cast<cScenarioBallRLMACEDPG>(mScene);
	if (mOutputNetFile != "")
	{
		scene->SaveActorNet(mOutputNetFile);
	}

	if (mCriticOutputNetFile != "")
	{
		scene->SaveCriticNet(mCriticOutputNetFile);
	}
}