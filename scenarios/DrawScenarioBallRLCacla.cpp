#include "DrawScenarioBallRLCacla.h"

cDrawScenarioBallRLCacla::cDrawScenarioBallRLCacla(cCamera& cam)
	: cDrawScenarioBallRL(cam)
{
	mCriticOutputNetFile = "";
}

cDrawScenarioBallRLCacla::~cDrawScenarioBallRLCacla()
{
}

void cDrawScenarioBallRLCacla::ParseArgs(const cArgParser& parser)
{
	cDrawScenarioBallRL::ParseArgs(parser);
	parser.ParseString("critic_output_net_file", mCriticOutputNetFile);
}

void cDrawScenarioBallRLCacla::BuildScene()
{
	mScene = std::shared_ptr<cScenarioBallRLCacla>(new cScenarioBallRLCacla());
}

void cDrawScenarioBallRLCacla::SaveNet() const
{
	std::shared_ptr<cScenarioBallRLCacla> cacla_scene = std::static_pointer_cast<cScenarioBallRLCacla>(mScene);
	if (mOutputNetFile != "")
	{
		cacla_scene->SaveActorNet(mOutputNetFile);
	}
	
	if (mCriticOutputNetFile != "")
	{
		cacla_scene->SaveCriticNet(mCriticOutputNetFile);
	}
}