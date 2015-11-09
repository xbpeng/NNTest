#include "DrawScenarioBallRLCacla.h"

cDrawScenarioBallRLCacla::cDrawScenarioBallRLCacla(cCamera& cam)
	: cDrawScenarioBallRL(cam)
{
}

cDrawScenarioBallRLCacla::~cDrawScenarioBallRLCacla()
{
}

void cDrawScenarioBallRLCacla::BuildScene()
{
	mScene = std::shared_ptr<cScenarioBallRLCacla>(new cScenarioBallRLCacla());
}