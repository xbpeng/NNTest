#include "DrawScenarioBallRLVarCacla.h"

cDrawScenarioBallRLVarCacla::cDrawScenarioBallRLVarCacla(cCamera& cam)
	: cDrawScenarioBallRLCacla(cam)
{
}

cDrawScenarioBallRLVarCacla::~cDrawScenarioBallRLVarCacla()
{
}

void cDrawScenarioBallRLVarCacla::BuildScene()
{
	mScene = std::shared_ptr<cScenarioBallRLCacla>(new cScenarioBallRLVarCacla());
}