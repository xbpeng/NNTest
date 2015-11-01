#include "DrawScenarioArmRLPretrain.h"
#include "ScenarioArmRLPretrain.h"
#include "render/DrawUtil.h"

cDrawScenarioArmRLPretrain::cDrawScenarioArmRLPretrain(cCamera& cam)
	: cDrawScenarioArmRL(cam)
{
}

cDrawScenarioArmRLPretrain::~cDrawScenarioArmRLPretrain()
{
}

void cDrawScenarioArmRLPretrain::BuildScene()
{
	mScene = std::shared_ptr<cScenarioArmRLPretrain>(new cScenarioArmRLPretrain());
	mSimScene = std::static_pointer_cast<cScenarioSimChar>(mScene); // arg hack
}

void cDrawScenarioArmRLPretrain::DrawScene()
{
	cDrawScenarioArmRL::DrawScene();
	DrawPredTarget();
}

void cDrawScenarioArmRLPretrain::DrawPredTarget() const
{
	auto scene = std::static_pointer_cast<cScenarioArmRLPretrain>(mScene);
	const tVector& pred_target = scene->GetPredTarget();

	cDrawUtil::SetLineWidth(4);
	cDrawUtil::SetColor(tVector(1, 0, 0, 0.5));
	cDrawUtil::DrawCross(pred_target, 0.15);
}