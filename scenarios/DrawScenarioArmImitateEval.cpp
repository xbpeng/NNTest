#include "DrawScenarioArmImitateEval.h"
#include "ScenarioArmImitateEval.h"
#include "render/DrawCharacter.h"
#include "render/DrawUtil.h"

const double gLinkWidth = 0.025f;
const tVector gLineColor = tVector(0, 0, 0, 1);
const tVector gFilLColor = tVector(0.6f, 0.65f, 0.675f, 1);
const tVector gKinCharDrawOffset = tVector(0, 0, 0.5, 0);

cDrawScenarioArmImitateEval::cDrawScenarioArmImitateEval(cCamera& cam)
	: cDrawScenarioArmEval(cam)
{
}

cDrawScenarioArmImitateEval::~cDrawScenarioArmImitateEval()
{
}

void cDrawScenarioArmImitateEval::BuildScene()
{
	mScene = std::shared_ptr<cScenarioArmEval>(new cScenarioArmImitateEval());
	mSimScene = std::static_pointer_cast<cScenarioSimChar>(mScene); // arg hack
}

void cDrawScenarioArmImitateEval::DrawCharacter()
{
	cDrawScenarioArmEval::DrawCharacter();
	DrawKinChar();
}

void cDrawScenarioArmImitateEval::DrawKinChar()
{
	auto scene = std::static_pointer_cast<cScenarioArmImitateEval>(mScene);
	const auto& kin_char = scene->GetKinChar();

	glPushMatrix();
	cDrawUtil::Translate(gKinCharDrawOffset);
	cDrawCharacter::Draw(*kin_char.get(), gLinkWidth, gFilLColor, gLineColor);
	glPopMatrix();
}

void cDrawScenarioArmImitateEval::SetTarget(const tVector& target)
{
	// do nothing
}