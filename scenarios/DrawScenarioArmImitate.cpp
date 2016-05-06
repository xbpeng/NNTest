#include "DrawScenarioArmImitate.h"
#include "ScenarioArmImitate.h"
#include "render/DrawCharacter.h"
#include "render/DrawUtil.h"

const double gLinkWidth = 0.025f;
const tVector gLineColor = tVector(0, 0, 0, 1);
const tVector gFilLColor = tVector(0.6f, 0.65f, 0.675f, 1);
const tVector gKinCharDrawOffset = tVector(0, 0, 0.5, 0);

cDrawScenarioArmImitate::cDrawScenarioArmImitate(cCamera& cam)
	: cDrawScenarioArm(cam)
{
}

cDrawScenarioArmImitate::~cDrawScenarioArmImitate()
{
}

void cDrawScenarioArmImitate::BuildScene()
{
	mScene = std::shared_ptr<cScenarioArmImitate>(new cScenarioArmImitate());
	mSimScene = std::static_pointer_cast<cScenarioSimChar>(mScene);
}

void cDrawScenarioArmImitate::SetTarget(const tVector& target)
{
	// do nothing
}

void cDrawScenarioArmImitate::DrawCharacter()
{
	cDrawScenarioArm::DrawCharacter();
	DrawKinChar();
}

void cDrawScenarioArmImitate::DrawKinChar()
{
	auto scene = std::static_pointer_cast<cScenarioArmImitate>(mScene);
	const auto& kin_char = scene->GetKinChar();

	glPushMatrix();
	cDrawUtil::Translate(gKinCharDrawOffset);
	cDrawCharacter::Draw(*kin_char.get(), gLinkWidth, gFilLColor, gLineColor);
	glPopMatrix();
}