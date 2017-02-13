#include "DrawScenarioArmRL.h"
#include "render/DrawUtil.h"
#include "render/DrawSimCharacter.h"
#include "render/DrawWorld.h"
#include "stuff/SimArm.h"
#include "util/FileUtil.h"

const tVector gLineColor = tVector(0, 0, 0, 1);
const tVector gCoachFillTint = tVector(0, 0, 0, 0);
const tVector gCamPos0 = tVector(0, 0, 1, 0);

const int gTracerBufferSize = 50;
const double gTracerSamplePeriod = 0.033;

cDrawScenarioArmRL::cDrawScenarioArmRL(cCamera& cam)
	: cDrawScenarioArm(cam)
{
	mOutputNetFile = "";
}

cDrawScenarioArmRL::~cDrawScenarioArmRL()
{
}

void cDrawScenarioArmRL::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cDrawScenarioArm::ParseArgs(parser);
	parser->ParseString("output_net_file", mOutputNetFile);
}

void cDrawScenarioArmRL::Keyboard(unsigned char key, int x, int y)
{
	cDrawScenarioArm::Keyboard(key, x, y);

	switch (key)
	{
	case 't':
		ToggleTraining();
		break;
	case 's':
		SaveNet(mOutputNetFile);
		break;
	default:
		break;
	}
}

void cDrawScenarioArmRL::Update(double time_elapsed)
{
	cDrawScenarioArm::Update(time_elapsed);
}

void cDrawScenarioArmRL::DrawCharacter()
{
	auto rl_scene = GetRLScene();
	const auto& coach = rl_scene->GetCoach();
	glPushMatrix();
	cDrawUtil::Translate(tVector(0, 0, -0.01, 0));
	mScene->DrawArm(coach, gCoachFillTint, gLineColor);
	glPopMatrix();
	
	cDrawScenarioArm::DrawCharacter();
}

void cDrawScenarioArmRL::ToggleTraining()
{
	auto rl_scene = GetRLScene();
	rl_scene->ToggleTraining();
	bool enable_training = rl_scene->EnableTraining();
	if (enable_training)
	{
		printf("Training enabled\n");
	}
	else
	{
		printf("Training disabled\n");
	}
}

std::shared_ptr<cScenarioArmRL> cDrawScenarioArmRL::GetRLScene() const
{
	return std::static_pointer_cast<cScenarioArmRL>(mScene);
}

void cDrawScenarioArmRL::BuildScene()
{
	mScene = std::shared_ptr<cScenarioArmRL>(new cScenarioArmRL());
	mSimScene = std::static_pointer_cast<cScenarioSimChar>(mScene); // arg hack
}

void cDrawScenarioArmRL::InitTracer()
{
	cDrawScenarioArm::InitTracer();
	auto rl_scene = GetRLScene();
	AddCharTrace(rl_scene->GetCoach(), tVector(0, 0, 0, 0.5));
}

void cDrawScenarioArmRL::SaveNet(const std::string& out_file) const
{
	auto rl_scene = GetRLScene();
	rl_scene->SaveNet(out_file);
	printf("Model saved to %s\n", out_file.c_str());
}