#include "DrawScenarioArmTrain.h"
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

cDrawScenarioArmTrain::cDrawScenarioArmTrain(cCamera& cam)
	: cDrawScenarioArm(cam)
{
	mOutputNetFile = "";
}

cDrawScenarioArmTrain::~cDrawScenarioArmTrain()
{
}

void cDrawScenarioArmTrain::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cDrawScenarioArm::ParseArgs(parser);
	parser->ParseString("output_net_file", mOutputNetFile);
}

void cDrawScenarioArmTrain::Keyboard(unsigned char key, int x, int y)
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

void cDrawScenarioArmTrain::Update(double time_elapsed)
{
	cDrawScenarioArm::Update(time_elapsed);
}

void cDrawScenarioArmTrain::DrawCharacter()
{
	cDrawScenarioArm::DrawCharacter();
}

void cDrawScenarioArmTrain::ToggleTraining()
{
	auto rl_scene = GetTrainScene();
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

std::shared_ptr<cScenarioArmTrain> cDrawScenarioArmTrain::GetTrainScene() const
{
	return std::static_pointer_cast<cScenarioArmTrain>(mScene);
}

void cDrawScenarioArmTrain::BuildScene()
{
	mScene = std::shared_ptr<cScenarioArmTrain>(new cScenarioArmTrain());
	mSimScene = std::static_pointer_cast<cScenarioSimChar>(mScene); // arg hack
}

void cDrawScenarioArmTrain::SaveNet(const std::string& out_file) const
{
	auto scene = GetTrainScene();
	scene->SaveNet(out_file);
	printf("Model saved to %s\n", out_file.c_str());
}