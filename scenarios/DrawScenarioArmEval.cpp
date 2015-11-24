#include "DrawScenarioArmEval.h"
#include "ScenarioArmEval.h"

cDrawScenarioArmEval::cDrawScenarioArmEval(cCamera& cam)
	: cDrawScenarioArm(cam)
{
}

cDrawScenarioArmEval::~cDrawScenarioArmEval()
{
}

void cDrawScenarioArmEval::Keyboard(unsigned char key, int x, int y)
{
	cDrawScenarioArm::Keyboard(key, x, y);

	switch (key)
	{
	case 'o':
		ToggleOutputData();
		break;
	default:
		break;
	}
}

void cDrawScenarioArmEval::BuildScene()
{
	mScene = std::shared_ptr<cScenarioArmEval>(new cScenarioArmEval());
	mSimScene = std::static_pointer_cast<cScenarioSimChar>(mScene); // arg hack
}

std::string cDrawScenarioArmEval::BuildTextInfoStr() const
{
	std::string str = cDrawScenarioArm::BuildTextInfoStr();

	const cScenarioArmEval* eval_scene = reinterpret_cast<cScenarioArmEval*>(mScene.get());

	double avg_err = eval_scene->GetAvgErr();
	int err_samples = eval_scene->GetErrSampleCount();

	char str_buffer[128];
	sprintf(str_buffer, "Avg Error: %.4f\nError Samples: %i\n", avg_err, err_samples);
	str += str_buffer;

	return str;
}

void cDrawScenarioArmEval::ToggleOutputData()
{
	cScenarioArmEval* eval_scene = reinterpret_cast<cScenarioArmEval*>(mScene.get());
	bool enable = eval_scene->EnabledOutputData();
	eval_scene->EnableOutputData(!enable);

	enable = eval_scene->EnabledOutputData();
	if (enable)
	{
		printf("Begin writing data\n");
	}
	else
	{
		printf("End writing data\n");
	}
}