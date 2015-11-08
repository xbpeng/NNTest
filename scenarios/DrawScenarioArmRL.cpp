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

const std::string gCoachTorqueFilename = "output/coach_torques.txt";
const std::string gStudentTorqueFilename = "output/student_torques.txt";

cDrawScenarioArmRL::cDrawScenarioArmRL(cCamera& cam)
	: cDrawScenarioArm(cam)
{
	mOutputNetFile = "";
	mOutputTorques = false;
	mCoachTorqueFile = nullptr;
	mStudentTorqueFile = nullptr;
}

cDrawScenarioArmRL::~cDrawScenarioArmRL()
{
}

void cDrawScenarioArmRL::ParseArgs(const cArgParser& parser)
{
	cDrawScenarioArm::ParseArgs(parser);
	parser.ParseString("output_net_file", mOutputNetFile);
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
	case 'w':
		ToggleOutputTorques();
		break;
	case 'o':
		ToggleOutputData();
		break;
	default:
		break;
	}
}

void cDrawScenarioArmRL::Update(double time_elapsed)
{
	cDrawScenarioArm::Update(time_elapsed);

	if (mOutputTorques)
	{
		WriteTorques();
	}
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

void cDrawScenarioArmRL::ToggleOutputTorques()
{
	if (mOutputTorques)
	{
		EndWrite();
		printf("End Writing Torques.\n");
	}
	else
	{
		BeginWrite();
		printf("Begin Writing Torques.\n");
	}
	mOutputTorques = !mOutputTorques;
}

void cDrawScenarioArmRL::BeginWrite()
{
	mCoachTorqueFile = cFileUtil::OpenFile(gCoachTorqueFilename, "w");
	mStudentTorqueFile = cFileUtil::OpenFile(gStudentTorqueFilename, "w");
}

void cDrawScenarioArmRL::EndWrite()
{
	cFileUtil::CloseFile(mCoachTorqueFile);
	cFileUtil::CloseFile(mStudentTorqueFile);
}

void cDrawScenarioArmRL::WriteTorques()
{
	auto rl_scene = GetRLScene();
	WriteTorques(rl_scene->GetCoach(), mCoachTorqueFile);
	WriteTorques(rl_scene->GetCharacter(), mStudentTorqueFile);
}

void cDrawScenarioArmRL::WriteTorques(const std::shared_ptr<cSimCharacter>& character, FILE* out_file) const
{
	int num_joints = character->GetNumJoints();
	for (int j = 0; j < num_joints; ++j)
	{
		const cJoint& joint = character->GetJoint(j);
		tVector torque = tVector::Zero();
		
		if (joint.IsValid())
		{
			torque = joint.GetTorque();
		}
		
		fprintf(out_file, "%.5f\t", torque[2]);
	}
	fprintf(out_file, "\n");
}

void cDrawScenarioArmRL::ToggleOutputData()
{
	auto rl_scene = GetRLScene();
	bool enable = rl_scene->EnabledOutputData();
	rl_scene->EnableOutputData(!enable);

	enable = rl_scene->EnabledOutputData();
	if (enable)
	{
		printf("Begin writing data\n");
	}
	else
	{
		printf("End writing data\n");
	}
}