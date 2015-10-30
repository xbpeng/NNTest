#include "DrawScenarioArmRL.h"
#include "render/DrawUtil.h"
#include "render/DrawSimCharacter.h"
#include "util/FileUtil.h"

const tVector gLineColor = tVector(0, 0, 0, 1);
const tVector gCoachFillTint = tVector(0, 0, 0, 0);
const tVector gCamPos0 = tVector(0, 0, 1, 0);

const std::string gCoachTorqueFilename = "output/coach_torques.txt";
const std::string gStudentTorqueFilename = "output/student_torques.txt";

cDrawScenarioArmRL::cDrawScenarioArmRL(cCamera& cam)
	: cDrawScenario(cam)
{
	cam.TranslateToPos(gCamPos0);
	mOutputNetFile = "";
	mMouseDown = false;
	mOutputTorques = false;
	mCoachTorqueFile = nullptr;
	mStudentTorqueFile = nullptr;
}

cDrawScenarioArmRL::~cDrawScenarioArmRL()
{
}

void cDrawScenarioArmRL::ParseArgs(const cArgParser& parser)
{
	cDrawScenario::ParseArgs(parser);
	mArgParser = parser;

	parser.ParseString("output_net_file", mOutputNetFile);
}

void cDrawScenarioArmRL::Keyboard(unsigned char key, int x, int y)
{
	cDrawScenario::Keyboard(key, x, y);

	switch (key)
	{
	case 't':
		ToggleTraining();
		break;
	case 's':
		SaveNet(mOutputNetFile);
		break;
	case 'a':
		ToggleAutoTarget();
		break;
	case 'w':
		ToggleOutputTorques();
		break;
	case 'p':
		ToggleRandPose();
		break;
	default:
		break;
	}
}

void cDrawScenarioArmRL::Update(double time_elapsed)
{
	cDrawScenario::Update(time_elapsed);
	mScene->Update(time_elapsed);

	if (mOutputTorques)
	{
		WriteTorques();
	}
}

void cDrawScenarioArmRL::Init()
{
	cDrawScenario::Init();
	BuildScene();
	mScene->ParseArgs(mArgParser);
	mScene->Init();
	mMouseDown = false;
}

void cDrawScenarioArmRL::Reset()
{
	cDrawScenario::Reset();
	mScene->Reset();
	mMouseDown = false;
}

void cDrawScenarioArmRL::DrawScene()
{
	DrawGrid();
	DrawTarget();
	DrawCharacter();
	DrawViewRT();
}

void cDrawScenarioArmRL::DrawCharacter()
{
	const auto& coach = mScene->GetCoach();
	glPushMatrix();
	mScene->DrawArm(coach, gCoachFillTint, gLineColor);
	glPopMatrix();
	
	mScene->DrawCharacter();
}

void cDrawScenarioArmRL::DrawTarget() const
{
	mScene->DrawTarget();
}

void cDrawScenarioArmRL::ToggleTraining()
{
	mScene->ToggleTraining();
	bool enable_training = mScene->EnableTraining();
	if (enable_training)
	{
		printf("Training enabled\n");
	}
	else
	{
		printf("Training disabled\n");
	}
}

void cDrawScenarioArmRL::ToggleAutoTarget()
{
	bool enable_auto_target = mScene->EnabledAutoTarget();
	enable_auto_target = !enable_auto_target;
	mScene->EnableAutoTarget(enable_auto_target);
	
	if (enable_auto_target)
	{
		printf("Auto Target enabled\n");
	}
	else
	{
		printf("Auto Target disabled\n");
	}
}

void cDrawScenarioArmRL::ToggleRandPose()
{
	bool enable_rand_pose = mScene->EnabledRandPose();
	enable_rand_pose = !enable_rand_pose;
	mScene->EnableRandPose(enable_rand_pose);

	if (enable_rand_pose)
	{
		printf("Rand Pose Enabled\n");
	}
	else
	{
		printf("Rand Pose Disabled\n");
	}
}

const std::shared_ptr<cScenarioSimChar>& cDrawScenarioArmRL::GetScene() const
{
	return mSimScene;
}

void cDrawScenarioArmRL::BuildScene()
{
	mScene = std::shared_ptr<cScenarioArmRL>(new cScenarioArmRL());
	mSimScene = std::static_pointer_cast<cScenarioSimChar>(mScene); // arg hack
}

void cDrawScenarioArmRL::SetTarget(const tVector& target)
{
	mScene->SetTargetPos(target);
}

void cDrawScenarioArmRL::SaveNet(const std::string& out_file) const
{
	mScene->SaveNet(out_file);
	printf("Model saved to %s\n", out_file.c_str());
}

std::string cDrawScenarioArmRL::GetName() const
{
	return mScene->GetName();
}

void cDrawScenarioArmRL::DrawGrid() const
{
	const double spacing = 0.10f;
	const double big_spacing = spacing * 5.f;
	const tVector& origin = mCam.GetFocus();
	tVector size = tVector(mCam.GetWidth(), mCam.GetHeight(), 0, 0);

	glColor4f(188 / 255.f, 219 / 255.f, 242 / 255.f, 1.f);
	cDrawUtil::DrawGrid2D(origin, size, spacing, big_spacing);
}

void cDrawScenarioArmRL::DrawViewRT() const
{
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	const double aspect = mCam.GetAspectRatio();
	
	const auto& view_rt = mScene->GetViewRT();
	const tVector size = 0.5 * tVector(1 / aspect, 1, 0, 0);
	const tVector pos = tVector(0.96, 0.96, -1, 0) - 0.5 * size;

	cDrawUtil::SetColor(tVector(1, 1, 1, 1));
	cDrawUtil::DrawTexQuad(*view_rt.get(), pos, size);

	cDrawUtil::SetLineWidth(1);
	cDrawUtil::SetColor(tVector(0, 0, 0, 1));
	cDrawUtil::DrawRect(pos, size, cDrawUtil::eDrawWire);

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
}

std::string cDrawScenarioArmRL::BuildTextInfoStr() const
{
	std::string info = "";
	return info;
}

void cDrawScenarioArmRL::MouseClick(int button, int state, double x, double y)
{
	cDrawScenario::MouseClick(button, state, x, y);

	if (state == GLUT_DOWN)
	{
		tVector click_pos = tVector(x, y, 0, 0);
		tVector target_pos = mCam.ScreenToWorldPos(click_pos);
		target_pos[2] = 0;
		SetTarget(target_pos);
		mMouseDown = true;
	}
	else
	{
		mMouseDown = false;
	}
}

void cDrawScenarioArmRL::MouseMove(double x, double y)
{
	cDrawScenario::MouseMove(x, y);
	if (mMouseDown)
	{
		tVector click_pos = tVector(x, y, 0, 0);
		tVector target_pos = mCam.ScreenToWorldPos(click_pos);
		target_pos[2] = 0;
		SetTarget(target_pos);
	}
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
	WriteTorques(mScene->GetCoach(), mCoachTorqueFile);
	WriteTorques(mScene->GetCharacter(), mStudentTorqueFile);
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