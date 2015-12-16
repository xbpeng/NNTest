#include "DrawScenarioArm.h"
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

cDrawScenarioArm::cDrawScenarioArm(cCamera& cam)
	: cDrawScenario(cam)
{
	cam.TranslateToPos(gCamPos0);
	mMouseDown = false;
}

cDrawScenarioArm::~cDrawScenarioArm()
{
}

void cDrawScenarioArm::ParseArgs(const cArgParser& parser)
{
	cDrawScenario::ParseArgs(parser);
	mArgParser = parser;
}

void cDrawScenarioArm::Keyboard(unsigned char key, int x, int y)
{
	cDrawScenario::Keyboard(key, x, y);

	switch (key)
	{
	case 'a':
		ToggleAutoTarget();
		break;
	case 'g':
		ApplyRandForce();
		break;
	case 'p':
		ToggleRandPose();
		break;
	case 'y':
		ToggleTrace();
		break;
	default:
		break;
	}
}

void cDrawScenarioArm::Update(double time_elapsed)
{
	cDrawScenario::Update(time_elapsed);
	mScene->Update(time_elapsed);

	if (mEnableTrace)
	{
		mTracer.Update(time_elapsed);
	}
}

void cDrawScenarioArm::Init()
{
	cDrawScenario::Init();
	BuildScene();
	mScene->ParseArgs(mArgParser);
	mScene->Init();
	InitTracer();
	mMouseDown = false;
}

void cDrawScenarioArm::Clear()
{
	cDrawScenario::Clear();
	mScene->Clear();
	mTracer.Clear();
}

void cDrawScenarioArm::Reset()
{
	cDrawScenario::Reset();
	mScene->Reset();
	mMouseDown = false;
	mTracer.Reset();
}

bool cDrawScenarioArm::IsDone() const
{
	return mScene->IsDone();
}

void cDrawScenarioArm::DrawScene()
{
	DrawGrid();
	DrawTarget();
	DrawCharacter();
	DrawPerturbs();

	if (mEnableTrace)
	{
		mTracer.Draw();
	}

	DrawViewRT();
}

void cDrawScenarioArm::DrawCharacter()
{
	mScene->DrawCharacter();
}

void cDrawScenarioArm::DrawPerturbs() const
{
	const auto& world = mScene->GetWorld();
	cDrawWorld::DrawPerturbs(*world.get());
}

void cDrawScenarioArm::DrawTarget() const
{
	mScene->DrawTarget();
}

void cDrawScenarioArm::ToggleAutoTarget()
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

void cDrawScenarioArm::ToggleRandPose()
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

const std::shared_ptr<cScenarioSimChar>& cDrawScenarioArm::GetScene() const
{
	return mSimScene;
}

void cDrawScenarioArm::BuildScene()
{
	mScene = std::shared_ptr<cScenarioArm>(new cScenarioArm());
	mSimScene = std::static_pointer_cast<cScenarioSimChar>(mScene); // arg hack
}

void cDrawScenarioArm::InitTracer()
{
	mTracer.Init(gTracerBufferSize, gTracerSamplePeriod);
	AddCharTrace(mScene->GetCharacter(), tVector(0, 0, 1, 0.5));
}

void cDrawScenarioArm::AddCharTrace(const std::shared_ptr<cSimCharacter>& character,
									const tVector& col)
{
	cCharTracer::tParams params;
	params.mChar = character;
	params.mColors.push_back(col);
	params.mType = cCharTracer::eTraceJoint;
	params.mTraceID = cSimArm::eJointLinkEnd;
	mTracer.AddTrace(params);
}

void cDrawScenarioArm::ToggleTrace()
{
	mTracer.Reset();
	mEnableTrace = !mEnableTrace;

	if (mEnableTrace)
	{
		printf("Trace Enabled\n");
	}
	else
	{
		printf("Trace Disabled\n");
	}
}

void cDrawScenarioArm::SetTarget(const tVector& target)
{
	mScene->SetTargetPos(target);
}

void cDrawScenarioArm::ApplyRandForce()
{
	mScene->ApplyRandForce();
}

std::string cDrawScenarioArm::GetName() const
{
	return mScene->GetName();
}

void cDrawScenarioArm::DrawGrid() const
{
	const double spacing = 0.10f;
	const double big_spacing = spacing * 5.f;
	tVector origin = mCam.GetFocus();
	origin += tVector(0, 0, -1, 0);
	tVector size = tVector(mCam.GetWidth(), mCam.GetHeight(), 0, 0);

	glColor4f(188 / 255.f, 219 / 255.f, 242 / 255.f, 1.f);
	cDrawUtil::DrawGrid2D(origin, size, spacing, big_spacing);
}

void cDrawScenarioArm::DrawViewRT() const
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

	cDrawUtil::SetLineWidth(1);
	cDrawUtil::SetColor(tVector(1, 1, 1, 1));
	cDrawUtil::DrawRect(pos, size);
	cDrawUtil::SetColor(tVector(0, 0, 0, 1));
	cDrawUtil::DrawRect(pos, size, cDrawUtil::eDrawWire);

	cDrawUtil::SetColor(tVector(1, 1, 1, 1));
	cDrawUtil::DrawTexQuad(*view_rt.get(), pos, size);

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
}

std::string cDrawScenarioArm::BuildTextInfoStr() const
{
	std::string info = "";
	return info;
}

void cDrawScenarioArm::MouseClick(int button, int state, double x, double y)
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

void cDrawScenarioArm::MouseMove(double x, double y)
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