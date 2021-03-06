#include "DrawScenarioReg1D.h"
#include "render/DrawUtil.h"
#include "render/DrawCharacter.h"

const tVector gCamPos0 = tVector(0, 0, 1, 0);

cDrawScenarioReg1D::cDrawScenarioReg1D(cCamera& cam)
	: cDrawScenario(cam)
{
	cam.TranslatePos(gCamPos0);
	mMousePos.setZero();
	mMousePressed = false;
	mAutoTrainer = false;
}

cDrawScenarioReg1D::~cDrawScenarioReg1D()
{
}

void cDrawScenarioReg1D::Init()
{
	BuildScene(mScene);
	mScene->ParseArgs(mArgParser);
	mScene->Init();
	mMousePressed = false;
	mAutoTrainer = false;
}

void cDrawScenarioReg1D::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cDrawScenario::ParseArgs(parser);
	mArgParser = parser;
}

void cDrawScenarioReg1D::Reset()
{
	mScene->Reset();
	mMousePressed = false;
	mAutoTrainer = false;
}

void cDrawScenarioReg1D::Clear()
{
	mScene->Clear();
}

void cDrawScenarioReg1D::Update(double time_elapsed)
{
	mScene->Update(time_elapsed);
	if (mAutoTrainer)
	{
		TrainNet();
	}
}

void cDrawScenarioReg1D::Keyboard(unsigned char key, int x, int y)
{
	cDrawScenario::Keyboard(key, x, y);

	switch (key)
	{
	case '\r':
		TrainNet();
		break;
	case 't':
		mAutoTrainer = !mAutoTrainer;
		break;
	case 's':
		OutputPoints();
		break;
	default:
		break;
	}
}

void cDrawScenarioReg1D::MouseClick(int button, int state, double x, double y)
{
	cDrawScenario::MouseClick(button, state, x, y);

	if (state == GLUT_DOWN)
	{
		mMousePressed = true;
		tVector pos = mCam.ScreenToWorldPos(tVector(x, y, 0, 0));
		AddPt(pos);
		mMousePos = pos;
	}
	else if (state == GLUT_UP)
	{
		mMousePressed = false;
	}
}

void cDrawScenarioReg1D::MouseMove(double x, double y)
{
	if (mMousePressed)
	{
		const double dist_threshold = 0.005;
		tVector curr_pos = mCam.ScreenToWorldPos(tVector(x, y, 0, 0));
		double dist = (mMousePos - curr_pos).squaredNorm();
		if (dist > dist_threshold)
		{
			AddPt(curr_pos);
			mMousePos = curr_pos;
		}
	}
}

std::string cDrawScenarioReg1D::GetName() const
{
	return mScene->GetName();
}

void cDrawScenarioReg1D::BuildScene(std::unique_ptr<cScenarioReg1D>& out_scene)
{
	out_scene = std::unique_ptr<cScenarioReg1D>(new cScenarioReg1D());
}

void cDrawScenarioReg1D::DrawScene()
{
	DrawPoints();
	DrawNetEval();
}

void cDrawScenarioReg1D::DrawPoints() const
{
	tVector col = tVector(1, 0, 0, 1);
	tVector line_col = tVector(0, 0, 0, 1);

	double pt_r = 0.015;
	int num_points = mScene->GetNumPts();

	glLineWidth(1);
	for (int i = 0; i < num_points; ++i)
	{
		const tVector pt = mScene->GetPt(i);
		glPushMatrix();
		glTranslated(pt[0], pt[1], pt[2]);

		glColor4d(col[0], col[1], col[2], col[3]);
		cDrawUtil::DrawDisk(pt_r);

		glColor4d(line_col[0], line_col[1], line_col[2], line_col[3]);
		cDrawUtil::DrawDisk(pt_r, cDrawUtil::eDrawWire);

		glPopMatrix();
	}
}

void cDrawScenarioReg1D::DrawNetEval() const
{
	const tVector col = tVector(0, 0, 1, 0.75);
	const double weight = 3;

	const auto& eval_pts = mScene->GetEvalPts();
	glColor4d(col[0], col[1], col[2], col[3]);
	cDrawUtil::SetLineWidth(weight);
	cDrawUtil::DrawLineStrip(eval_pts);
}

void cDrawScenarioReg1D::TrainNet()
{
	mScene->TrainNet();
}

void cDrawScenarioReg1D::OutputPoints() const
{
	mScene->OutputPoints();
}

void cDrawScenarioReg1D::AddPt(const tVector& pt)
{
	mScene->AddPt(pt);
}