#include "DrawScenarioRNN.h"
#include "render/DrawUtil.h"
#include "render/DrawCharacter.h"

const tVector gCamPos0 = tVector(0, 0, 1, 0);

cDrawScenarioRNN::cDrawScenarioRNN(cCamera& cam)
	: cDrawScenario(cam)
{
	cam.TranslatePos(gCamPos0);
	mAutoTrainer = false;
}

cDrawScenarioRNN::~cDrawScenarioRNN()
{
}

void cDrawScenarioRNN::Init()
{
	BuildScene(mScene);
	mScene->ParseArgs(mArgParser);
	mScene->Init();
	mAutoTrainer = false;
}

void cDrawScenarioRNN::ParseArgs(const cArgParser& parser)
{
	cDrawScenario::ParseArgs(parser);
	mArgParser = parser;
}

void cDrawScenarioRNN::Reset()
{
	mScene->Reset();
	mAutoTrainer = false;
}

void cDrawScenarioRNN::Clear()
{
	mScene->Clear();
}

void cDrawScenarioRNN::Update(double time_elapsed)
{
	mScene->Update(time_elapsed);
	if (mAutoTrainer)
	{
		TrainNet();
	}
}

void cDrawScenarioRNN::Keyboard(unsigned char key, int x, int y)
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
	default:
		break;
	}
}

std::string cDrawScenarioRNN::GetName() const
{
	return mScene->GetName();
}

void cDrawScenarioRNN::BuildScene(std::unique_ptr<cScenarioRNN>& out_scene)
{
	out_scene = std::unique_ptr<cScenarioRNN>(new cScenarioRNN());
}

void cDrawScenarioRNN::DrawScene()
{
	DrawPoints();
	DrawNetEval();
}

void cDrawScenarioRNN::DrawPoints() const
{
	tVector col = tVector(1, 0, 0, 1);
	tVector line_col = tVector(0, 0, 0, 1);

	double pt_r = 0.015;
	int slices = 16;
	int num_points = mScene->GetNumPts();

	glLineWidth(1);
	for (int i = 0; i < num_points; ++i)
	{
		const tVector pt = mScene->GetPt(i);
		glPushMatrix();
		glTranslated(pt[0], pt[1], pt[2]);

		glColor4d(col[0], col[1], col[2], col[3]);
		cDrawUtil::DrawDisk(pt_r, slices);

		glColor4d(line_col[0], line_col[1], line_col[2], line_col[3]);
		cDrawUtil::DrawDisk(pt_r, slices, cDrawUtil::eDrawWire);

		glPopMatrix();
	}
}

void cDrawScenarioRNN::DrawNetEval() const
{
	const tVector col = tVector(0, 0, 1, 0.75);
	const double weight = 3;

	const auto& eval_pts = mScene->GetEvalPts();
	glColor4d(col[0], col[1], col[2], col[3]);
	glLineWidth(static_cast<float>(weight));
	cDrawUtil::DrawLineStrip(eval_pts);
}

void cDrawScenarioRNN::TrainNet()
{
	mScene->TrainNet();
}