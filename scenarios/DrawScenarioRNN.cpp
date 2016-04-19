#include "DrawScenarioRNN.h"
#include "render/DrawUtil.h"

cDrawScenarioRNN::cDrawScenarioRNN(cCamera& cam)
	: cDrawScenarioReg1D(cam)
{
	mNewSeq = true;
}

cDrawScenarioRNN::~cDrawScenarioRNN()
{
}

void cDrawScenarioRNN::Init()
{
	cDrawScenarioReg1D::Init();
	mNewSeq = true;
}

void cDrawScenarioRNN::Reset()
{
	cDrawScenarioReg1D::Reset();
	mNewSeq = true;
}

void cDrawScenarioRNN::MouseClick(int button, int state, double x, double y)
{
	cDrawScenarioReg1D::MouseClick(button, state, x, y);
	if (state == GLUT_DOWN)
	{
		mNewSeq = false;
	}
	else if (state == GLUT_UP)
	{
		mNewSeq = true;
	}
}

void cDrawScenarioRNN::MouseMove(double x, double y)
{
	if (mMousePressed)
	{
		cDrawScenarioReg1D::MouseMove(x, y);
	}
}

void cDrawScenarioRNN::BuildScene(std::unique_ptr<cScenarioReg1D>& out_scene)
{
	out_scene = std::unique_ptr<cScenarioReg1D>(new cScenarioRNN());
}

void cDrawScenarioRNN::AddPt(const tVector& pt)
{
	bool is_start = mNewSeq;
	auto scene = static_cast<cScenarioRNN*>(mScene.get());
	scene->AddPt(pt, is_start);
}

void cDrawScenarioRNN::DrawNetEval() const
{
	const tVector col = tVector(0, 0, 1, 0.75);
	const double weight = 3;

	const auto& eval_pts = mScene->GetEvalPts();
	glColor4d(col[0], col[1], col[2], col[3]);
	cDrawUtil::SetLineWidth(weight);

	const int num_pts = static_cast<int>(eval_pts.size());
	for (int i = 0; i < num_pts - 1; ++i)
	{
		int idx = i;
		const tVector& curr_pt = eval_pts[idx];
		const tVector& next_pt = eval_pts[idx + 1];
		bool is_end = next_pt[3] != 0;
		if (!is_end)
		{
			cDrawUtil::DrawLine(curr_pt, next_pt);
		}
	}
}