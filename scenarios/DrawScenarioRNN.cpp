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