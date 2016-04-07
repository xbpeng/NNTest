#include "DrawScenarioRegVar1D.h"
#include "scenarios/ScenarioRegVar1D.h"
#include "render/DrawUtil.h"

const tVector gCamPos0 = tVector(0, 0, 1, 0);

cDrawScenarioRegVar1D::cDrawScenarioRegVar1D(cCamera& cam)
	: cDrawScenarioReg1D(cam)
{
}

cDrawScenarioRegVar1D::~cDrawScenarioRegVar1D()
{
}

void cDrawScenarioRegVar1D::BuildScene(std::unique_ptr<cScenarioReg1D>& out_scene)
{
	out_scene = std::unique_ptr<cScenarioRegVar1D>(new cScenarioRegVar1D());
}

void cDrawScenarioRegVar1D::DrawNetEval() const
{
	cDrawScenarioReg1D::DrawNetEval();

	const tVector col0 = tVector(0, 0, 1, 0.75);
	const tVector col1 = tVector(0, 0.5, 0, 0.75);
	const double weight = 3;

	const auto& eval_pts = mScene->GetEvalPts();
	cDrawUtil::SetLineWidth(weight);

	int num_pts = static_cast<int>(eval_pts.size());
	for (int i = 0; i < (num_pts - 1); ++i)
	{
		const tVector& a = eval_pts[i];
		const tVector& b = eval_pts[i + 1];

		cDrawUtil::SetColor(col0);
		cDrawUtil::DrawLine(tVector(a[0], a[1], 0, 0), tVector(b[0], b[1], 0, 0));

		cDrawUtil::SetColor(col1);
		cDrawUtil::DrawLine(tVector(a[0], a[1] + 2 * a[2], 0, 0), tVector(b[0], b[1] + 2 * b[2], 0, 0));
		cDrawUtil::DrawLine(tVector(a[0], a[1] - 2 * a[2], 0, 0), tVector(b[0], b[1] - 2 * b[2], 0, 0));
	}
}