#include "DrawScenarioBallRLMACE.h"
#include "render/DrawUtil.h"

const tVector gActionCols[] = {
	tVector(0, 0, 1, 0.2),
	tVector(1, 0, 0, 0.2),
	tVector(0, 0.5, 0, 0.2),
	tVector(0, 0, 0, 0.2),
};
const int gNumActionCols = sizeof(gActionCols) / sizeof(gActionCols[0]);

cDrawScenarioBallRLMACE::cDrawScenarioBallRLMACE(cCamera& cam)
	: cDrawScenarioBallRL(cam)
{
}

cDrawScenarioBallRLMACE::~cDrawScenarioBallRLMACE()
{
}

void cDrawScenarioBallRLMACE::BuildScene()
{
	mScene = std::shared_ptr<cScenarioBallRLMACE>(new cScenarioBallRLMACE());
}

void cDrawScenarioBallRLMACE::UpdateTrace()
{
	tVector pos = mScene->GetBallPos();
	std::shared_ptr<cBallControllerMACE> ctrl = std::static_pointer_cast<cBallControllerMACE>(mScene->GetBall().GetController());
	int a_id = ctrl->GetCurrAction().mID;
	pos[3] = a_id;

	mTraceBuffer.Add(pos);
}

void cDrawScenarioBallRLMACE::DrawTrace() const
{
	cDrawUtil::SetLineWidth(3);

	int num_trace = static_cast<int>(mTraceBuffer.GetSize());
	for (int i = 0; i < num_trace - 1; ++i)
	{
		const tVector& a = mTraceBuffer[i];
		const tVector& b = mTraceBuffer[i + 1];

		int a_id = static_cast<int>(a[3]);

		tVector col = GetActionCol(a_id);
		cDrawUtil::SetColor(col);
		cDrawUtil::DrawLine(a, b);
	}
}

const tVector& cDrawScenarioBallRLMACE::GetActionCol(int a_id) const
{
	int idx = a_id % gNumActionCols;
	return gActionCols[idx];
}