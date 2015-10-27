#include "DrawBall.h"
#include "render/DrawUtil.h"

void cDrawBall::Draw(const cBall& ball, const tVector& col)
{
	const int slices = 16;
	tVector pos = ball.GetPos();
	double r = ball.GetRadius();
	pos[1] += r;

	glPushMatrix();
	cDrawUtil::Translate(pos);
	cDrawUtil::SetColor(col);
	cDrawUtil::DrawDisk(r, slices, cDrawUtil::eDrawSolid);

	cDrawUtil::SetLineWidth(1);
	cDrawUtil::SetColor(tVector(0, 0, 0, 1));
	cDrawUtil::DrawDisk(r, slices, cDrawUtil::eDrawWire);
	glPopMatrix();

	// draw samples
	const cBallController& ctrl = ball.GetController();
	const Eigen::VectorXd& samples = ctrl.GetGroundSamples();

	cDrawUtil::SetColor(tVector(1, 0, 0, 0.5));
	for (int i = 0; i < samples.size(); ++i)
	{
		tVector pos = ctrl.GetGroundSamplePos(i);
		pos[1] = samples[i];

		glPushMatrix();
		cDrawUtil::Translate(pos);
		cDrawUtil::DrawDisk(0.02, 16);
		glPopMatrix();
	}
}