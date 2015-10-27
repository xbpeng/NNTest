#include "DrawPenaltyGround.h"
#include "render/DrawUtil.h"

const double gMarkerSpacing = 0.20;
const double gBigMarkerSpacing = gMarkerSpacing * 5;
const double gMarkerH = 0.04;
const double gBigMarkerH = 0.075;

void cDrawPenaltyGround::Draw(const cPenaltyGround& ground, const tVector& bound_min, const tVector& bound_max)
{
	const double ground_h = ground.GetHeight();

	tVector origin = (bound_min + bound_max) * 0.5;
	double w = bound_max[0] - bound_min[0];
	double h = bound_max[1] - bound_min[1];

	double max_x = origin(0) + w;
	double max_y = ground_h;
	double min_x = origin(0) - w;
	double min_y = std::min(origin(1) - h * 0.5f, max_y - 0.05f);

	tVector pos = tVector(origin(0), (min_y + max_y) * 0.5, 0, 0);
	tVector size = tVector(w, (max_y - min_y), 0, 0);

	cDrawUtil::DrawRuler2D(pos, size, gMarkerSpacing, gBigMarkerSpacing, gMarkerH, gBigMarkerH);

	cDrawUtil::SetLineWidth(1);

	int num_boxes = ground.GetNumBoxes();
	for (int i = 0; i < num_boxes; ++i)
	{
		const cPenaltyGround::tPenaltyBox& box = ground.GetBox(i);
		tVector box_col = (box.mHit) ? tVector(0.8, 0, 0, 1) : tVector(0.3, 0.3, 0.3, 1);

		tVector box_pos = tVector(box.mStart + 0.5 * box.mSize, 0.5 * (max_y + min_y), 0, 0);
		tVector box_size = tVector(box.mSize, max_y - min_y, 0, 0);

		cDrawUtil::SetColor(box_col);
		cDrawUtil::DrawBox(box_pos, box_size, cDrawUtil::eDrawSolid);
		cDrawUtil::SetColor(tVector(0, 0, 0, 1));
		cDrawUtil::DrawBox(box_pos, box_size, cDrawUtil::eDrawWire);
	}
}