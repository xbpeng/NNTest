#include "PenaltyGround.h"

cPenaltyGround::tPenaltyBox::tPenaltyBox()
{
	mStart = 0;
	mSize = 0;
	mHit = false;
}

cPenaltyGround::tParams::tParams()
{
	mHeight = 0;

	mMinSpacing = 0.2;
	mMaxSpacing = 2.0;
	mMinBoxSize = 0.3;
	mMaxBoxSize = 2.0;
	mNumBoxes = 16;

	mSpacingProb1 = 0;
	mMinSpacing1 = 0.2;
	mMaxSpacing1 = 2.0;
}

cPenaltyGround::cPenaltyGround()
{
}

cPenaltyGround::~cPenaltyGround()
{

}

double cPenaltyGround::GetHeight() const
{
	return mParams.mHeight;
}

void cPenaltyGround::Init(const tParams& params)
{
	mParams = params;
	Reset();
}

void cPenaltyGround::Update(double bound_min, double bound_max)
{
	for (int i = 0; i < GetNumBoxes(); ++i)
	{
		tPenaltyBox& box = mBoxes[i];
		if (box.mStart + box.mSize < bound_min)
		{
			double max_pos = FindMaxPos();

			++mBoxCount;

			if (box.mHit)
			{
				++mHitCount;
			}

			BuildRandBox(box);
			box.mStart += max_pos;
		}
	}
}

void cPenaltyGround::Reset()
{
	InitBoxes();
	mBoxCount = 0;
	mHitCount = 0;
}

double cPenaltyGround::SampleHeight(const tVector& pos) const
{
	int contact_idx = CheckPenaltyContact(pos);
	bool contact = contact_idx != -1;
	double h = mParams.mHeight;
	if (contact)
	{
		h -= 1;
	}
	return h;
}


int cPenaltyGround::GetNumBoxes() const
{
	return static_cast<int>(mBoxes.size());
}

const cPenaltyGround::tPenaltyBox& cPenaltyGround::GetBox(int i) const
{
	return mBoxes[i];
}

int cPenaltyGround::CheckPenaltyContact(const tVector& pos) const
{
	int idx = -1;
	for (int i = 0; i < GetNumBoxes(); ++i)
	{
		bool contact = CheckContact(mBoxes[i], pos);
		if (contact)
		{
			idx = i;
			break;
		}
	}
	return idx;
}

void cPenaltyGround::SetHit(int i, bool hit)
{
	mBoxes[i].mHit = hit;
}

int cPenaltyGround::GetBoxCount() const
{
	return mBoxCount;
}

int cPenaltyGround::GetHitCount() const
{
	return mHitCount;
}

double cPenaltyGround::GetSuccessRate() const
{
	double succ_rate = 0;
	if (mBoxCount > 0)
	{
		succ_rate = 1 - static_cast<double>(mHitCount) / mBoxCount;
	}
	return succ_rate;
}

void cPenaltyGround::InitBoxes()
{
	int num_boxes = mParams.mNumBoxes;
	mBoxes.resize(num_boxes);

	double curr_x = 0;
	for (int i = 0; i < num_boxes; ++i)
	{
		tPenaltyBox& box = mBoxes[i];
		BuildRandBox(box);

		box.mStart += curr_x;
		curr_x = box.mStart + box.mSize;
	}
}

void cPenaltyGround::BuildRandBox(tPenaltyBox& out_box)
{
	out_box = tPenaltyBox();
	out_box.mStart = CalcSpacing();
	out_box.mSize = cMathUtil::RandDouble(mParams.mMinBoxSize, mParams.mMaxBoxSize);
}

bool cPenaltyGround::CheckContact(const tPenaltyBox& box, const tVector& pos) const
{
	return (pos[0] >= box.mStart) && (pos[0] <= box.mStart + box.mSize);
}

double cPenaltyGround::FindMaxPos() const
{
	double max_pos = -std::numeric_limits<double>::infinity();
	for (int i = 0; i < GetNumBoxes(); ++i)
	{
		const tPenaltyBox& box = mBoxes[i];
		max_pos = std::max(box.mStart + box.mSize, max_pos);
	}
	return max_pos;
}

double cPenaltyGround::CalcSpacing() const
{
	double rand = cMathUtil::RandDouble();
	double d = 0;

	static int hack_count = 0;

	if (rand < mParams.mSpacingProb1 || hack_count > 2)
	{
		d = cMathUtil::RandDouble(mParams.mMinSpacing1, mParams.mMaxSpacing1);
		hack_count = 0;
	}
	else
	{
		d = cMathUtil::RandDouble(mParams.mMinSpacing, mParams.mMaxSpacing);
		++hack_count;
	}

	return d;  
}