#pragma once
#include "util/MathUtil.h"

class cPenaltyGround
{
public:
	struct tPenaltyBox
	{
		double mStart;
		double mSize;
		bool mHit;

		tPenaltyBox();
	};

	struct tParams
	{
		int mNumBoxes;

		double mHeight;
		double mMinSpacing;
		double mMaxSpacing;
		double mMinBoxSize;
		double mMaxBoxSize;

		double mSpacingProb1;
		double mMinSpacing1;
		double mMaxSpacing1;

		tParams();
	};

	cPenaltyGround();
	virtual ~cPenaltyGround();

	virtual void Init(const tParams& params);
	virtual void Update(double bound_min, double bound_max);
	virtual void Reset();
	virtual double GetHeight() const;
	virtual double SampleHeight(const tVector& pos) const;

	virtual int GetNumBoxes() const;
	virtual const tPenaltyBox& GetBox(int i) const;

	virtual int CheckPenaltyContact(const tVector& pos) const;
	virtual void SetHit(int i, bool hit);

	virtual int GetBoxCount() const;
	virtual int GetHitCount() const;
	virtual double GetSuccessRate() const;

protected:
	tParams mParams;

	int mBoxCount;
	int mHitCount;

	std::vector<tPenaltyBox> mBoxes;

	virtual void InitBoxes();
	virtual void BuildRandBox(tPenaltyBox& out_box);
	virtual bool CheckContact(const tPenaltyBox& box, const tVector& pos) const;
	virtual double FindMaxPos() const;

	virtual double CalcSpacing() const;
};