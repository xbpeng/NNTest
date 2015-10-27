#include "Ball.h"

cBall::cBall() :
	mCtrl(*this)
{
	mPos.setZero();
	mRadius = 0.1;
}

cBall::~cBall()
{
}

const tVector& cBall::GetPos() const
{
	return mPos;
}

void cBall::SetPos(const tVector& pos)
{
	mPos = pos;
}

double cBall::GetRadius() const
{
	return mRadius;
}

bool cBall::IsNewCycle() const
{
	return mCtrl.IsNewCycle();
}

cBallController& cBall::GetController()
{
	return mCtrl;
}

const cBallController& cBall::GetController() const
{
	return mCtrl;
}

void cBall::Update(double time_step)
{
	mCtrl.Update(time_step);
}

void cBall::Reset()
{
	mPos.setZero();
	mCtrl.Reset();
}