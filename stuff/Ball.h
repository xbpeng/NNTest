#pragma once
#include "util/MathUtil.h"
#include "BallController.h"

class cBall
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cBall();
	virtual ~cBall();

	virtual void Update(double time_step);
	virtual void Reset();
	virtual void SetController(const std::shared_ptr<cBallController>& ctrl);

	virtual const tVector& GetPos() const;
	virtual void SetPos(const tVector& pos);
	virtual double GetRadius() const;

	virtual bool IsNewCycle() const;
	virtual std::shared_ptr<cBallController>& GetController();
	virtual const std::shared_ptr<cBallController>& GetController() const;

protected:
	tVector mPos;
	double mRadius;
	std::shared_ptr<cBallController> mCtrl;
};