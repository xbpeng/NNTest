#pragma once

#include "ArmController.h"
#include "sim/RBDUtil.h"
#include "opt/QuadProg.h"

class cArmQPController : public cArmController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cArmQPController();
	virtual ~cArmQPController();

	virtual void Init(cSimCharacter* character, const tVector& gravity);
	virtual void SetTargetPos(const tVector& target);

protected:
	enum eQPParam
	{
		eQPParamAcc,
		eQPParamTau,
		eQPParamMax
	};

	tVector mGravity;
	tVector mEndTarget;
	double mReach;

	cRBDModel mRBDModel;
	Eigen::MatrixXd mJacobian;
	Eigen::MatrixXd mJacobianDot;
	cQuadProg::tProb mProb;
	cQuadProg::tSoln mSoln;

	virtual void UpdateRBDModel();
	virtual void UpdatePoliAction();
	virtual void ComputeQPTau();
	virtual void InitQP();
	virtual void SetupQPTorqueLim(double torque_lim);

	virtual void BuildQPProb(const tVector& end_target, cQuadProg::tProb& out_prob);
	virtual void BuildObjective(const Eigen::VectorXd& pose, const Eigen::VectorXd& vel, const tVector& end_target, cQuadProg::tProb& out_prob);
	virtual void BuildConstraints(const Eigen::VectorXd& pose, const Eigen::VectorXd& vel, cQuadProg::tProb& out_prob) const;
	virtual void BuildConstraintsPhysics(const Eigen::VectorXd& pose, const Eigen::VectorXd& vel,
										Eigen::MatrixXd& out_A, Eigen::VectorXd& out_b) const;
	virtual Eigen::VectorXd BuildTotalBodyForce() const;
	virtual void BuildObjectiveEndEffectors(const Eigen::VectorXd& pose, const Eigen::VectorXd& vel,
											const tVector& target_pos, Eigen::MatrixXd& out_C, Eigen::VectorXd& out_d) const;
	virtual void BuildObjectivePose(const Eigen::VectorXd& pose, const Eigen::VectorXd& vel,
											Eigen::MatrixXd& out_C, Eigen::VectorXd& out_d) const;
	virtual void BuildObjectiveForce(const Eigen::VectorXd& pose, const Eigen::VectorXd& vel,
											Eigen::MatrixXd& out_C, Eigen::VectorXd& out_d) const;

	virtual int CalcQPDim() const;
	virtual int GetQPParamOffset(eQPParam param) const;
	virtual int GetQPParamSize(eQPParam param) const;

	virtual int GetEndEffectorID() const;
	virtual double CalcReach() const;
};