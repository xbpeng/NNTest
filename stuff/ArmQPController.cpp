#include "ArmQPController.h"
#include "SimArm.h"

cArmQPController::cArmQPController()
{
	mGravity = gGravity;
	mReach = 1;
}

cArmQPController::~cArmQPController()
{
}

void cArmQPController::Init(cSimCharacter* character, const tVector& gravity)
{
	cArmController::Init(character);
	mGravity = gravity;
	mRBDModel.Init(mChar->GetJointMat(), mChar->GetBodyDefs(), mGravity);
	InitQP();

	mUpdateCounter = mUpdatePeriod;
	mReach = CalcReach();
}

void cArmQPController::SetTorqueLimit(double torque_lim)
{
	cArmController::SetTorqueLimit(torque_lim);
	SetupQPTorqueLim(torque_lim);
}

void cArmQPController::UpdateRBDModel()
{
	Eigen::VectorXd pose;
	Eigen::VectorXd vel;
	mChar->BuildPose(pose);
	mChar->BuildVel(vel);

	mRBDModel.Update(pose, vel);
	cRBDUtil::BuildJacobian(mRBDModel, mJacobian);
	cRBDUtil::BuildJacobianDot(mRBDModel, mJacobianDot);
}

void cArmQPController::UpdatePoliAction()
{
	UpdateRBDModel();
	ComputeQPTau();

	int tau_offset = GetQPParamOffset(eQPParamTau);
	int tau_size = GetQPParamSize(eQPParamTau);
	const auto& tau = mSoln.mX.segment(tau_offset, tau_size);

	int root_size = mChar->GetParamSize(mChar->GetRootID());
	mPoliAction = tau.segment(root_size, tau_size - root_size);
	mPoliAction *= gTorqueScale;
}

void cArmQPController::ComputeQPTau()
{
	BuildQPProb(mTargetPos, mProb);
	int fail = cQuadProg::Solve(mProb, mSoln);
	assert(fail == 0);
}

void cArmQPController::InitQP()
{
	int n = CalcQPDim();
	mProb = cQuadProg::tProb(n);
	mSoln.mX = mProb.mInitX;

	SetupQPTorqueLim(mTorqueLim);
}

void cArmQPController::SetupQPTorqueLim(double torque_lim)
{
	int tau_offset = GetQPParamOffset(eQPParamTau);
	int tau_size = GetQPParamSize(eQPParamTau);

	// non-neg constraint for contact forces
	mProb.mLower.segment(tau_offset, tau_size) = -torque_lim * Eigen::VectorXd::Ones(tau_size);
	mProb.mUpper.segment(tau_offset, tau_size) = torque_lim * Eigen::VectorXd::Ones(tau_size);
}

void cArmQPController::BuildQPProb(const tVector& end_target, cQuadProg::tProb& out_prob)
{
	Eigen::VectorXd pose;
	Eigen::VectorXd vel;
	mChar->BuildPose(pose);
	mChar->BuildVel(vel);

	tVector clamped_target = end_target;
	tVector root_pos = mChar->GetRootPos();
	tVector delta = clamped_target - root_pos;
	double dist = delta.norm();
	if (dist > mReach)
	{
		delta *= mReach / dist;
		clamped_target = root_pos + delta;
	}

	BuildObjective(pose, vel, clamped_target, out_prob);
	BuildConstraints(pose, vel, out_prob);
}

void cArmQPController::BuildObjective(const Eigen::VectorXd& pose, const Eigen::VectorXd& vel, const tVector& end_target, cQuadProg::tProb& out_prob)
{
	int n = CalcQPDim();
	assert(out_prob.mC.rows() == n);
	assert(out_prob.mC.cols() == n);
	assert(out_prob.md.size() == n);
	out_prob.mC.setZero();
	out_prob.md.setZero();

	// pose objective
	{
		double w = 0.5;
		Eigen::MatrixXd curr_C = Eigen::MatrixXd::Zero(n, n);
		Eigen::VectorXd curr_d = Eigen::VectorXd::Zero(n);
		BuildObjectivePose(pose, vel, curr_C, curr_d);
		out_prob.mC += w * curr_C;
		out_prob.md += w * curr_d;
	}

	// end effector objective
	{
		double w = 5;
		Eigen::MatrixXd curr_C = Eigen::MatrixXd::Zero(n, n);
		Eigen::VectorXd curr_d = Eigen::VectorXd::Zero(n);
		BuildObjectiveEndEffectors(pose, vel, end_target, curr_C, curr_d);
		out_prob.mC += w * curr_C;
		out_prob.md += w * curr_d;
	}

	// force objective
	{
		double w = 0.01;
		Eigen::MatrixXd curr_C = Eigen::MatrixXd::Zero(n, n);
		Eigen::VectorXd curr_d = Eigen::VectorXd::Zero(n);
		BuildObjectiveForce(pose, vel, curr_C, curr_d);
		out_prob.mC += w * curr_C;
		out_prob.md += w * curr_d;
	}
}

void cArmQPController::BuildConstraints(const Eigen::VectorXd& pose, const Eigen::VectorXd& vel, cQuadProg::tProb& out_prob) const
{
	std::vector<Eigen::MatrixXd> As;
	std::vector<Eigen::VectorXd> bs;

	Eigen::MatrixXd A_physics;
	Eigen::VectorXd b_physics;
	BuildConstraintsPhysics(pose, vel, A_physics, b_physics);

	out_prob.mA = A_physics;
	out_prob.mb = b_physics;
	out_prob.mNumEquality = static_cast<int>(A_physics.rows());
}

void cArmQPController::BuildConstraintsPhysics(const Eigen::VectorXd& pose, const Eigen::VectorXd& vel,
												Eigen::MatrixXd& out_A, Eigen::VectorXd& out_b) const
{
	const int acc_offset = GetQPParamOffset(eQPParamAcc);
	const int acc_size = GetQPParamSize(eQPParamAcc);
	const int tau_offset = GetQPParamOffset(eQPParamTau);
	const int tau_size = GetQPParamSize(eQPParamTau);
	
	int rows = tau_size;
	int cols = CalcQPDim();
	out_A = Eigen::MatrixXd::Zero(rows, cols);
	out_b = Eigen::VectorXd::Zero(rows);

	auto acc_block = out_A.block(0, acc_offset, rows, acc_size);
	auto tau_block = out_A.block(0, tau_offset, rows, tau_size);
	
	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();
	const Eigen::MatrixXd& body_defs = mChar->GetBodyDefs();

	acc_block = mRBDModel.GetMassMat();
	tau_block = -1 * Eigen::MatrixXd::Identity(tau_size, tau_size);
	
	out_b = mRBDModel.GetBiasForce();
	out_b -= BuildTotalBodyForce();
}

Eigen::VectorXd cArmQPController::BuildTotalBodyForce() const
{
	int num_dof = mChar->GetNumDof();
	Eigen::VectorXd total_force = Eigen::VectorXd::Zero(num_dof);
	return total_force;
}

void cArmQPController::BuildObjectivePose(const Eigen::VectorXd& pose, const Eigen::VectorXd& vel,
											Eigen::MatrixXd& out_C, Eigen::VectorXd& out_d) const
{
	//const double omega_p = 10;
	//const double si_p = 4;

	const double omega_p = 5;
	const double si_p = 5;
	
	int acc_offset = GetQPParamOffset(eQPParamAcc);
	int acc_size = GetQPParamSize(eQPParamAcc);

	auto d_acc = out_d.segment(acc_offset, acc_size);
	auto C_acc = out_C.block(acc_offset, acc_offset, acc_size, acc_size);
	C_acc.setIdentity();

	Eigen::VectorXd pose_ref = Eigen::VectorXd::Zero(pose.size());
	Eigen::VectorXd vel_ref = Eigen::VectorXd::Zero(vel.size());

	Eigen::VectorXd pose_err = pose_ref - pose;
	Eigen::VectorXd vel_err = vel_ref - vel;

	int first_id = 1;
	int first_offset = mChar->GetParamOffset(first_id);
	int first_size = mChar->GetParamSize(first_id);
	pose_err.segment(first_offset, first_size).setZero();

	d_acc = -omega_p * si_p * vel_err - omega_p * omega_p * pose_err;
	d_acc = C_acc * d_acc;
}

void cArmQPController::BuildObjectiveEndEffectors(const Eigen::VectorXd& pose, const Eigen::VectorXd& vel,
												const tVector& target_pos, Eigen::MatrixXd& out_C, Eigen::VectorXd& out_d) const
{
	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();

	const Eigen::MatrixXd& J = mJacobian;
	const Eigen::MatrixXd& Jd = mJacobianDot;

	int acc_offset = GetQPParamOffset(eQPParamAcc);
	int acc_size = GetQPParamSize(eQPParamAcc);
	auto C_acc = out_C.block(acc_offset, acc_offset, acc_size, acc_size);
	auto d_acc = out_d.segment(acc_offset, acc_size);

	C_acc.setZero();
	d_acc.setZero();

	// ignore angualr components
	const Eigen::MatrixXd W = cSpAlg::BuildSV(tVector::Zero(), tVector::Ones()).asDiagonal();
	const double omega = 20;
	const double si = 2;

	int joint_id = GetEndEffectorID();
	const tVector end_pos_ref = target_pos;
	const cSpAlg::tSpVec target_vel = cSpAlg::tSpVec::Zero();

	tVector end_pos = cKinTree::CalcJointWorldPos(joint_mat, pose, joint_id);

	double weight = 1;
	Eigen::MatrixXd curr_W = W;

	// everything should be in a coordinate frame with origin at the end effector
	// and axes aligned with the world fram
	cSpAlg::tSpTrans trans = cSpAlg::BuildTrans(end_pos);
	cSpAlg::tSpVec end_vel = cRBDUtil::MultJacobianEndEff(joint_mat, vel, J, joint_id);
	end_vel = cSpAlg::ApplyTransM(trans, end_vel);

	// pad positioon out to be 6d
	cSpAlg::tSpVec pos_err = cSpAlg::BuildSV(tVector::Zero(), end_pos_ref - end_pos);
	cSpAlg::tSpVec vel_err = target_vel - end_vel;

	Eigen::MatrixXd J_end_eff = cRBDUtil::ExtractEndEffJacobian(joint_mat, J, joint_id);
	Eigen::MatrixXd J_local = cSpAlg::BuildSpatialMatM(trans) * J_end_eff;
	cSpAlg::tSpVec b = -omega * si * vel_err - omega * omega * pos_err;

	C_acc += J_local.transpose() * weight * curr_W * J_local;
	d_acc += J_local.transpose() * weight * curr_W * b;
}

void cArmQPController::BuildObjectiveForce(const Eigen::VectorXd& pose, const Eigen::VectorXd& vel,
											Eigen::MatrixXd& out_C, Eigen::VectorXd& out_d) const
{
	int tau_offset = GetQPParamOffset(eQPParamTau);
	int tau_size = GetQPParamSize(eQPParamTau);

	auto C_tau = out_C.block(tau_offset, tau_offset, tau_size, tau_size);
	C_tau.setIdentity();

	auto d_tau = out_d.segment(tau_offset, tau_size);
	d_tau.setZero();
	d_tau = C_tau * d_tau;
}

int cArmQPController::CalcQPDim() const
{
	int dim = 0;
	for (int i = 0; i < eQPParamMax; ++i)
	{
		dim += GetQPParamSize(static_cast<eQPParam>(i));
	}
	return dim;
}

int cArmQPController::GetQPParamOffset(eQPParam param) const
{
	int offset = -1;
	int num_dofs = mChar->GetNumDof();
	switch (param)
	{
	case eQPParamAcc:
		offset = 0;
		break;
	case eQPParamTau:
		offset = num_dofs;
		break;
	default:
		assert(false); // unsupported QP param
		break;
	}
	return offset;
}

int cArmQPController::GetQPParamSize(eQPParam param) const
{
	int size = -1;
	int num_dofs = mChar->GetNumDof();
	switch (param)
	{
	case eQPParamAcc:
		size = num_dofs;
		break;
	case eQPParamTau:
		size = num_dofs;
		break;
	default:
		assert(false); // unsupported QP param
		break;
	}
	return size;
}

double cArmQPController::CalcReach() const
{
	double reach = 0;
	const auto& joint_mat = mChar->GetJointMat();
	for (int j = 0; j < mChar->GetNumJoints(); ++j)
	{
		double len = cKinTree::CalcLinkLength(joint_mat, j);
		reach += len;
	}
	return reach;
}