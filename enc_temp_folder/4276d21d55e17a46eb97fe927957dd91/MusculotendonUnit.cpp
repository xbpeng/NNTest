#include "MusculotendonUnit.h"
#include "SimCharacter.h"
#include "util\JsonUtil.h"

const std::string gOptCELengthKey = "OptCELength";
const std::string gSlackLengthKey = "SlackLength";
const std::string gForceMaxKey = "ForceMax";
const std::string gAttachPtsKey = "AttachPts";
const std::string gJointIDKey = "JointID";
const std::string gLocalPosKey = "LocalPos";

cMusculotendonUnit::tAttachPt::tAttachPt()
{
	mJointID = gInvalidIdx;
	mLocalPos.setZero();
}

bool cMusculotendonUnit::tAttachPt::Parse(const Json::Value& json)
{
	bool succ = true;
	mJointID = json[gJointIDKey].asInt();
	cJsonUtil::ReadVectorJson(json[gLocalPosKey], mLocalPos);
	return succ;
}

cMusculotendonUnit::tParams::tParams()
{
	Clear();
}

void cMusculotendonUnit::tParams::Clear()
{
	mOptCELength = 0;
	mSlackLength = 0;
	mForceMax = 0;
	mAttachPts.clear();
}

bool cMusculotendonUnit::ParseParams(const Json::Value& json, tParams& out_params)
{
	bool succ = !json.isNull();
	if (succ)
	{
		out_params.Clear();
		out_params.mOptCELength = json[gOptCELengthKey].asDouble();
		out_params.mSlackLength = json[gSlackLengthKey].asDouble();
		out_params.mForceMax = json[gForceMaxKey].asDouble();

		const Json::Value& attach_pt_arr = json[gAttachPtsKey];
		int num_pts = attach_pt_arr.size();
		for (int i = 0; i < num_pts; ++i)
		{
			const Json::Value& attach_pt_json = attach_pt_arr.get(i, 0);
			tAttachPt attach_pt;
			bool pt_succ = attach_pt.Parse(attach_pt_json);

			if (pt_succ)
			{
				out_params.mAttachPts.push_back(attach_pt);
			}
			else
			{
				break;
				succ = false;
			}
		}
		out_params.mAttachPts;
	}

	if (!succ)
	{
		out_params.Clear();
	}

	return succ;
}

cMusculotendonUnit::cMusculotendonUnit()
	: cController()
{
	Clear();
	mActivationRate = 0.01;
}

cMusculotendonUnit::~cMusculotendonUnit()
{
}

void cMusculotendonUnit::Init(cSimCharacter* character, const tParams& params)
{
	cController::Init(character);
	mValid = true;
	mParams = params;
	ResetParams();

	SetupSegActuations(mSegActuations);
}

void cMusculotendonUnit::Clear()
{
	cController::Clear();
	ResetParams();
	mParams.Clear();
	mSegActuations.clear();
}

void cMusculotendonUnit::Reset()
{
	cController::Reset();
	ResetParams();
}

void cMusculotendonUnit::Update(double time_step)
{
	double max_vel = -10;
	if (IsActive())
	{
		double da = time_step * mActivationRate;
		mActivation = (1 - da) * mActivation + da * mExcitation;
		mCELength += time_step * mCEVel;

		double len = CalcLength();
		double serial_len = len - mCELength;

		double F_se = CalcForceSerial(serial_len);
		double F_pe = CalcForceParallel(mCELength);
		
		double fl = (1 / 0.56) * (mCELength / mParams.mOptCELength - 1);
		fl = std::abs(fl);
		fl = fl * fl * fl;
		fl *= std::log(0.05);
		fl = std::exp(fl);

		double fv = (F_se - F_pe) / (mActivation * fl);
		double v_ce0 = (max_vel * (fv - 1)) / (-5 * fv - 1);
		double v_ce1 = (2 * (fv - 1.5) + 1) * max_vel / (2 * (fv - 1.5) * (7.56 * 5) - 1);

		mCEVel = (fv < 1) ? v_ce0 : v_ce1;
		mCEVel *= mParams.mOptCELength;
		mForce = F_se * mParams.mForceMax;

		ApplyForce(mForce);
	}
}

/*
void cMusculotendonUnit::Update(double time_step)
{
	double max_vel = -10;
	if (IsActive())
	{
		double da = time_step * mActivationRate;
		mActivation = (1 - da) * mActivation + da * mExcitation;
		mCELength += time_step * mCEVel;

		double len = CalcLength();
		double serial_len = len - mCELength;

		double F_se = CalcForceSerial(serial_len);
		double F_pe = CalcForceParallel(mCELength);

		double fl = (1 / 0.56) * (mCELength / mParams.mOptCELength - 1);
		fl = std::abs(fl);
		fl = fl * fl * fl;
		fl *= std::log(0.05);
		fl = std::exp(fl);

		//double fv = (F_se - F_pe) / (mActivation * fl);
		double f_pe_approx = (1 / 0.56) * (mCELength / mParams.mOptCELength - 1);
		f_pe_approx *= f_pe_approx;
		double fv = (F_se) / (mActivation * fl + f_pe_approx);
		double v_ce0 = (max_vel * (fv - 1)) / (-5 * fv - 1);
		double v_ce1 = (2 * (fv - 1.5) + 1) * max_vel / (2 * (fv - 1.5) * (7.56 * 5) - 1);

		mCEVel = (fv < 0) ? v_ce0 : v_ce1;
		mForce = F_se * mParams.mForceMax;

		ApplyForce(mForce);
	}
}
*/

int cMusculotendonUnit::GetNumAttachPts() const
{
	return static_cast<int>(mParams.mAttachPts.size());
}

const cMusculotendonUnit::tAttachPt& cMusculotendonUnit::GetAttachPt(int id) const
{
	assert(id >= 0 && id < GetNumAttachPts());
	return mParams.mAttachPts[id];
}

tVector cMusculotendonUnit::CalcAttachPtWorldPos(int id) const
{
	const tAttachPt& pt = GetAttachPt(id);
	const cJoint& joint = mChar->GetJoint(pt.mJointID);
	tVector pos = joint.CalcWorldPos(pt.mLocalPos);
	return pos;
}

void cMusculotendonUnit::SetExcitation(double u)
{
	mExcitation = u;
}

double cMusculotendonUnit::GetExcitation() const
{
	return mExcitation;
}

void cMusculotendonUnit::ResetParams()
{
	mActivation = 0.01;
	mExcitation = 0;
	mCEVel = 0;
	mCELength = mParams.mOptCELength;
	mForce = 0;
}

int cMusculotendonUnit::GetNumSegs() const
{
	return GetNumAttachPts() - 1;
}

void cMusculotendonUnit::SetupSegActuations(std::vector<tSegActuation>& out_actuations) const
{
	out_actuations.clear();
	int num_pts = GetNumAttachPts();
	if (num_pts > 1)
	{
		const auto& joint_mat = mChar->GetJointMat();

		int prev_joint = GetAttachPt(0).mJointID;
		for (int i = 1; i < num_pts; ++i)
		{
			const tAttachPt& pt = GetAttachPt(i);
			int curr_joint = pt.mJointID;

			tSegActuation curr_actuation;
			curr_actuation.mSegID = i - 1;

			assert(curr_joint <= curr_joint); // joints need to be ordered parent before child
			while (curr_joint > prev_joint)
			{
				curr_actuation.mJointIDs.push_back(curr_joint);
				curr_joint = cKinTree::GetParent(joint_mat, curr_joint);
			}

			if (curr_actuation.mJointIDs.size() > 0)
			{
				out_actuations.push_back(curr_actuation);
			}
			prev_joint = pt.mJointID;
		}
	}
}

double cMusculotendonUnit::CalcLength() const
{
	double len = 0;
	int num_pts = GetNumAttachPts();
	if (num_pts > 1)
	{
		tVector prev_pos = CalcAttachPtWorldPos(0);
		for (int i = 1; i < num_pts; ++i)
		{
			tVector curr_pos = CalcAttachPtWorldPos(i);
			len += (curr_pos - prev_pos).norm();
			prev_pos = curr_pos;
		}
	}
	return len;
}

double cMusculotendonUnit::GetRestLength() const
{
	double rest_len = mParams.mOptCELength + mParams.mSlackLength;
	return rest_len;
}

double cMusculotendonUnit::CalcForceSerial(double serial_len) const
{
	double strain = serial_len / mParams.mSlackLength - 1;
	double ref_strain = 0.04;
	double f_se = strain / ref_strain;
	f_se = std::max(f_se, 0.0);
	f_se *= f_se;
	//f_se *= mParams.mForceMax; // mForceMax gets divided out later so skip mult
	return f_se;
}

double cMusculotendonUnit::CalcForceParallel(double parallel_len) const
{
	double norm_len = parallel_len / mParams.mOptCELength;
	double hpe = (1 / 0.56) * (norm_len - 1);
	double lpe = (1 / 0.28) * (0.44 - norm_len);

	hpe = std::max(hpe, 0.0);
	lpe = std::max(lpe, 0.0);
	hpe *= hpe;
	lpe *= lpe;

	double f_pe = hpe - lpe;
	//f_pe *= mParams.mForceMax;
	return f_pe;
}

void cMusculotendonUnit::ApplyForce(double force)
{

}