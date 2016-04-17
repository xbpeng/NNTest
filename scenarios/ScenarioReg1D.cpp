#include "ScenarioReg1D.h"
#include "util/FileUtil.h"

const std::string gPointsKey = "Points";

cScenarioReg1D::cScenarioReg1D()
{
	Clear();
}

cScenarioReg1D::~cScenarioReg1D()
{
}

void cScenarioReg1D::Init()
{
	InitTrainer();
	if (mInputFile != "")
	{
		LoadPoints(mInputFile);
	}
}

void cScenarioReg1D::ParseArgs(const cArgParser& parser)
{
	parser.ParseString("solver_file", mSolverFile);
	parser.ParseString("net_file", mNetFile);
	parser.ParseString("input_file", mInputFile);
	parser.ParseString("output_file", mOutputFile);

	parser.ParseInt("num_evals_pts", mNumEvalPts);
	parser.ParseInt("pases_per_step", mPassesPerStep);
}

void cScenarioReg1D::Reset()
{
	mTrainer->Reset();
	mEvalPts.clear();
	mPts.clear();
	
	if (mInputFile != "")
	{
		LoadPoints(mInputFile);
	}
}

void cScenarioReg1D::Clear()
{
	mSolverFile = "";
	mNetFile = "";
	mPts.clear();
	mTrainer.reset();
	mEvalPts.clear();
	mNumEvalPts = 100;
	mPassesPerStep = 100;
}

void cScenarioReg1D::Update(double time_elapsed)
{
}

int cScenarioReg1D::GetNumPts() const
{
	return static_cast<int>(mPts.size());
}

const tVector& cScenarioReg1D::GetPt(int i) const
{
	return mPts[i];
}

void cScenarioReg1D::AddPt(const tVector& pt)
{
	mPts.push_back(pt);
	tExpTuple tuple;
	BuildTuple(pt, tuple);
	mTrainer->AddTuple(tuple);
}

const std::vector<tVector, Eigen::aligned_allocator<tVector>>& cScenarioReg1D::GetEvalPts() const
{
	return mEvalPts;
}

void cScenarioReg1D::TrainNet()
{
	for (int i = 0; i < mPassesPerStep; ++i)
	{
		mTrainer->Train();
	}
	
	EvalNet();
}

void cScenarioReg1D::LoadPoints(const std::string& filename)
{
	std::ifstream f_stream(filename);
	Json::Reader reader;
	Json::Value root;
	bool succ = reader.parse(f_stream, root);
	f_stream.close();

	if (succ)
	{
		const Json::Value& points_json = root[gPointsKey];
		succ = ParsePoints(points_json, mPts);
	}
	
	if (!succ)
	{
		printf("Failed to parse %s\n", filename.c_str());
		assert(false);
	}
}

void cScenarioReg1D::OutputPoints(const std::string& filename) const
{
	FILE* f = cFileUtil::OpenFile(filename, "w");
	if (f != nullptr)
	{
		fprintf(f,"{\n\"%s\":[\n", gPointsKey.c_str());
		for (int i = 0; i < GetNumPts(); ++i)
		{
			if (i != 0)
			{
				fprintf(f, ",\n");
			}
			const tVector& pt = GetPt(i);
			std::string json = BuildPtJson(pt);
			fprintf(f, "%s", json.c_str());
		}
		fprintf(f, "\n]}");
		cFileUtil::CloseFile(f);
	}
	else
	{
		printf("Failed to write data to %s\n", filename.c_str());
		assert(false);
	}
}

void cScenarioReg1D::OutputPoints() const
{
	if (mOutputFile != "")
	{
		OutputPoints(mOutputFile);
	}
	else
	{
		printf("No output file specified\n");
		assert(false);
	}
}

std::string cScenarioReg1D::GetName() const
{
	return "Regression 1D";
}

void cScenarioReg1D::InitTrainer()
{
	BuildTrainer(mTrainer);

	cNeuralNetTrainer::tParams trainer_params;
	trainer_params.mNetFile = mNetFile;
	trainer_params.mSolverFile = mSolverFile;
	trainer_params.mPlaybackMemSize = 10000;
	trainer_params.mPoolSize = 1;
	trainer_params.mNumInitSamples = 0;
	trainer_params.mInitInputOffsetScale = false;

	mTrainer->Init(trainer_params);
	SetupScale();
}

void cScenarioReg1D::BuildTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer)
{
	out_trainer = std::shared_ptr<cNeuralNetTrainer>(new cNeuralNetTrainer());
}

void cScenarioReg1D::SetupScale()
{
	int input_size = mTrainer->GetInputSize();
	int output_size = mTrainer->GetOutputSize();
	
	Eigen::VectorXd offset = Eigen::VectorXd::Zero(input_size);
	Eigen::VectorXd scale = Eigen::VectorXd::Ones(input_size);
	mTrainer->SetInputOffsetScale(offset, scale);

	Eigen::VectorXd output_offset = Eigen::VectorXd::Zero(output_size);
	Eigen::VectorXd output_scale = Eigen::VectorXd::Ones(output_size);
	mTrainer->SetOutputOffsetScale(output_offset, output_scale);
}

void cScenarioReg1D::BuildTuple(const tVector& pt, tExpTuple& out_tuple) const
{
	int state_size = mTrainer->GetStateSize();
	int action_size = mTrainer->GetActionSize();

	out_tuple.mStateBeg.resize(state_size);
	out_tuple.mAction.resize(action_size);

	out_tuple.mStateBeg[0] = pt[0];
	out_tuple.mAction[0] = pt[1];
	out_tuple.mStateEnd = out_tuple.mStateBeg;
}

tVector cScenarioReg1D::BuildPt(const Eigen::VectorXd& x, const Eigen::VectorXd& y) const
{
	return tVector(x[0], y[0], 0, 0);
}

void cScenarioReg1D::EvalNet()
{
	const double pad = 0.2;
	if (GetNumPts() > 0)
	{
		double min_x = 0;
		double max_x = 0;
		FindMinMaxX(min_x, max_x);
		min_x -= pad;
		max_x += pad;

		mEvalPts.resize(mNumEvalPts);

		const auto& net = GetNet();
		Eigen::VectorXd x = Eigen::VectorXd::Zero(net->GetInputSize());
		Eigen::VectorXd y = Eigen::VectorXd::Zero(net->GetOutputSize());

		for (int i = 0; i < mNumEvalPts; ++i)
		{
			x(0) = static_cast<double>(i) / (mNumEvalPts - 1) * (max_x - min_x) + min_x;
			net->Eval(x, y);
			mEvalPts[i] = BuildPt(x, y);
		}
	}
	else
	{
		mEvalPts.clear();
	}
}

void cScenarioReg1D::FindMinMaxX(double& out_min_x, double& out_max_x) const
{
	int num_pts = GetNumPts();
	if (num_pts > 0)
	{
		out_min_x = std::numeric_limits<double>::infinity();
		out_max_x = -std::numeric_limits<double>::infinity();

		for (int i = 0; i < num_pts; ++i)
		{
			const tVector& pt = GetPt(i);
			out_min_x = std::min(pt(0), out_min_x);
			out_max_x = std::max(pt(0), out_max_x);
		}
	}
	else
	{
		out_min_x = 0;
		out_max_x = 0;
	}
}

const std::unique_ptr<cNeuralNet>& cScenarioReg1D::GetNet() const
{
	return mTrainer->GetNet();
}

std::string cScenarioReg1D::BuildPtJson(const tVector& pt) const
{
	return cJsonUtil::BuildVectorJson(pt);
}

bool cScenarioReg1D::ParsePoints(const Json::Value& root, tVectorArr& out_points) const
{
	bool succ = true;
	if (!root.isNull() && root.isArray())
	{
		int num_vals = root.size();
		out_points.clear();
		out_points.reserve(num_vals);

		for (int i = 0; i < num_vals; ++i)
		{
			const Json::Value& pt_json = root.get(i, 0);;
			tVector point;
			cJsonUtil::ReadVectorJson(pt_json, point);
			out_points.push_back(point);
		}
	}
	else
	{
		succ = false;
	}
	return succ;
}