#include "ScenarioArmTrainDPG.h"
#include "learning/DPGTrainer.h"
#include "stuff/SimArm.h"
#include "render/DrawUtil.h"
#include "render/DrawSimCharacter.h"
#include "util/FileUtil.h"

const int gTrainerPlaybackMemSize = 25000;

cScenarioArmTrainDPG::cScenarioArmTrainDPG()
{
}

cScenarioArmTrainDPG::~cScenarioArmTrainDPG()
{
}

void cScenarioArmTrainDPG::Init()
{
	cScenarioArmTrain::Init();

	// hack
	auto trainer = std::static_pointer_cast<cDPGTrainer>(mTrainer);
	int num_samples = 200;
	double min_theta = -M_PI;
	double max_theta = M_PI;
	int state_size = trainer->GetStateSize();
	int action_size = trainer->GetActionSize();
	
	tExpTuple tuple;
	tuple.mStateBeg = Eigen::VectorXd::Zero(state_size);
	tuple.mAction = Eigen::VectorXd::Zero(action_size);
	tuple.mStateBeg[0] = 0.5;

	FILE* critic_f = cFileUtil::OpenFile("scripts/dpg_arm_plot/critic_vals.txt", "w");
	FILE* actor_f = cFileUtil::OpenFile("scripts/dpg_arm_plot/actor_data.txt", "w");
	FILE* dpg_f = cFileUtil::OpenFile("scripts/dpg_arm_plot/dpg_data.txt", "w");

	for (int i = 0; i < num_samples; ++i)
	{
		double curr_theta = (max_theta - min_theta) * i / (num_samples - 1.0) + min_theta;
		double chain_len = mChar->CalcJointChainLength(mChar->GetNumJoints() - 1);
		tMatrix rot_mat = cMathUtil::RotateMat(tVector(0, 0, 1, 0), curr_theta);
		tVector delta = rot_mat * tVector(chain_len, 0, 0, 0);

		//tuple.mStateBeg[2] = curr_theta;
		tuple.mStateBeg[4] = delta[0];
		tuple.mStateBeg[5] = delta[1];

		Eigen::VectorXd actor_y;
		trainer->EvalActor(tuple, actor_y);
		tuple.mAction = actor_y;

		Eigen::VectorXd critic_y;
		trainer->EvalCritic(tuple, critic_y);
		
		Eigen::VectorXd dpg_y;
		trainer->CalcDPG(tuple, dpg_y);

		fprintf(critic_f, "%.5f, %.5f\n", curr_theta, critic_y[0]);

		fprintf(actor_f, "%.5f", curr_theta);
		for (int j = 0; j < actor_y.size(); ++j)
		{
			fprintf(actor_f, ", %.5f", actor_y[j]);
		}
		fprintf(actor_f, "\n");

		fprintf(dpg_f, "%.5f", curr_theta);
		for (int j = 0; j < dpg_y.size(); ++j)
		{
			fprintf(dpg_f, ", %.5f", dpg_y[j]);
		}
		fprintf(dpg_f, "\n");
	}
	cFileUtil::CloseFile(actor_f);
	cFileUtil::CloseFile(dpg_f);
	cFileUtil::CloseFile(critic_f);
}

std::string cScenarioArmTrainDPG::GetName() const
{
	return "Arm Train DPG";
}

void cScenarioArmTrainDPG::RecordFlagsEnd(tExpTuple& out_tuple) const
{
	bool fail = CheckFail();
	out_tuple.SetFlag(fail, cDPGTrainer::eFlagFail);
}

void cScenarioArmTrainDPG::InitTrainer()
{
	//cScenarioArmTrain::InitTrainer();

	mTrainerParams.mPlaybackMemSize = gTrainerPlaybackMemSize;
	mTrainerParams.mPoolSize = 1;
	mTrainerParams.mNumInitSamples = 20000;
	mTrainerParams.mInitInputOffsetScale = false;
	mTrainerParams.mFreezeTargetIters = 500;
	mTrainerParams.mPretrainIters = 5000;

	mTrainer->Init(mTrainerParams);
	SetupScale();

	SetupActionBounds();
}

void cScenarioArmTrainDPG::BuildTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer) const
{
	auto trainer = std::shared_ptr<cDPGTrainer>(new cDPGTrainer());
	trainer->SetDPGReg(0.0001);
	trainer->SetQDiff(10);
	out_trainer = trainer;
}

void cScenarioArmTrainDPG::SetupCriticScale()
{
	auto trainer = std::static_pointer_cast<cDPGTrainer>(mTrainer);
	auto ctrl = GetController();
	int state_size = trainer->GetStateSize();
	int action_size = trainer->GetActionSize();
	int critic_input_size = trainer->GetCriticInputSize();
	int critic_output_size = trainer->GetCriticOutputSize();

	Eigen::VectorXd state_offset = Eigen::VectorXd::Zero(state_size);
	Eigen::VectorXd state_scale = Eigen::VectorXd::Ones(state_size);
	ctrl->BuildNNInputOffsetScale(state_offset, state_scale);

	Eigen::VectorXd action_offset = Eigen::VectorXd::Zero(action_size);
	Eigen::VectorXd action_scale = Eigen::VectorXd::Ones(action_size);
	ctrl->BuildNNOutputOffsetScale(action_offset, action_scale);

	Eigen::VectorXd critic_input_offset = Eigen::VectorXd::Zero(state_size + action_size);
	Eigen::VectorXd critic_input_scale = Eigen::VectorXd::Zero(state_size + action_size);
	critic_input_offset.segment(0, state_size) = state_offset;
	critic_input_offset.segment(state_size, action_size) = action_offset;
	critic_input_scale.segment(0, state_size) = state_scale;
	critic_input_scale.segment(state_size, action_size) = action_scale;

	assert(critic_input_offset.size() == critic_input_size);
	assert(critic_input_scale.size() == critic_input_size);
	trainer->SetCriticInputOffsetScale(critic_input_offset, critic_input_scale);

	Eigen::VectorXd critic_output_offset = -0.5 * Eigen::VectorXd::Ones(critic_output_size);
	Eigen::VectorXd critic_output_scale = 2 * Eigen::VectorXd::Ones(critic_output_size);
	trainer->SetCriticOutputOffsetScale(critic_output_offset, critic_output_scale);
}

void cScenarioArmTrainDPG::PrintInfo() const
{
	cScenarioArmTrain::PrintInfo();

	if (!mEnableTraining)
	{
		Eigen::VectorXd state;
		Eigen::VectorXd action;
		RecordState(state);
		RecordAction(action);
		tExpTuple tuple;
		tuple.mStateBeg = state;
		tuple.mAction = action;

		auto trainer = std::static_pointer_cast<cDPGTrainer>(mTrainer);
		Eigen::VectorXd dpg;
		trainer->CalcDPG(tuple, dpg);

		printf("DPG:\n");
		for (int i = 0; i < dpg.size(); ++i)
		{
			printf("%.5f\t", dpg[i]);
		}
		printf("\n");
	}

	printf("\n");
}