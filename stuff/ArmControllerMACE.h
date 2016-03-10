#pragma once

#include "ArmNNController.h"
#include "learning/NeuralNet.h"

class cArmControllerMACE : public cArmNNController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cArmControllerMACE();
	virtual ~cArmControllerMACE();

	virtual void Reset();

	virtual int GetNumActionFrags() const;
	virtual int GetActionFragSize() const;
	virtual int GetNetOutputSize() const;

	virtual void RecordPoliAction(Eigen::VectorXd& out_action) const;
	
	virtual bool IsExpCritic() const;
	virtual bool IsExpActor() const;

	virtual void BuildNNOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;

protected:
	int mNumActionFrags;
	std::vector<double> mBoltzmannBuffer;
	bool mExpCritic;
	bool mExpActor;

	virtual void LoadNetIntern(const std::string& net_file);

	virtual void DecideAction();
	virtual void UpdateFragParams();
	virtual void BuildActorAction(const Eigen::VectorXd& params, int a_id, tAction& out_action) const;

	virtual void FetchExpNoiseScale(Eigen::VectorXd& out_noise) const;
	virtual void BuildActorBias(int a_id, Eigen::VectorXd& out_bias) const;

	virtual int GetMaxFragIdx(const Eigen::VectorXd& params) const;
	virtual double GetMaxFragVal(const Eigen::VectorXd& params) const;
	virtual void GetFrag(const Eigen::VectorXd& params, int a_idx, Eigen::VectorXd& out_action) const;
	virtual void SetFrag(const Eigen::VectorXd& frag, int a_idx, Eigen::VectorXd& out_params) const;
	virtual double GetVal(const Eigen::VectorXd& params, int a_idx) const;
	virtual void SetVal(double val, int a_idx, Eigen::VectorXd& out_params) const;
};