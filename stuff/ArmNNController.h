#pragma once

#include "ArmController.h"
#include "learning/NeuralNet.h"

class cArmNNController : public cArmController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cArmNNController();
	virtual ~cArmNNController();

	virtual void Init(cSimCharacter* character);
	virtual void Reset();
	virtual void Clear();

	virtual bool LoadNet(const std::string& net_file);
	virtual void LoadModel(const std::string& model_file);
	virtual void LoadScale(const std::string& scale_file);

	virtual void CopyNet(const cNeuralNet& net);
	virtual void SaveNet(const std::string& out_file) const;

	virtual bool HasNet() const;
	virtual bool IsOffPolicy() const;

protected:
	cNeuralNet mNet;
	double mExpNoise;
	bool mOffPolicy;

	virtual void LoadNetIntern(const std::string& net_file);

	virtual void UpdatePoliAction();
	virtual void DecideAction();
	virtual void ApplyExpNoise(tAction& out_action) const;
	virtual void FetchExpNoiseScale(Eigen::VectorXd& out_noise) const;

	virtual void ApplyPoliAction(double time_step, const tAction& action);

	virtual int GetNetInputSize() const;
	virtual int GetNetOutputSize() const;
};