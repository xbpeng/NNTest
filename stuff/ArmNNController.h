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

	virtual void CopyNet(const cNeuralNet& net);
	virtual void SaveNet(const std::string& out_file) const;

	virtual bool HasNet() const;

protected:
	cNeuralNet mNet;

	virtual void UpdatePoliAction();
	virtual void ApplyPoliAction(const Eigen::VectorXd& action) const;
};