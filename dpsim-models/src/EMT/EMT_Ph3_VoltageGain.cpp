/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_VoltageGain.h>

using namespace CPS;


EMT::Ph3::VoltageGain::VoltageGain(String uid, String name, Logger::Level logLevel, Bool withTrafo) :
	CompositePowerComp<Real>(uid, name, true, true, logLevel),


	// netwotk voltage --> powerflow --> sub voltage source --> Gain --> --> sub voltage source (again) --> network

	// emt before signal always

	mVs(mAttributes->create<Matrix>("Vs", Matrix::Zero(3, 1))), // amplitude sub voltage source
	mElecActivePower(mAttributes->create<Real>("P_elec", 0)),
	mElecPassivePower(mAttributes->create<Real>("Q_elec", 0)),
	mVcd(mAttributes->create<Real>("Vc_d", 0)), // d-axis of voltage
	mVcq(mAttributes->create<Real>("Vc_q", 0)), // q-axis of voltage
	mVcdq(mAttributes->create<Matrix>("Vcdq", Matrix::Zero(2, 1))),

	mInputVoltage(mAttributes->create<Matrix>("V_input", Matrix::Zero(2, 1))), // input for gain
	mGainVoltage(mAttributes->create<Matrix>("V_gain", Matrix::Zero(2, 1))), // output of gain

	mGainInputs(mAttributes->createDynamic<Matrix>("voltagectrl_inputs")),
	mGainOutputs(mAttributes->createDynamic<Matrix>("voltagectrl_outputs")),
	mGainStates(mAttributes->createDynamic<Matrix>("voltagectrl_states")){

	mPhaseType = PhaseType::ABC;

	setVirtualNodeNumber(1);
	setTerminalNumber(1);

	SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", type(), name);
	**mIntfVoltage = Matrix::Zero(3, 1);
	**mIntfCurrent = Matrix::Zero(3, 1);

	// Create electrical sub components
	mSubCtrledVoltageSource = EMT::Ph3::VoltageSource::make(**mName + "_src", mLogLevel);

	// Pre-step of the subcontrolled voltage source is handled explicitly in mnaParentPreStep
	addMNASubComponent(mSubCtrledVoltageSource, MNA_SUBCOMP_TASK_ORDER::NO_TASK, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

	//Log subcomponents
	SPDLOG_LOGGER_INFO(mSLog, "Electrical subcomponents: ");
	for (auto subcomp: mSubComponents)
		SPDLOG_LOGGER_INFO(mSLog, "- {}", subcomp->name());

	// Create control sub components
	mGain = Signal::Gain::make(**mName + "_Gain", mLogLevel);

	// Gain as inputref comes in initializeNodes
	mGain->mInputRef->setReference(mVs); // input voltage of the gain
	//mGainVoltage->setReference(mGain->mOutputRef); // output of gain is saved in gain voltage

	// Sub voltage source
	// mGainVoltage->setReference(mSubCtrledVoltageSource->mVoltageRef); // output voltage
	//mSubCtrledVoltageSource->mVoltageRef->setReference(mVs); // controlled source gets value from gain
	mVs->setReference(mSubCtrledVoltageSource->mIntfVoltage);

	// input, state and output of the gain are saved
	// maybe useless
	mGainInputs->setReference(mGain->mInputCurr);
	mGainStates->setReference(mGain->mStateCurr);
	mGainOutputs->setReference(mGain->mOutputCurr);
}

//setter goal voltage and frequency
void EMT::Ph3::VoltageGain::setParameters(Real K_p, Matrix InputVoltage) {
	mParametersSet = true;

	SPDLOG_LOGGER_INFO(mSLog, "General Parameters:");
	SPDLOG_LOGGER_INFO(mSLog, "K_p = {}", K_p);

	mGain->setParameters(K_p);

	**mK_p = K_p;
	**mInputVoltage = InputVoltage;
}


void EMT::Ph3::VoltageGain::initializeFromNodesAndTerminals(Real frequency) {

	// use complex interface quantities for initialization calculations
	MatrixComp intfVoltageComplex = Matrix::Zero(3, 1);
	MatrixComp intfCurrentComplex = Matrix::Zero(3, 1);

	// terminal powers in consumer system -> convert to generator system
	Real activePower = terminal(0)->singlePower().real();;
	Real reactivePower = terminal(0)->singlePower().imag();

	// derive complex threephase initialization from single phase initial values (only valid for balanced systems)

	// intfvoltagecomplex = powerflow result
	intfVoltageComplex(0, 0) = RMS3PH_TO_PEAK1PH * initialSingleVoltage(0); // node part and network
	intfVoltageComplex(1, 0) = intfVoltageComplex(0, 0) * SHIFT_TO_PHASE_B;
	intfVoltageComplex(2, 0) = intfVoltageComplex(0, 0) * SHIFT_TO_PHASE_C;

	intfCurrentComplex(0, 0) = -std::conj(2./3.*Complex(activePower, reactivePower) / intfVoltageComplex(0, 0));
	intfCurrentComplex(1, 0) = intfCurrentComplex(0, 0) * SHIFT_TO_PHASE_B;
	intfCurrentComplex(2, 0) = intfCurrentComplex(0, 0) * SHIFT_TO_PHASE_C;


	// Set Node 0 voltage
	mVirtualNodes[0]->setInitialVoltage(intfVoltageComplex); // 3x1 Matrix


	// Calculate Powers
	**mIntfVoltage = intfVoltageComplex.real();
	**mIntfCurrent = intfCurrentComplex.real();

	**mElecActivePower = ( 3./2. * intfVoltageComplex(0,0) *  std::conj( - intfCurrentComplex(0,0)) ).real();
	**mElecPassivePower = ( 3./2. * intfVoltageComplex(0,0) *  std::conj( - intfCurrentComplex(0,0)) ).imag();

	// Initialize and connect controlled source
	mSubCtrledVoltageSource->setParameters(mVirtualNodes[0]->initialVoltage(), frequency); //voltage ref, frequency
	mSubCtrledVoltageSource->connect({ SimNode::GND, mVirtualNodes[0] });


	// Initialize gain controller
	mGain->setParameters(**mK_p);


	// Initialize electrical subcomponents
	for (auto subcomp: mSubComponents) {
		subcomp->initialize(mFrequencies);
		subcomp->initializeFromNodesAndTerminals(frequency);
	}

	// Initialize control subcomponents
	// voltage input for gain controller
	//Matrix ircdq;
	Real theta = std::arg(mVirtualNodes[0]->initialSingleVoltage());
	**mVcdq = parkTransformPowerInvariant(theta, intfVoltageComplex.real());
	//ircdq = parkTransformPowerInvariant(theta, -1 * filterInterfaceInitialCurrent.real());

	//mVcd = **mVcdq(0, 0);
	//mVcq = **mVcdq(1, 0);
	//**mIrcd = ircdq(0, 0);
	//**mIrcq = ircdq(1, 0);

	// Gain input settings --> input is voltage in dq-space
	mGain->mInputRef->setReference(mVcdq);
	mGain->setInitialValues(**mVcdq, Matrix::Zero(2,1), Matrix::Zero(2,1));


	SPDLOG_LOGGER_INFO(mSLog,
		"\n--- Initialization from powerflow ---"
		"\nInterface voltage across: {:s}"
		"\nInterface current: {:s}"
		"\nTerminal 0 initial voltage: {:s}"
		"\nTerminal 0 connected to {:s} = sim node {:d}"
		"\nVirtual node 0 initial voltage: {:s}",
		Logger::phasorToString(intfVoltageComplex(0, 0)),
		Logger::phasorToString(intfCurrentComplex(0, 0)),
		Logger::phasorToString(initialSingleVoltage(0)),
		mTerminals[0]->node()->name(), mTerminals[0]->node()->matrixNodeIndex(),
		Logger::phasorToString(mVirtualNodes[0]->initialSingleVoltage()));
		SPDLOG_LOGGER_INFO(mSLog,"\n--- Initialization from powerflow finished ---");

}

void EMT::Ph3::VoltageGain::mnaParentInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	mTimeStep = timeStep;

	// TODO: these are actually no MNA tasks
	mMnaTasks.push_back(std::make_shared<ControlPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<ControlStep>(*this));
}

void EMT::Ph3::VoltageGain::addControlPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	// add pre-step dependencies of subcomponents
	mGain->signalAddPreStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
}

void EMT::Ph3::VoltageGain::controlPreStep(Real time, Int timeStepCount) {
	// add pre-step of subcomponents
	mGain->signalPreStep(time, timeStepCount);
}

void EMT::Ph3::VoltageGain::addControlStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	// add step dependencies of subcomponents
	mGain->signalAddStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);

	// add step dependencies of component itself
	attributeDependencies.push_back(mIntfCurrent);
	attributeDependencies.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mVs);
}

Matrix EMT::Ph3::VoltageGain::parkTransformPowerInvariant(Real theta, const Matrix &fabc) {
	// Calculates fdq = Tdq * fabc
	// Assumes that d-axis starts aligned with phase a
	Matrix Tdq = getParkTransformMatrixPowerInvariant(theta);
	Matrix dqvector = Tdq * fabc;
	return dqvector;
}

Matrix EMT::Ph3::VoltageGain::getParkTransformMatrixPowerInvariant(Real theta) {
	// Return park matrix for theta
	// Assumes that d-axis starts aligned with phase a
	Matrix Tdq = Matrix::Zero(2, 3);
	Real k = sqrt(2. / 3.);
	Tdq <<
		k * cos(theta), k * cos(theta - 2. * M_PI / 3.), k * cos(theta + 2. * M_PI / 3.),
		-k * sin(theta), -k * sin(theta - 2. * M_PI / 3.), -k * sin(theta + 2. * M_PI / 3.);
	return Tdq;
}

Matrix EMT::Ph3::VoltageGain::inverseParkTransformPowerInvariant(Real theta, const Matrix &fdq) {
	// Calculates fabc = Tabc * fdq
	// with d-axis starts aligned with phase a
	Matrix Tabc = getInverseParkTransformMatrixPowerInvariant(theta);
	Matrix fabc = Tabc * fdq;
	return fabc;
}


Matrix EMT::Ph3::VoltageGain::getInverseParkTransformMatrixPowerInvariant(Real theta) {
	// Return inverse park matrix for theta
	/// with d-axis starts aligned with phase a
	Matrix Tabc = Matrix::Zero(3, 2);
	Real k = sqrt(2. / 3.);
	Tabc <<
		k * cos(theta), - k * sin(theta),
		k * cos(theta - 2. * M_PI / 3.), - k * sin(theta - 2. * M_PI / 3.),
		k * cos(theta + 2. * M_PI / 3.), - k * sin(theta + 2. * M_PI / 3.);
	return Tabc;
}


void EMT::Ph3::VoltageGain::controlStep(Real time, Int timeStepCount) {
	// add step of subcomponents

	mGain->signalStep(time, timeStepCount);
	**mVs = inverseParkTransformPowerInvariant(std::arg(mVirtualNodes[0]->initialSingleVoltage()), mGain->mOutputRef->get());
}

void EMT::Ph3::VoltageGain::mnaParentAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	prevStepDependencies.push_back(mVs);
	prevStepDependencies.push_back(mIntfCurrent);
	prevStepDependencies.push_back(mIntfVoltage);
	attributeDependencies.push_back(mGain->mOutputRef);
	modifiedAttributes.push_back(mRightVector);
}

void EMT::Ph3::VoltageGain::mnaParentPreStep(Real time, Int timeStepCount) {

	// set Voltage reference for voltage source
	mSubCtrledVoltageSource->mVoltageRef->set(**mVs);

	std::dynamic_pointer_cast<MNAInterface>(mSubCtrledVoltageSource)->mnaPreStep(time, timeStepCount);
	// pre-step of component itself
	mnaApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph3::VoltageGain::mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mIntfCurrent);
}

void EMT::Ph3::VoltageGain::mnaParentPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	mnaCompUpdateCurrent(**leftVector);
	mnaCompUpdateVoltage(**leftVector);
}

//Current update
void EMT::Ph3::VoltageGain::mnaCompUpdateCurrent(const Matrix& leftvector) {


}

//Voltage update
void EMT::Ph3::VoltageGain::mnaCompUpdateVoltage(const Matrix& leftVector) {
	for (auto virtualNode : mVirtualNodes)
		virtualNode->mnaUpdateVoltage(leftVector);
	(**mIntfVoltage)(0,0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(0,0));
	(**mIntfVoltage)(1,0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(0,1));
	(**mIntfVoltage)(2,0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(0,2));
}
