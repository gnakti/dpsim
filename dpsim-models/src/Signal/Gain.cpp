/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/Signal/Gain.h>

using namespace CPS;
using namespace CPS::Signal;

Gain::Gain(String name, Logger::Level logLevel) :
	SimSignalComp(name, name, logLevel),

	// references of input and output
    mInputRef(mAttributes->createDynamic<Matrix>("input_ref")), // input voltage
	mOutputRef(mAttributes->createDynamic<Matrix>("output_ref")), // output voltage


	// previous states
    mInputPrev(mAttributes->create<Matrix>("input_prev", Matrix::Zero(2,1))),
    mStatePrev(mAttributes->create<Matrix>("state_prev", Matrix::Zero(2,1))),
    mOutputPrev(mAttributes->create<Matrix>("output_prev", Matrix::Zero(2,1))),

	// current states
    mInputCurr(mAttributes->create<Matrix>("input_curr", Matrix::Zero(2,1))),
    mStateCurr(mAttributes->create<Matrix>("state_curr", Matrix::Zero(2,1))),
    mOutputCurr(mAttributes->create<Matrix>("output_curr", Matrix::Zero(2,1)))

	{
		SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", type(), name);
	}

void Gain::setParameters(Real K_p) {
	// setter for Gain parameter K_p
    mK_p = K_p;

    SPDLOG_LOGGER_INFO(mSLog, "K_p = {}", mK_p);
}

// setter for initial state-space values
void Gain::setInitialValues(Matrix input_init, Matrix state_init, Matrix output_init) {
	**mInputCurr = input_init;
    **mStateCurr = state_init;
    **mOutputCurr = output_init;

    SPDLOG_LOGGER_INFO(mSLog, "Initial values:");
    SPDLOG_LOGGER_INFO(mSLog, "inputCurrInit = {}, stateCurrInit = {}, outputCurrInit = {}", **mInputCurr, **mStateCurr, **mOutputCurr);
}

void Gain::signalAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
    prevStepDependencies.push_back(mInputCurr);
	prevStepDependencies.push_back(mOutputCurr);
	modifiedAttributes.push_back(mInputPrev);
    modifiedAttributes.push_back(mOutputPrev);
};

void Gain::signalPreStep(Real time, Int timeStepCount) {
    **mInputPrev = **mInputCurr;
    **mStatePrev = **mStateCurr;
    **mOutputPrev = **mOutputCurr;
}

void Gain::signalAddStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	attributeDependencies.push_back(mInputRef);
	modifiedAttributes.push_back(mInputCurr);
    modifiedAttributes.push_back(mOutputCurr);
};

void Gain::signalStep(Real time, Int timeStepCount) {
	// current input is the voltage reference from outside
	**mInputCurr = **mInputRef;

    SPDLOG_LOGGER_INFO(mSLog, "Time {}:", time);
    SPDLOG_LOGGER_INFO(mSLog, "Input values: inputCurr = {}, inputPrev = {}, statePrev = {}", **mInputCurr, **mInputPrev, **mStatePrev);

	// output voltage is gain * input voltage
    **mOutputCurr = mK_p * **mInputCurr;
	**mOutputRef = **mOutputCurr;

    SPDLOG_LOGGER_INFO(mSLog, "State values: stateCurr = {}", **mStateCurr);
    SPDLOG_LOGGER_INFO(mSLog, "Output values: outputCurr = {}:", **mOutputCurr);
}

Task::List Gain::getTasks() {
	return Task::List({std::make_shared<PreStep>(*this), std::make_shared<Step>(*this)});
}
