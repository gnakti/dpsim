/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/Signal/VCO.h>

using namespace CPS;
using namespace CPS::Signal;

VCO::VCO(String name, Logger::Level logLevel) :
	SimSignalComp(name, name, logLevel),
    mInputRef(Attribute<Real>::createDynamic("input_ref", mAttributes)),
    /// CHECK: Which of these really need to be attributes?
    mInputPrev(Attribute<Real>::create("input_prev", mAttributes)),
    mStatePrev(Attribute<Real>::create("state_prev", mAttributes)),
    mOutputPrev(Attribute<Real>::create("output_prev", mAttributes)),
    mInputCurr(Attribute<Real>::create("input_curr", mAttributes)),
    mStateCurr(Attribute<Real>::create("state_curr", mAttributes)),
    mOutputCurr(Attribute<Real>::create("output_curr", mAttributes)) { }


void VCO::setParameters(Real omegaNom) {
    // Input is OmegaNom
    // Output is Theta
    mOmegaNom = omegaNom;
    mSLog->info("OmegaNom = {}", mOmegaNom);

    **mInputCurr = mOmegaNom;
    **mInputRef = mOmegaNom;
}

void VCO::setSimulationParameters(Real timestep) {
    mTimeStep = timestep;
    mSLog->info("Integration step = {}", mTimeStep);
}

void VCO::setInitialValues(Real input_init, Real state_init, Real output_init) {
	**mInputCurr = mOmegaNom;
    **mStateCurr = state_init;
    **mOutputCurr = output_init;

    mSLog->info("Initial values:");
    mSLog->info("inputCurrInit = ({}), stateCurrInit = ({}), outputCurrInit = ({})", **mInputCurr, **mStateCurr, **mOutputCurr);
}

void VCO::signalAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
    prevStepDependencies.push_back(mInputCurr);
	prevStepDependencies.push_back(mOutputCurr);
	modifiedAttributes.push_back(mInputPrev);
    modifiedAttributes.push_back(mOutputPrev);
};

void VCO::signalPreStep(Real time, Int timeStepCount) {
    **mInputPrev = **mInputCurr;
    **mStatePrev = **mStateCurr;
    **mOutputPrev = **mOutputCurr;
}

void VCO::signalAddStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	attributeDependencies.push_back(mInputRef);
	modifiedAttributes.push_back(mInputCurr);
    modifiedAttributes.push_back(mOutputCurr);
};

void VCO::signalStep(Real time, Int timeStepCount) {
    **mInputCurr = **mInputRef;

    mSLog->info("Time {}:", time);
    mSLog->info("Input values: inputCurr = {}, inputPrev = ({}, stateCurr = ({}, statePrev = ({})", **mInputCurr, **mInputPrev, **mStateCurr, **mStatePrev);

    **mStateCurr = (**mStatePrev + mTimeStep * **mInputCurr);
    **mOutputCurr = **mStateCurr;

    mSLog->info("State values: stateCurr = ({})", **mStateCurr);
    mSLog->info("Output values: outputCurr = ({}):", **mOutputCurr);
}

Task::List VCO::getTasks() {
	return Task::List({std::make_shared<PreStep>(*this), std::make_shared<Step>(*this)});
}