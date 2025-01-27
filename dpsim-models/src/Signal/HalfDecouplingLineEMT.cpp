/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/Signal/HalfDecouplingLineEMT.h>

using namespace CPS;
using namespace CPS::EMT::Ph1;
using namespace CPS::Signal;

HalfDecouplingLineEMT::HalfDecouplingLineEMT(String name, Logger::Level logLevel)
    : SimSignalComp(name, name, logLevel),
      mStates(mAttributes->create<Matrix>("states")),
      mSrcCurRef(mAttributes->create<Real>("i_src1")),
      mCoupledVoltage(mAttributes->createDynamic<Real>("coupled_voltage")),
      mCoupledCurrent(mAttributes->createDynamic<Real>("coupled_current")),
      mVolt(mAttributes->create<Real>("volt")),
      mCur(mAttributes->create<Real>("cur")) {

  mRes = Resistor::make(name + "_r", logLevel);
  mSrc = CurrentSource::make(name + "_i", logLevel);
  mSrcCur = mSrc->mCurrentRef;
}

void HalfDecouplingLineEMT::setParameters(SimNode<Real>::Ptr node1,
                                      SimNode<Real>::Ptr node2, Real resistance,
                                      Real inductance, Real capacitance, 
                                      Attribute<Real> coupledVoltage,
                                      Attribute<Real> coupledCurrent) {

  mCoupledVoltage->setReference(coupledVoltage);
  mCoupledCurrent->setReference(coupledCurrent);

  mResistance = resistance;
  mInductance = inductance;
  mCapacitance = capacitance;
  mNode1 = node1;
  mNode2 = node2;

  mSurgeImpedance = sqrt(inductance / capacitance);
  mDelay = sqrt(inductance * capacitance);
  SPDLOG_LOGGER_INFO(mSLog, "surge impedance: {}", mSurgeImpedance);
  SPDLOG_LOGGER_INFO(mSLog, "delay: {}", mDelay);

  mRes->setParameters(mSurgeImpedance + mResistance / 4);
  mRes->connect({node1, SimNode<Real>::GND});
  mSrc->setParameters(0);
  mSrc->connect({node1, SimNode<Real>::GND});
}

void HalfDecouplingLineEMT::initialize(Real omega, Real timeStep) {
  if (mDelay < timeStep)
    throw SystemError("Timestep too large for decoupling");

  mBufSize = static_cast<UInt>(ceil(mDelay / timeStep));
  mAlpha = 1 - (mBufSize - mDelay / timeStep);
  SPDLOG_LOGGER_INFO(mSLog, "bufsize {} alpha {}", mBufSize, mAlpha);

  // Initialization based on static PI-line model
  Complex volt1 = mNode1->initialSingleVoltage();
  Complex volt2 = mNode2->initialSingleVoltage();
  Complex initAdmittance = 1. / Complex(mResistance, omega * mInductance) +
                           Complex(0, omega * mCapacitance / 2);
  Complex cur1 = volt1 * initAdmittance -
                 volt2 / Complex(mResistance, omega * mInductance);
  Complex cur2 = volt2 * initAdmittance -
                 volt1 / Complex(mResistance, omega * mInductance);
  SPDLOG_LOGGER_INFO(mSLog, "initial voltages: v_k {} v_m {}", volt1, volt2);
  SPDLOG_LOGGER_INFO(mSLog, "initial currents: i_km {} i_mk {}", cur1, cur2);

  // Resize ring buffers and initialize
  mVoltBuf.resize(mBufSize, volt1.real());
  mCurBuf.resize(mBufSize, cur1.real());
}

Real HalfDecouplingLineEMT::interpolate(std::vector<Real> &data) {
  // linear interpolation of the nearest values
  Real c1 = data[mBufIdx];
  Real c2 = mBufIdx == mBufSize - 1 ? data[0] : data[mBufIdx + 1];
  return mAlpha * c1 + (1 - mAlpha) * c2;
}

void HalfDecouplingLineEMT::step(Real time, Int timeStepCount) {

  Real denom =
      (mSurgeImpedance + mResistance / 4) * (mSurgeImpedance + mResistance / 4);

  if (timeStepCount == 0) {
    // initialization
    **mSrcCurRef = mCur - mVolt / (mSurgeImpedance + mResistance / 4);
  } else {
    // Update currents
    **mSrcCurRef = -mSurgeImpedance / denom *
                        (mCoupledVoltage + (mSurgeImpedance - mResistance / 4) * mCoupledCurrent) -
                    mResistance / 4 / denom *
                        (mVolt + (mSurgeImpedance - mResistance / 4) * mCur);
  }
  mSrcCur->set(**mSrcCurRef);
}

void HalfDecouplingLineEMT::PreStep::execute(Real time, Int timeStepCount) {
  mLine.step(time, timeStepCount);
}

void HalfDecouplingLineEMT::postStep() {
  // Update ringbuffers with new values
  mVoltBuf[mBufIdx] = -mRes->intfVoltage()(0, 0);
  mCurBuf[mBufIdx] = -mRes->intfCurrent()(0, 0) + mSrcCur->get().real();

  mBufIdx++;
  if (mBufIdx == mBufSize)
    mBufIdx = 0;

  Real mVolt = interpolate(mVoltBuf);
  Real mCur = interpolate(mCurBuf);
}

void HalfDecouplingLineEMT::PostStep::execute(Real time, Int timeStepCount) {
  mLine.postStep();
}

Task::List HalfDecouplingLineEMT::getTasks() {
  return Task::List(
      {std::make_shared<PreStep>(*this), std::make_shared<PostStep>(*this)});
}

IdentifiedObject::List HalfDecouplingLineEMT::getLineComponents() {
  return IdentifiedObject::List({mRes, mSrc});
}
