/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <vector>

#include <dpsim-models/EMT/EMT_Ph1_CurrentSource.h>
#include <dpsim-models/EMT/EMT_Ph1_Resistor.h>
#include <dpsim-models/SimSignalComp.h>
#include <dpsim-models/Task.h>

namespace CPS {
namespace Signal {
class HalfDecouplingLineEMT : public SimSignalComp,
                          public SharedFactory<HalfDecouplingLineEMT> {
protected:
  Real mDelay;
  Real mResistance;
  Real mInductance;
  Real mCapacitance;
  Real mSurgeImpedance;

  std::shared_ptr<EMT::SimNode> mNode1, mNode2;
  std::shared_ptr<EMT::Ph1::Resistor> mRes;
  std::shared_ptr<EMT::Ph1::CurrentSource> mSrc;
  Attribute<Complex>::Ptr mSrcCur;

  // Ringbuffers for the values of previous timesteps
  // TODO make these matrix attributes
  std::vector<Real> mVoltBuf, mCurBuf;
  UInt mBufIdx = 0;
  UInt mBufSize;
  Real mAlpha;

  Real interpolate(std::vector<Real> &data);

public:
  typedef std::shared_ptr<HalfDecouplingLineEMT> Ptr;

  const Attribute<Real>::Ptr mSrcCurRef;
  const Attribute<Real>::Ptr mVolt;
  const Attribute<Real>::Ptr mCur;
  const Attribute<Real>::Ptr mCoupledVoltage;
  const Attribute<Real>::Ptr mCoupledCurrent;

  ///FIXME: workaround for dependency analysis as long as the states aren't attributes
  const Attribute<Matrix>::Ptr mStates;

  HalfDecouplingLineEMT(String name, Logger::Level logLevel = Logger::Level::info);

  void setParameters(SimNode<Real>::Ptr node1, SimNode<Real>::Ptr node2,
                     Real resistance, Real inductance, Real capacitance,
                     HalfDecouplingLineEMT::Ptr couplingHalfLine);
  void initialize(Real omega, Real timeStep);
  void step(Real time, Int timeStepCount);
  void postStep();
  Task::List getTasks();
  IdentifiedObject::List getLineComponents();

  class PreStep : public Task {
  public:
    PreStep(HalfDecouplingLineEMT &line)
        : Task(**line.mName + ".MnaPreStep"), mLine(line) {
      mPrevStepDependencies.push_back(mLine.mStates);
      mModifiedAttributes.push_back(mLine.mSrc->mCurrentRef);
    }

    void execute(Real time, Int timeStepCount);

  private:
    HalfDecouplingLineEMT &mLine;
  };

  class PostStep : public Task {
  public:
    PostStep(HalfDecouplingLineEMT &line)
        : Task(**line.mName + ".PostStep"), mLine(line) {
      mAttributeDependencies.push_back(mLine.mRes->mIntfVoltage);
      mAttributeDependencies.push_back(mLine.mRes->mIntfCurrent);
      mModifiedAttributes.push_back(mLine.mStates);
    }

    void execute(Real time, Int timeStepCount);

  private:
    HalfDecouplingLineEMT &mLine;
  };
};
} // namespace Signal
} // namespace CPS
