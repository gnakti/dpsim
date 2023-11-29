/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <vector>

#include <dpsim-models/SimPowerComp.h>
#include <dpsim-models/SimSignalComp.h>
#include <dpsim-models/Task.h>

namespace CPS {
namespace Signal {
	class Gain :
		public SimSignalComp,
		public SharedFactory<Gain> {

	protected:
		/// Integration time step
        Real mTimeStep;

		Real mK_p;


	public:

		/// This is never explicitely set to reference anything, so the outside code is responsible for setting up the reference.
		const Attribute<Matrix>::Ptr mInputRef;
		/// Value, which goes to outside classes
		const Attribute<Matrix>::Ptr mOutputRef;

		const Attribute<Matrix>::Ptr mInputVoltage;


		/// Previous Input
        const Attribute<Matrix>::Ptr mInputPrev;
        /// Current Input
        const Attribute<Matrix>::Ptr mInputCurr;
        /// Previous State
        const Attribute<Matrix>::Ptr mStatePrev;
        /// Current State
        const Attribute<Matrix>::Ptr mStateCurr;
        /// Previous Output
        const Attribute<Matrix>::Ptr mOutputPrev;
        /// Current Output
        const Attribute<Matrix>::Ptr mOutputCurr;



		Gain(String name, Logger::Level logLevel = Logger::Level::off);
		/// Setter for integration step parameter
		void setParameters(Real K_p);
		/// Setter for initial values
        void setInitialValues(Matrix input_init, Matrix state_init, Matrix output_init);
		/// pre step operations
		void signalPreStep(Real time, Int timeStepCount);
		/// step operations
		void signalStep(Real time, Int timeStepCount);
		/// pre step dependencies
		void signalAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes);
		/// add step dependencies
		void signalAddStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes);

		Task::List getTasks();

        class PreStep : public Task {
        public:
			PreStep(Gain& gain) :
                Task(**gain.mName + ".PreStep"), mGain(gain) {
					mGain.signalAddPreStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
			}
			void execute(Real time, Int timeStepCount) { mGain.signalPreStep(time, timeStepCount); };
		private:
			Gain& mGain;
        };

		class Step : public Task {
		public:
			Step(Gain& gain) :
				Task(**gain.mName + ".Step"), mGain(gain) {
					mGain.signalAddStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
			}
			void execute(Real time, Int timeStepCount) { mGain.signalStep(time, timeStepCount); };
		private:
			Gain& mGain;
		};
	};
}
}
