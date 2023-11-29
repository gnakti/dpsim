/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/
#pragma once

#include <dpsim-models/CompositePowerComp.h>
#include <dpsim-models/Solver/MNAInterface.h>
#include <dpsim-models/Definitions.h>
#include <dpsim-models/EMT/EMT_Ph3_VoltageSource.h>
#include <dpsim-models/Signal/Gain.h>


namespace CPS {
namespace EMT {
namespace Ph3 {
	class VoltageGain :
		public CompositePowerComp<Real>,
		public SharedFactory<VoltageGain> {
	protected:

		// ### General Parameters ###

		/// Simulation step
		Real mTimeStep;

		// ### Control Subcomponents ###

		/// Gain
		std::shared_ptr<Signal::Gain> mGain;

		/// Controlled voltage source
		std::shared_ptr<EMT::Ph3::VoltageSource> mSubCtrledVoltageSource;

		// #### solver ####
		///
		std::vector<const Matrix*> mRightVectorStamps;

	public:
		// ### General Parameters ###


		// ### Inverter Interfacing Variables ###

		// Control inputs

		// sub voltage source (3-phase)
		const Attribute<Matrix>::Ptr mVs;

		// dq-transformated voltage voltage source with singular components
		const Attribute<Matrix>::Ptr mVcdq;
		const Attribute<Real>::Ptr mVcd;
		const Attribute<Real>::Ptr mVcq;

		// parameter for gain controller
		const Attribute<Real>::Ptr mK_p;

		// power attributes
		const Attribute<Real>::Ptr mElecActivePower;
		const Attribute<Real>::Ptr mElecPassivePower;

		// Control outputs
		/// Voltage as control output after transformation interface
		const Attribute<Matrix>::Ptr mGainVoltage; // in dq-frame

		// input, state and output vector for logging
		const Attribute<Matrix>::Ptr mGainInputs;
		const Attribute<Matrix>::Ptr mGainStates;
		const Attribute<Matrix>::Ptr mGainOutputs;

		/// Defines name amd logging level
		VoltageGain(String name, Logger::Level logLevel = Logger::Level::off)
			: VoltageGain(name, name, logLevel) {}
		/// Defines UID, name, logging level and connection trafo existence
		VoltageGain(String uid, String name, Logger::Level logLevel = Logger::Level::off, Bool withTrafo = false);

		// #### General ####
		/// Initializes component from power flow data
		void initializeFromNodesAndTerminals(Real frequency);
		/// Setter for gengit eral parameters of inverter
		void setParameters(Real K_p);



		// #### MNA section ####
		/// Initializes internal variables of the component
		void mnaParentInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) override;
		/// Updates current through the component
		void mnaCompUpdateCurrent(const Matrix& leftVector) override;
		/// Updates voltage across component
		void mnaCompUpdateVoltage(const Matrix& leftVector) override;
		/// MNA pre step operations
		void mnaParentPreStep(Real time, Int timeStepCount) override;
		/// MNA post step operations
		void mnaParentPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) override;
		/// Add MNA pre step dependencies
		void mnaParentAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) override;
		/// Add MNA post step dependencies
		void mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) override;

		// Transformation Utils
		Matrix parkTransformPowerInvariant(Real theta, const Matrix &fabc);

		Matrix getParkTransformMatrixPowerInvariant(Real theta);

		Matrix inverseParkTransformPowerInvariant(Real theta, const Matrix &fdq);

		Matrix getInverseParkTransformMatrixPowerInvariant(Real theta);


		// #### Control section ####
		/// Control pre step operations
		void controlPreStep(Real time, Int timeStepCount);
		/// Perform step of controller
		void controlStep(Real time, Int timeStepCount);
		/// Add control step dependencies
		void addControlPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes);
		/// Add control step dependencies
		void addControlStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes);

		class ControlPreStep : public CPS::Task {
		public:
			ControlPreStep(VoltageGain& VoltageGain) :
				Task(**VoltageGain.mName + ".ControlPreStep"), mVoltageGain(VoltageGain) {
					mVoltageGain.addControlPreStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
			}
			void execute(Real time, Int timeStepCount) { mVoltageGain.controlPreStep(time, timeStepCount); };

		private:
			VoltageGain& mVoltageGain;
		};

		class ControlStep : public CPS::Task {
		public:
			ControlStep(VoltageGain& VoltageGain) :
				Task(**VoltageGain.mName + ".ControlStep"), mVoltageGain(VoltageGain) {
					mVoltageGain.addControlStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
			}
			void execute(Real time, Int timeStepCount) { mVoltageGain.controlStep(time, timeStepCount); };

		private:
			VoltageGain& mVoltageGain;
		};
	};
}
}
}
