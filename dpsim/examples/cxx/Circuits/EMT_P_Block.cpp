/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <DPsim.h>

using namespace DPsim;
using namespace CPS;


int main(int argc, char* argv[]) {
	// Define simulation scenario
	Real timeStep = 0.0001;
	Real finalTime = 0.1;
	String simName = "EMT_P_Block";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode<Real>::make("n1");
	auto n2 = SimNode<Real>::make("n2");

	// Components
	auto vs = EMT::Ph3::VoltageSource::make("vs");
	vs->setParameters(CPS::Math::singlePhaseVariableToThreePhase(Complex(10, 0)), 50);
	auto r1 = EMT::Ph3::Resistor::make("r_1");
	r1->setParameters(CPS::Math::singlePhaseParameterToThreePhase(5));
	auto p1 = EMT::Ph3::VoltageGain::make("p_1");
	p1->setParameters(5, CPS::Math::singlePhasePowerToThreePhase(100));

	// Topology
	vs->connect({ n1 });
	p1->connect({ n1, n2 });
	r1->connect({ n2 });

	// Define system topology
	auto sys = SystemTopology(50, SystemNodeList{n1, n2}, SystemComponentList{vs, r1, p1});

	// Logger
	auto logger = DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("v2", n2->attribute("v"));

	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::EMT);
	sim.addLogger(logger);

	sim.run();

	return 0;
}
