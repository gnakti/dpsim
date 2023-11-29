/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <DPsim.h>
#include "../Examples.h"

using namespace DPsim;
using namespace CPS;


int main(int argc, char* argv[]) {

	CIM::Examples::Grids::SGIB::ScenarioConfig scenario;

	Real finalTime = 2;
	Real timeStep = 0.0001;
	String simName = "EMT_P_Block";
	Logger::setLogDir("logs/"+simName);
	String simNamePF = simName + "_PF";
	Logger::setLogDir("logs/" + simNamePF);

	// ----- POWERFLOW FOR INITIALIZATION -----
	Real timeStepPF = finalTime;
	Real finalTimePF = finalTime+timeStepPF;


	// Components
	auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);

	auto extnetPF = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
	extnetPF->setParameters(scenario.systemNominalVoltage);
	extnetPF->setBaseVoltage(scenario.systemNominalVoltage);
	extnetPF->modifyPowerFlowBusType(PowerflowBusType::VD);

	auto linePF = SP::Ph1::PiLine::make("PiLine", Logger::Level::debug);
	linePF->setParameters(scenario.lineResistance, scenario.lineInductance, scenario.lineCapacitance);
	linePF->setBaseVoltage(scenario.systemNominalVoltage);

	auto loadPF = SP::Ph1::Load::make("Load", Logger::Level::debug);
	loadPF->setParameters(-scenario.pvNominalActivePower, -scenario.pvNominalReactivePower, scenario.systemNominalVoltage);
	loadPF->modifyPowerFlowBusType(PowerflowBusType::PQ);

	// Topology
	extnetPF->connect({ n1PF });
	linePF->connect({ n1PF, n2PF });
	loadPF->connect({ n2PF });
	auto systemPF = SystemTopology(50,
			SystemNodeList{n1PF, n2PF},
			SystemComponentList{extnetPF, linePF, loadPF});

	// Logging
	auto loggerPF = DataLogger::make(simNamePF);
	loggerPF->logAttribute("v1", n1PF->attribute("v"));
	loggerPF->logAttribute("v2", n2PF->attribute("v"));

	// Simulation
	Simulation simPF(simNamePF, Logger::Level::debug);
	simPF.setSystem(systemPF);
	simPF.setTimeStep(timeStepPF);
	simPF.setFinalTime(finalTimePF);
	simPF.setDomain(Domain::SP);
	simPF.setSolverType(Solver::Type::NRP);
	simPF.setSolverAndComponentBehaviour(Solver::Behaviour::Initialization);
	simPF.doInitFromNodesAndTerminals(false);
	simPF.addLogger(loggerPF);
	simPF.run();


	// Define simulation scenario


	// Nodes
	auto n1 = SimNode<Real>::make("n1");
	auto n2 = SimNode<Real>::make("n2");

	// Components
	auto vs = EMT::Ph3::VoltageSource::make("vs");
	vs->setParameters(CPS::Math::singlePhaseVariableToThreePhase(Complex(10, 0)), 50);
	auto r1 = EMT::Ph3::Resistor::make("r_1");
	r1->setParameters(CPS::Math::singlePhaseParameterToThreePhase(5));
	auto p1 = EMT::Ph3::VoltageGain::make("p_1");
	p1->setParameters(5);

	// Topology
	vs->connect({ n1 });
	p1->connect({ n1 });
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
