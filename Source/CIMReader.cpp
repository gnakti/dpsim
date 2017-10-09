/** Read CIM files
 *
 * @author Georg Reinke <georg.reinke@rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
 * @license GNU General Public License (version 3)
 *
 * DPsim
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************************/

#include "CIMReader.h"
#include "CIMModel.hpp"
#include "IEC61970.hpp"

#include "Components/PQLoad.h"
#include "Components/RxLine.h"
#include "Components/TwoWindingTransformer.h"

using namespace DPsim;
using namespace IEC61970::Base::Core;
using namespace IEC61970::Base::Domain;
using namespace IEC61970::Base::Equivalents;
using namespace IEC61970::Base::Topology;
using namespace IEC61970::Base::Wires;

// TODO is UnitMulitplier actually used/set anywhere?
double CIMReader::unitValue(double value, UnitMultiplier mult) {
	switch (mult) {
	case UnitMultiplier::p:
		value *= 1e-12;
		break;
	case UnitMultiplier::n:
		value *= 1e-9;
		break;
	case UnitMultiplier::micro:
		value *= 1e-6;
		break;
	case UnitMultiplier::m:
		value *= 1e-3;
		break;
	case UnitMultiplier::c:
		value *= 1e-2;
		break;
	case UnitMultiplier::d:
		value *= 1e-1;
		break;
	case UnitMultiplier::k:
		value *= 1e3;
		break;
	case UnitMultiplier::M:
		value *= 1e6;
		break;
	case UnitMultiplier::G:
		value *= 1e9;
		break;
	case UnitMultiplier::T:
		value *= 1e12;
		break;
	default:
		break;
	}
	return value;
}

CIMReader::CIMReader(Real om) {
	mModel.setDependencyCheckOff();
	mNumVoltageSources = 0;
	mVoltages = nullptr;
	mFrequency = om;
}

CIMReader::~CIMReader() {
	if (mVoltages)
		delete[] mVoltages;
}

BaseComponent* CIMReader::mapACLineSegment(ACLineSegment* line) {
	std::vector<int> &nodes = mEqNodeMap.at(line->mRID); // TODO can fail
	if (nodes.size() != 2) {
		std::cerr << "ACLineSegment " << line->mRID << " has " << nodes.size() << " terminals, ignoring" << std::endl;
		// TODO better error handling (throw exception?)
		return nullptr;
	}
	Real r = line->r.value;
	Real x = line->x.value;
	std::cerr << "RxLine " << line->name << " rid=" << line->mRID << " node1=" << nodes[0] << " node2=" << nodes[1];
	std::cerr << " R=" << r << " X=" << x << std::endl;
	return new RxLine(line->name, nodes[0], nodes[1], r, x/mFrequency);
}

BaseComponent* CIMReader::mapAsynchronousMachine(AsynchronousMachine* machine) {
	return newFlowPQLoad(machine->mRID, machine->name);
}

BaseComponent* CIMReader::mapEnergyConsumer(EnergyConsumer* con) {
	return newFlowPQLoad(con->mRID, con->name);
}

BaseComponent* CIMReader::mapEquivalentInjection(EquivalentInjection* inj) {
	return newFlowPQLoad(inj->mRID, inj->name);
}

BaseComponent* CIMReader::mapExternalNetworkInjection(ExternalNetworkInjection* inj) {
	std::vector<int> &nodes = mEqNodeMap.at(inj->mRID);
	if (nodes.size() != 1) {
		std::cerr << "ExternalNetworkInjection " << inj->mRID << " has " << nodes.size() << " terminals, ignoring" << std::endl;
		return nullptr;
	}
	int node = nodes[0];
	SvVoltage *volt = mVoltages[node-1];
	if (!volt) {
		std::cerr << "ExternalNetworkInjection " << inj->mRID << " has no associated SvVoltage, ignoring" << std::endl;
		return nullptr;
	}
	std::cerr << "IdealVoltageSource " << inj->name << " rid=" << inj->mRID << " node1=" << node << " node2=0 ";
	std::cerr << " V=" << volt->v.value << "<" << volt->angle.value << std::endl;
	return new IdealVoltageSource(inj->name, node, 0, Complex(volt->v.value, volt->angle.value*PI/180), ++mNumVoltageSources);
}

BaseComponent* CIMReader::mapPowerTransformer(PowerTransformer* trans) {
	std::vector<int> &nodes = mEqNodeMap.at(trans->mRID);
	if (nodes.size() != trans->PowerTransformerEnd.size()) {
		std::cerr << "PowerTransformer " << trans->mRID << " has differing number of terminals and windings, ignoring" << std::endl;
		return nullptr;
	}
	if (nodes.size() != 2) {
		// TODO three windings also possible
		std::cerr << "PowerTransformer " << trans->mRID << " has " << nodes.size() << "terminals; ignoring" << std::endl;
		return nullptr;
	}
	for (PowerTransformerEnd *end : trans->PowerTransformerEnd) {
		if (end->endNumber == 1) {
			std::cerr << "PowerTransformer " << trans->name << " rid=" << trans->mRID << " node1=" << nodes[0] << "node2=" << nodes[1] << " R=" << end->r.value << " X=" << end->x.value << std::endl;
			return new TwoWindingTransformer(trans->name, nodes[0], nodes[1], end->r.value, end->x.value/mFrequency);
		}
	}
	std::cerr << "PowerTransformer " << trans->mRID << " has no primary End; ignoring" << std::endl;
	return nullptr;
}

BaseComponent* CIMReader::mapSynchronousMachine(SynchronousMachine* machine) {
	// TODO: don't use SvVoltage, but map to a SynchronGenerator instead?
	std::vector<int> &nodes = mEqNodeMap.at(machine->mRID);
	if (nodes.size() != 1) {
		// TODO check with the model if this assumption (only 1 terminal) is always true
		std::cerr << "SynchronousMachine " << machine->mRID << " has " << nodes.size() << " terminals, ignoring" << std::endl;
		return nullptr;
	}
	int node = nodes[0];
	SvVoltage *volt = mVoltages[node-1];
	if (!volt) {
		std::cerr << "SynchronousMachine " << machine->mRID << " has no associated SvVoltage, ignoring" << std::endl;
		return nullptr;
	}
	std::cerr << "VoltSourceRes " << machine->name << " rid=" << machine->mRID << " node1=" << node << " node2=0 ";
	std::cerr << " V=" << volt->v.value << "<" << volt->angle.value << " R=" << machine->r.value << std::endl;
	// TODO is it appropiate to use this resistance here
	return new VoltSourceRes(machine->name, node, 0, Complex(volt->v.value, volt->angle.value*PI/180), machine->r.value);
}

BaseComponent* CIMReader::mapComponent(BaseClass* obj) {
	if (ACLineSegment *line = dynamic_cast<ACLineSegment*>(obj))
		return mapACLineSegment(line);
	if (AsynchronousMachine *machine = dynamic_cast<AsynchronousMachine*>(obj))
		return mapAsynchronousMachine(machine);
	if (EnergyConsumer *con = dynamic_cast<EnergyConsumer*>(obj))
		return mapEnergyConsumer(con);
	if (EquivalentInjection *inj = dynamic_cast<EquivalentInjection*>(obj))
		return mapEquivalentInjection(inj);
	if (ExternalNetworkInjection *inj = dynamic_cast<ExternalNetworkInjection*>(obj))
		return mapExternalNetworkInjection(inj);
	if (PowerTransformer *trans = dynamic_cast<PowerTransformer*>(obj))
		return mapPowerTransformer(trans);
	if (SynchronousMachine *syncMachine = dynamic_cast<SynchronousMachine*>(obj))
		return mapSynchronousMachine(syncMachine);
	return nullptr;
}

BaseComponent* CIMReader::newFlowPQLoad(std::string rid, std::string name) {
	std::vector<int> &nodes = mEqNodeMap.at(rid);
	if (nodes.size() != 1) {
		std::cerr << rid << " has " << nodes.size() << " terminals; ignoring" << std::endl;
		return nullptr;
	}
	auto search = mPowerFlows.find(rid);
	if (search == mPowerFlows.end()) {
		std::cerr << rid << " has no associated SvPowerFlow, ignoring" << std::endl;
		return nullptr;
	}
	SvPowerFlow* flow = search->second;
	int node = nodes[0];
	SvVoltage *volt = mVoltages[node-1];
	if (!volt) {
		std::cerr << rid << " has no associated SvVoltage, ignoring" << std::endl;
		return nullptr;
	}
	std::cerr << "PQLoad " << name << " rid=" << rid << " node1=" << node << " node2=0 P=" << flow->p.value << " Q=" << flow->q.value;
	std::cerr << " V=" << volt->v.value << "<" << volt->angle.value << std::endl;
	return new PQLoad(name, node, 0, flow->p.value, flow->q.value, volt->v.value, volt->angle.value*PI/180);
}

bool CIMReader::addFile(std::string filename) {
	return mModel.addCIMFile(filename);
}

void CIMReader::parseFiles() {
	mModel.parseFiles();
	// First, go through all topological nodes and collect them in a list.
	// Since all nodes have references to the equipment connected to them (via Terminals), but not
	// the other way around (which we need for instantiating the components), we collect that information here as well.
	for (BaseClass* obj : mModel.Objects) {
		TopologicalNode* topNode = dynamic_cast<TopologicalNode*>(obj);
		if (topNode) {
			std::cerr << "TopologicalNode " << mTopNodes.size()+1 << " rid=" << topNode->mRID << " Terminals:" << std::endl;
			mTopNodes[topNode->mRID] = mTopNodes.size()+1;
			for (Terminal* term : topNode->Terminal) {
				std::cerr << "    " << term->mRID << std::endl;
				ConductingEquipment *eq = term->ConductingEquipment;
				if (!eq) {
					std::cerr << "Terminal " << term->mRID << " has no Conducting Equipment, ignoring!" << std::endl;
				} else {
					std::cerr << "    eq " << eq->mRID << " sequenceNumber " << term->sequenceNumber << std::endl;
					std::vector<int> &nodesVec = mEqNodeMap[eq->mRID];
					if (nodesVec.size() < term->sequenceNumber)
						nodesVec.resize(term->sequenceNumber);
					nodesVec[term->sequenceNumber-1] = mTopNodes.size();
				}
			}
		}
	}
	// Collect voltage state variables associated to nodes that are used
	// for various components.
	mVoltages = new SvVoltage*[mTopNodes.size()];
	std::cerr << "Voltages" << std::endl;
	for (BaseClass* obj : mModel.Objects) {
		if (SvVoltage* volt = dynamic_cast<SvVoltage*>(obj)) {
			TopologicalNode* node = volt->TopologicalNode;
			if (!node) {
				std::cerr << "SvVoltage references missing Topological Node, ignoring" << std::endl;
				continue;
			}
			auto search = mTopNodes.find(node->mRID);
			if (search == mTopNodes.end()) {
				std::cerr << "SvVoltage references Topological Node " << node->mRID << " missing from mTopNodes, ignoring" << std::endl;
				continue;
			}
			mVoltages[search->second-1] = volt;
			std::cerr << volt->v.value << "<" << volt->angle.value << " at " << search->second << std::endl;
		} else if (SvPowerFlow* flow = dynamic_cast<SvPowerFlow*>(obj)) {
			// TODO could there be more than one power flow per equipment?
			Terminal* term = flow->Terminal;
			mPowerFlows[term->ConductingEquipment->mRID] = flow;
		}
	}
	for (BaseClass* obj : mModel.Objects) {
		BaseComponent* comp = mapComponent(obj);
		if (comp)
			mComponents.push_back(comp);
	}
}

std::vector<BaseComponent*>& CIMReader::getComponents() {
	return mComponents;
}

int CIMReader::mapTopologicalNode(std::string mrid) {
	auto search = mTopNodes.find(mrid);
	if (search == mTopNodes.end())
		return -1;
	return search->second;
}

int CIMReader::getNumVoltageSources() {
	return mNumVoltageSources;
}