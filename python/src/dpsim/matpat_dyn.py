import pandas as pd
import numpy as np
import scipy.io
from enum import Enum
import dpsimpy

# define dpsimpy domains
class Domain(Enum):
    PF = 1
    SP = 2
    DP = 3
    EMT = 4
    
# default multiplier for matpower data
w_mw = 1e-6
mw_w = 1e6
kv_v = 1e3
        
class Reader:

    def __init__(self, mpc_file_path, mpc_name = 'mpc', mpc_dyn_file_path=None, mpc_dyn_name = 'mpc'):
        """
        Read input file (returns multidimensional dict)
        
        @param mpc_file_path: path to the mpc file containing the static data (normal mpc file)
        @param mpc_name: name of the struct inside of mpc_file_path
        @param mpc_dyn_file_path: Optional parameters, used for dynamic simulations in DPsim.
                                  Indicates the path to the dynamic mpc file containing the dynamic data
        @param mpc_dyn_name: name of the struct inside of mpc_dyn_file_path
        """
        self.mpc_raw = scipy.io.loadmat(mpc_file_path, simplify_cells= True)
        self.mpc_name = mpc_name
        
        self.dyn_data = False
        if (mpc_dyn_file_path is not None):
            self.dyn_data = True
            self.mpc_raw_dyn = scipy.io.loadmat(mpc_dyn_file_path, simplify_cells= True)
            self.mpc_dyn_name = mpc_dyn_name

    def process_mpc(self, frequency):
        """
        Process raw mpc data and create corresponding dataframes
        @param frequency: system frequency
        """

        # System frequency (not included in mpc but needed for setting dpsimpy component parameters i.e inductances, capacitances ..)
        self.mpc_freq = frequency
        self.mpc_omega = 2 * np.pi * frequency

        # Base power (MVA)
        self.mpc_base_power_MVA =  self.mpc_raw[self.mpc_name]['baseMVA'] * mw_w

        #### Busses #####
        mpc_bus_raw = self.mpc_raw[self.mpc_name]['bus']
        bus_data_header = ["bus_i", "type", "Pd", "Qd", "Gs", "Bs", "area",
                           "Vm", "Va", "baseKV", "zone", "Vmax", "Vmin"]
        self.mpc_bus_data = pd.DataFrame(mpc_bus_raw, columns = bus_data_header)
        
        # scipy.io.loadmat loads all matrix entries as double. Convert specific columns back to int
        self.mpc_bus_data['bus_i'] = self.mpc_bus_data['bus_i'].astype(int)
        self.mpc_bus_data['type'] = self.mpc_bus_data['type'].astype(int)
        self.mpc_bus_data['area'] = self.mpc_bus_data['area'].astype(int)
        self.mpc_bus_data['zone'] = self.mpc_bus_data['zone'].astype(int)
        
        #### Generators #####
        mpc_gen_raw = self.mpc_raw[self.mpc_name]['gen']
        gen_data_header = ["bus", "Pg", "Qg", "Qmax", "Qmin", "Vg", "mBase", "status",
                           "Pmax", "Pmin", "Pc1", "Pc2", "Qc1min", "Qc1max", "Qc2min",
                           "Qc2max", "ramp_agc", "ramp_10", "ramp_30", "ramp_q", "apf"]
        self.mpc_gen_data = pd.DataFrame(mpc_gen_raw, columns = gen_data_header)
        
        # scipy.io.loadmat loads all matrix entries as double. Convert specific columns back to int
        self.mpc_gen_data['bus'] = self.mpc_gen_data['bus'].astype(int)
        self.mpc_gen_data['status'] = self.mpc_gen_data['status'].astype(int)
       
        #### Branches #####
        # extract only first 13 columns since following columns include results
        mpc_branch_raw = self.mpc_raw[self.mpc_name]['branch'][:, :13]
        branch_data_header = ["fbus", "tbus", "r", "x", "b", "rateA", "rateB",
                            "rateC", "ratio", "angle", "status", "angmin", "angmax"]
        self.mpc_branch_data = pd.DataFrame(mpc_branch_raw, columns = branch_data_header)

        # scipy.io.loadmat loads all matrix entries as double. Convert specific columns back to int
        self.mpc_branch_data['fbus'] = self.mpc_branch_data['fbus'].astype(int)
        self.mpc_branch_data['tbus'] = self.mpc_branch_data['tbus'].astype(int)
        self.mpc_branch_data['status'] = self.mpc_branch_data['status'].astype(int)
        
        #### Additional fields: bus_names, bus_assets #####
        ## Obs.: not part of the original mpc format!
        self.mpc_bus_names_dict=dict()
        self.mpc_bus_assets_dict=dict()

        if 'bus_names' in self.mpc_raw[self.mpc_name]:
            self.mpc_bus_names_dict=dict(zip(self.mpc_bus_data['bus_i'], self.mpc_raw[self.mpc_name]['bus_names']))

        if 'bus_assets' in self.mpc_raw[self.mpc_name]:
            self.mpc_bus_assets_dict=dict(zip(self.mpc_bus_data['bus_i'],self.mpc_raw[self.mpc_name]['bus_assets']))

        #### TODO Generator costs ####
    
    def process_mpc_dyn(self):
        #### Dynamic generator data
        self.mpc_dyn_gen_data = None
        if 'gen' in self.mpc_raw_dyn[self.mpc_dyn_name]:
            mpc_dyn_gen_data = self.mpc_raw_dyn[self.mpc_dyn_name]['gen'][:, :16]
            
            dyn_gen_data_header = ["bus","model","BaseS","H","D","Xd","Xq","Xd_t","Xq_t","Xd_s","Xq_s","Td0_t","Tq0_t","Td0_s","Tq0_s", "Ra"]
            self.mpc_dyn_gen_data = pd.DataFrame(mpc_dyn_gen_data, columns = dyn_gen_data_header)
            # scipy.io.loadmat loads all matrix entries as double. Convert specific columns back to int
            self.mpc_dyn_gen_data['bus'] = self.mpc_dyn_gen_data['bus'].astype(int)
            self.mpc_dyn_gen_data['model'] = self.mpc_dyn_gen_data['model'].astype(int)
            
        #### AVR data
        self.mpc_avr_data = None
        if 'exc' in self.mpc_raw_dyn[self.mpc_dyn_name]:
            mpc_avr_data = self.mpc_raw_dyn[self.mpc_dyn_name]['exc']
            avr_data_header = ["bus", "Type", "Ka", "Te", "Ta", "Tb","U_min", "U_max", "Kbc"]
            self.mpc_avr_data = pd.DataFrame(np.matrix(mpc_avr_data), columns = avr_data_header)
            # scipy.io.loadmat loads all matrix entries as double. Convert specific columns back to int
            self.mpc_avr_data['bus'] = self.mpc_avr_data['bus'].astype(int)
            self.mpc_avr_data['Type'] = self.mpc_avr_data['Type'].astype(int)

        #### PSS data
        self.mpc_pss_data = None
        if 'pss' in self.mpc_raw_dyn[self.mpc_dyn_name]:
            mpc_pss_data = self.mpc_raw_dyn[self.mpc_dyn_name]['pss']
            pss_data_header = ["bus", "Type", "Tw", "T1", "T2", "T3", "T4", "Ks", "U_smin", "U_smax"]
            self.mpc_pss_data = pd.DataFrame(np.matrix(mpc_pss_data), columns = pss_data_header)
           
           # scipy.io.loadmat loads all matrix entries as double. Convert specific columns back to int
            self.mpc_pss_data['bus'] = self.mpc_pss_data['bus'].astype(int)
            self.mpc_pss_data['Type'] = self.mpc_pss_data['Type'].astype(int)
            
        #### TG data
        self.mpc_tg_data = None
        if 'gov' in self.mpc_raw_dyn[self.mpc_dyn_name]:
            mpc_tg_data = self.mpc_raw_dyn[self.mpc_dyn_name]['gov']
            tg_data_header = ["bus", "Type", "K", "T1", "T2", "T3", "Pup", "Pdown", "Pmax", "Pmin"]
            self.mpc_tg_data = pd.DataFrame(np.matrix(mpc_tg_data), columns = tg_data_header)
            # scipy.io.loadmat loads all matrix entries as double. Convert specific columns back to int
            self.mpc_tg_data['bus'] = self.mpc_tg_data['bus'].astype(int)
            self.mpc_tg_data['Type'] = self.mpc_tg_data['Type'].astype(int)
            
    def create_dpsim_objects(self, domain, frequency, log_level=dpsimpy.LogLevel.info, 
                             with_pss=True, with_avr=True, with_tg=True):
        """
        Create dpsim objects with the data contained in the mpc files.
        The dpsim nodes are stored in the dict self.dpsimpy_busses_dict.
        All other components are stored in the dict self.dpsimpy_comp_dict
        
        @param domain: modeling domain to be used in dpsim (PF, SP, EMT or DP)
        @param frequency: system frequency
        @param log_level: log level used by the dpsim objects
        """
        self.log_level = log_level
        self.domain = domain
        
        # process mpc files
        self.process_mpc(frequency=frequency)
        if (self.dyn_data):
            self.process_mpc_dyn()        

        # get the correct module to be used (dpsimpy.sp, dpsimpy.dp or dpsimpy.emt) 
        self.dpsimpy_components = None
        if (self.domain == Domain.PF or self.domain == Domain.SP):
            self.dpsimpy_components = dpsimpy.sp.ph1
            self.dpsimpy_components.SimNode = getattr(dpsimpy.sp, "SimNode")
        elif (self.domain == Domain.DP):
            self.dpsimpy_components = dpsimpy.dp.ph1
            self.dpsimpy_components.SimNode = getattr(dpsimpy.dp, "SimNode")
        elif (self.domain == Domain.EMT):
            self.dpsimpy_components = dpsimpy.emt.ph3
            self.dpsimpy_components.SimNode = getattr(dpsimpy.emt, "SimNode")  
        else:
            raise Exception('ERROR: domain {} is not supported in dpsimpy'.format(self.domain))
        
        # return values: nodes and components
        self.dpsimpy_busses_dict = {}
        self.dpsimpy_comp_dict = {}

        # initialize number of loads to 0
        self.loads_count = 0
        
        # initialize number of network injections to 0
        self.inj = 0

        for index, bus in self.mpc_bus_data.iterrows():
            # create dpsimpy nodes
            bus_index = str(self.mpc_bus_data.at[index,'bus_i']) #index: 0....N-1, bus_index: 1...N
            bus_assets= []
            bus_name = self.get_node_name(bus_index)    # bus_name="N"+bus_index
            if self.mpc_bus_assets_dict:
                if isinstance(self.mpc_bus_assets_dict[int(bus_index)], str):
                    bus_assets.append(self.mpc_bus_assets_dict[int(bus_index)])
                else:
                    bus_assets=list(self.mpc_bus_assets_dict[int(bus_index)])
                    
            if (self.domain == Domain.EMT):      
                self.dpsimpy_busses_dict[bus_name] = self.dpsimpy_components.SimNode(bus_name, dpsimpy.PhaseType.ABC)
            else:
                self.dpsimpy_busses_dict[bus_name] = self.dpsimpy_components.SimNode(bus_name, dpsimpy.PhaseType.Single)                
               
            # for each bus type create corresponding dpsimpy component
            # 1 = PQ, 2 = PV, 3 = ref, 4 = isolated
            bus_type = self.mpc_bus_data.at[index,'type']

            # PQ busses
            if bus_type == 1:
                if not bus_assets: 
                    # aggregated load
                    self.map_energy_consumer(index, bus_index, bus_type=dpsimpy.PowerflowBusType.PQ)
                    
                else:
                    for i, comp in enumerate(bus_assets): 
                        # WARN: now the loads are created and the first load is initialized with the total power P_d Q_d!!!!!!!!                       
                        if i==0:
                            self.map_energy_consumer(index, bus_index, load_name=comp, bus_type=dpsimpy.PowerflowBusType.PQ)
                        else:
                            load_baseV = self.mpc_bus_data.at[index,'baseKV'] * kv_v
                            self.dpsimpy_comp_dict[comp] = [self.dpsimpy_components.Load(comp, log_level)]
                            self.dpsimpy_comp_dict[comp][0].set_parameters(0, 0, load_baseV)
                            self.dpsimpy_comp_dict[comp].append([self.dpsimpy_busses_dict[bus_name]])

                #shunts
                self.map_shunt(index, bus_index)
                    
            # Generators
            elif bus_type == 2:
                # map SG
                self.map_synchronous_machine(index, bus_index, with_pss=with_pss, with_tg=with_tg, with_avr=with_avr)
                
                # check if there is a load connected to PV bus (and create it)
                self.map_energy_consumer(index, bus_index, bus_type=dpsimpy.PowerflowBusType.PV)

                #shunts
                self.map_shunt(index, bus_index)

            # Slack bus
            elif bus_type == 3:
                #TODO: Use SG or NetworkInjection???? --> SG
                # # set initial Q is not implemented for network injection in DPsim. 
                # # The mapping was changed here to test case ieee14/ieee39.
                # # This mapping does not work with asm uc because Pg and Qg of the slack are equal to zero
                # # TODO implement initial reactive power for slack in Dpsim.
                
                #self.map_network_injection(index, bus_index, map_network_injection)
                self.map_synchronous_machine(index, bus_index, bus_type=dpsimpy.PowerflowBusType.VD,
                                             with_pss=with_pss, with_tg=with_tg, with_avr=with_avr)

                # check if there is a load connected to slack bus (and create it)
                self.map_energy_consumer(index, bus_index, bus_type=dpsimpy.PowerflowBusType.VD)
                
                #shunts
                self.map_shunt(index, bus_index)
                    
            #isolated
            elif bus_type == 4:
                print("Isolated bus type")
            else:
                print("Bus type error")

        ### branches ####
        line_num = 0
        trafo_num = 0
        for index, branch in self.mpc_branch_data.iterrows():

            ### Distinction between lines and transformer is done now with the off nominal ratio.
            
            # get branch ratio
            branch_ratio = self.mpc_branch_data.at[index,'ratio']

            # get matpower index
            fbus_index = self.mpc_branch_data.at[index,'fbus'] # matpower index 1 ... N
            tbus_index = self.mpc_branch_data.at[index,'tbus'] # matpower index 1 ... N

            # get rows of interest
            tmp_fbus = self.mpc_bus_data.loc[self.mpc_bus_data['bus_i'] == fbus_index]
            tmp_tbus = self.mpc_bus_data.loc[self.mpc_bus_data['bus_i'] == tbus_index]

            # get base voltages
            fbus_baseV = self.mpc_bus_data.at[tmp_fbus.first_valid_index(),'baseKV'] * kv_v
            tbus_baseV = self.mpc_bus_data.at[tmp_tbus.first_valid_index(),'baseKV'] * kv_v

            # Lines
            ### In matpower, tap = 0 is used to indicate transmission line rather than transformer,
            ### i.e. mathematically equivalent to transformer with tap = 1). That means, that transformers
            ### with ratio=1 can be represented with tap = 0 so that we have to check also the base voltages 
            if (branch_ratio==0) and (fbus_baseV==tbus_baseV):
                line_num = line_num + 1
                line_name = "line%s_%s-%s" %(line_num, fbus_index, tbus_index)

                # get line parameters
                line_baseZ = tbus_baseV * tbus_baseV / (self.mpc_base_power_MVA)
                line_r = self.mpc_branch_data.at[index,'r'] * line_baseZ
                line_x = self.mpc_branch_data.at[index,'x'] * line_baseZ
                line_b = self.mpc_branch_data.at[index,'b'] / line_baseZ
                line_l = line_x / self.mpc_omega
                line_c = line_b / self.mpc_omega
                line_g = 0 # line conductance is not included in mpc
                
                # create dpsim object
                self.dpsimpy_comp_dict[line_name] = [self.dpsimpy_components.PiLine(line_name, log_level)]
                if (self.domain==Domain.EMT):
                    self.dpsimpy_comp_dict[line_name][0].set_parameters(dpsimpy.Math.single_phase_parameter_to_three_phase(line_r), 
                                                                        dpsimpy.Math.single_phase_parameter_to_three_phase(line_l), 
                                                                        dpsimpy.Math.single_phase_parameter_to_three_phase(line_c), 
                                                                        dpsimpy.Math.single_phase_parameter_to_three_phase(line_g))
                else:
                    self.dpsimpy_comp_dict[line_name][0].set_parameters(line_r, line_l, line_c, line_g)
                if (self.domain==Domain.PF):
                    self.dpsimpy_comp_dict[line_name][0].set_base_voltage(tbus_baseV)
                
                # add connections
                self.dpsimpy_comp_dict[line_name].append([self.dpsimpy_busses_dict[self.get_node_name(tbus_index)], self.dpsimpy_busses_dict[self.get_node_name(fbus_index)]])

            # Transformers
            else:
                # Primary side is fbus (fbus_baseV, primary_V)
                # Secondary side is tbus
                # In matpower impedaces are referred to tbus
                

                #is there if the second condition in upper if is false: not(fbus_baseV==tbus_baseV)
                if (branch_ratio==0):
                    branch_ratio=1
                # 
                trafo_num = trafo_num + 1
                transf_name = "transformer%s_%s-%s" %(trafo_num, fbus_index, tbus_index)

                # From MATPOWER-manual taps at “from” bus,  impedance at “to” bus,  i.e.  ifr=x=b= 0,tap=|Vf|/|Vt|
                # transform impedances to absolute values
                fbus_baseV = fbus_baseV
                transf_baseZ = tbus_baseV * tbus_baseV / (self.mpc_base_power_MVA)
                transf_ratioAbs = branch_ratio * fbus_baseV / tbus_baseV         
                
                #TO DO: For transformation of Impedances a ratio with angle shift has to be used!     

                # DPsim convention: impedance values must be referred to high voltage side (and base voltage set to higher voltage)
                if fbus_baseV > tbus_baseV:
                    # HV side is fbus_baseV
                    # impedances are referred to LV side --> change side
                    transf_baseV = fbus_baseV
                    transf_r = self.mpc_branch_data.at[index,'r'] * (transf_ratioAbs**2) * transf_baseZ
                    transf_x = self.mpc_branch_data.at[index,'x'] * (transf_ratioAbs**2) * transf_baseZ
                    transf_l = transf_x / self.mpc_omega
                else:
                    transf_baseV = tbus_baseV
                    if (self.domain==Domain.PF):
                        transf_r = self.mpc_branch_data.at[index,'r'] * transf_baseZ * (branch_ratio**2)
                        transf_x = self.mpc_branch_data.at[index,'x'] * transf_baseZ * (branch_ratio**2)
                    else:
                        transf_r = self.mpc_branch_data.at[index,'r'] * transf_baseZ
                        transf_x = self.mpc_branch_data.at[index,'x'] * transf_baseZ                                                                                                                                              transf_x = self.mpc_branch_data.at[index,'x'] * transf_baseZ
                    transf_l = transf_x / self.mpc_omega
                
                # create dpsim component
                trafo = self.dpsimpy_components.Transformer(transf_name, log_level)
                if (self.domain==Domain.EMT):
                    trafo.set_parameters(fbus_baseV, tbus_baseV, np.abs(transf_ratioAbs), np.angle(transf_ratioAbs), 
                                         dpsimpy.Math.single_phase_parameter_to_three_phase(transf_r), 
                                         dpsimpy.Math.single_phase_parameter_to_three_phase(transf_l))
                else:
                    trafo.set_parameters(fbus_baseV, tbus_baseV, np.abs(transf_ratioAbs), 
                                         np.angle(transf_ratioAbs), transf_r, transf_l)
                if (self.domain==Domain.PF):
                    trafo.set_base_voltage(transf_baseV)
                self.dpsimpy_comp_dict[transf_name] = [trafo]

                # add connections
                self.dpsimpy_comp_dict[transf_name].append([self.dpsimpy_busses_dict[self.get_node_name(fbus_index)], self.dpsimpy_busses_dict[self.get_node_name(tbus_index)]])

    def get_node_name(self, node_index):
        bus_name = ""
        if self.mpc_bus_names_dict:
            bus_name= self.mpc_bus_names_dict[int(node_index)]
        else:
            bus_name = 'N' + str(node_index)
    
        return bus_name
    
    def map_synchronous_machine(self, index, bus_index, bus_type=dpsimpy.PowerflowBusType.PV,
                                with_pss=True, with_tg=True, with_avr=True):
        #
        gen_name = "Gen_N" + str(bus_index)
                
        # relevant data from self.mpc_gen_data. Identification with bus number available in mpc_bus_data and mpc_gen_data
        gen_data = self.mpc_gen_data.loc[self.mpc_gen_data['bus'] == self.mpc_bus_data.at[index,'bus_i']]
        gen_baseS = gen_data['mBase']*mw_w  # gen base MVA default is mpc.baseMVA
        gen_baseV = self.mpc_bus_data.at[index,'baseKV']*kv_v # gen base kV
        gen_v = gen_data['Vg']*gen_baseV    # gen set point voltage (gen['Vg'] in p.u.)
        gen_p = gen_data['Pg']*mw_w         # gen ini. active power (gen['Pg'] in MVA)
        gen_q = gen_data['Qg']*mw_w         # gen ini. reactive power (gen['Qg'] in MVAr)
        
        gen = None
        if (self.domain == Domain.PF):
            gen = self.dpsimpy_components.SynchronGenerator(gen_name, self.log_level)
            gen.set_parameters(gen_baseS, gen_baseV, gen_p, gen_v, bus_type, gen_q)
            gen.set_base_voltage(gen_baseV) 
            gen.modify_power_flow_bus_type(bus_type)          
        else:
            # get dynamic data of the generator
            gen_dyn_row_idx =  self.mpc_dyn_gen_data.index[self.mpc_dyn_gen_data['bus'] == int(bus_index)].tolist()[0]
            # TODO: throw error if len(self.mpc_dyn_gen_data.index[self.mpc_dyn_gen_data['bus'] == int(bus_index)].tolist() > 1) ?? 
            # --> two gens associated to one node...

            # gen_model = self.mpc_dyn_gen_data['model'][gen_dyn_row_idx]
            gen_model=4
            gen_baseS = self.mpc_dyn_gen_data['BaseS'][gen_dyn_row_idx] * mw_w 
            H = self.mpc_dyn_gen_data['H'][gen_dyn_row_idx]
            Ra = self.mpc_dyn_gen_data['Ra'][gen_dyn_row_idx]
            #Ll = self.mpc_dyn_gen_data['Xl'][gen_dyn_row_idx]
            Ll=0.1
            Td0_t = self.mpc_dyn_gen_data['Td0_t'][gen_dyn_row_idx]
            Td0_s = self.mpc_dyn_gen_data['Td0_s'][gen_dyn_row_idx]
            Tq0_t = self.mpc_dyn_gen_data['Tq0_t'][gen_dyn_row_idx]
            Tq0_s = self.mpc_dyn_gen_data['Tq0_s'][gen_dyn_row_idx]
            Ld = self.mpc_dyn_gen_data['Xd'][gen_dyn_row_idx]
            Ld_t = self.mpc_dyn_gen_data['Xd_t'][gen_dyn_row_idx]
            Ld_s = self.mpc_dyn_gen_data['Xd_s'][gen_dyn_row_idx]
            Lq = self.mpc_dyn_gen_data['Xq'][gen_dyn_row_idx]
            Lq_t = self.mpc_dyn_gen_data['Xq_t'][gen_dyn_row_idx]
            Lq_s = self.mpc_dyn_gen_data['Xq_s'][gen_dyn_row_idx]
                
            if (gen_model==3):
                gen = self.dpsimpy_components.SynchronGenerator3OrderVBR(gen_name, self.log_level)
                gen.set_operational_parameters_per_unit(nom_power=gen_baseS, nom_voltage=gen_baseV, nom_frequency=self.mpc_freq, 
                                                        H=H, Ld=Ld, Lq=Lq, L0=Ll, Ld_t=Ld_t, Td0_t=Td0_t)
            elif (gen_model==4):
                gen = self.dpsimpy_components.SynchronGenerator4OrderVBR(gen_name, self.log_level)
                gen.set_operational_parameters_per_unit(nom_power=gen_baseS, nom_voltage=gen_baseV, nom_frequency=self.mpc_freq, 
                                        H=H, Ld=Ld, Lq=Lq, L0=Ll, Ld_t=Ld_t, Lq_t=Lq_t, Td0_t=Td0_t, Tq0_t=Tq0_t)
            elif (gen_model==5):
                gen = self.dpsimpy_components.SynchronGenerator5OrderVBR(gen_name, self.log_level)
                gen.set_operational_parameters_per_unit(nom_power=gen_baseS, nom_voltage=gen_baseV, nom_frequency=self.mpc_freq, 
                                        H=H, Ld=Ld, Lq=Lq, L0=Ll, Ld_t=Ld_t, Lq_t=Lq_t, Td0_t=Td0_t, Tq0_t=Tq0_t,
                                        Ld_s=Ld_s, Lq_s=Lq_s, Td0_s=Td0_s, Tq0_s=Tq0_s, Taa=0)
            elif (gen_model==6):
                gen = self.dpsimpy_components.SynchronGenerator6bOrderVBR(gen_name, self.log_level)
                gen.set_operational_parameters_per_unit(nom_power=gen_baseS, nom_voltage=gen_baseV, nom_frequency=self.mpc_freq, 
                                        H=H, Ld=Ld, Lq=Lq, L0=Ll, Ld_t=Ld_t, Lq_t=Lq_t, Td0_t=Td0_t, Tq0_t=Tq0_t,
                                        Ld_s=Ld_s, Lq_s=Lq_s, Td0_s=Td0_s, Tq0_s=Tq0_s, Taa=0)		
            else:
                raise Exception('Matpower reader does not support the generator model {}. Supported models are: "3", "4", "5", "6".'.format(gen_model))
        
        #### SG controllers ####
        if (self.domain != Domain.PF):
            # search for avr
            if with_avr:
                if (self.mpc_avr_data is not None and int(bus_index) in self.mpc_avr_data['bus'].tolist()):
                    try:
                        avr_row_idx = self.mpc_avr_data.index[self.mpc_avr_data['bus'] == int(bus_index)].tolist()[0]
                        exciter = dpsimpy.signal.ExciterStatic("Exciter_Bus" + bus_index, self.log_level)
                        exciter_parameters = dpsimpy.signal.ExciterStaticParameters()
                        exciter_parameters.Ta = self.mpc_avr_data['Ta'][avr_row_idx]
                        exciter_parameters.Tb = self.mpc_avr_data['Tb'][avr_row_idx]
                        exciter_parameters.Te = self.mpc_avr_data['Te'][avr_row_idx]
                        exciter_parameters.Ka = self.mpc_avr_data['Ka'][avr_row_idx]
                        exciter_parameters.MaxEfd = self.mpc_avr_data['U_max'][avr_row_idx]
                        exciter_parameters.MinEfd = self.mpc_avr_data['U_min'][avr_row_idx]
                        exciter_parameters.Kbc = self.mpc_avr_data['Kbc'][avr_row_idx]
                        exciter_parameters.Kbc = 0
                        exciter.set_parameters(exciter_parameters)
                        gen.add_exciter(exciter)
                    except Exception as e:
                        print("ERROR: " + str(e))
                        raise Exception()
                
            # search for pss
            if with_pss:
                if (self.mpc_pss_data is not None and int(bus_index) in self.mpc_pss_data['bus'].tolist()):
                    try:
                        pss_row_idx =  self.mpc_pss_data.index[self.mpc_pss_data['bus'] == int(bus_index)].tolist()[0]
                        pss = dpsimpy.signal.PSS1A("PSS_Bus" + bus_index, self.log_level)
                        pss_parameters = dpsimpy.signal.PSS1AParameters()
                        pss_parameters.Kp = 0 
                        pss_parameters.Kv = 0 
                        pss_parameters.Kw = self.mpc_pss_data['Ks'][pss_row_idx]
                        pss_parameters.T1 = self.mpc_pss_data['T1'][pss_row_idx]
                        pss_parameters.T2 = self.mpc_pss_data['T2'][pss_row_idx]
                        pss_parameters.T3 = self.mpc_pss_data['T3'][pss_row_idx]
                        pss_parameters.T4 = self.mpc_pss_data['T4'][pss_row_idx]
                        pss_parameters.Vs_max = self.mpc_pss_data['U_smax'][pss_row_idx]
                        pss_parameters.Vs_min = self.mpc_pss_data['U_smin'][pss_row_idx]
                        pss_parameters.Tw = self.mpc_pss_data['Tw'][pss_row_idx]
                        pss.set_parameters(pss_parameters)
                        gen.add_pss(pss)
                    except Exception as e:
                        print("ERROR: " + str(e))
                        raise Exception()
                
            # search for turbine governors
            if with_tg:
                if (self.mpc_tg_data is not None and int(bus_index) in self.mpc_tg_data['bus'].tolist()):
                    try:
                        tg_row_idx =  self.mpc_tg_data.index[self.mpc_tg_data['bus'] == int(bus_index)].tolist()[0]
                        governor = dpsimpy.signal.SteamTurbineGovernor("TG_Bus" + bus_index, self.log_level)
                        governor_parameters = dpsimpy.signal.SteamGorvernorParameters()
                        governor_parameters.OmRef = 1
                        governor_parameters.R = 1 / self.mpc_tg_data['K'][tg_row_idx]
                        #governor_parameters.T1 = self.mpc_tg_data['T2'][tg_row_idx]
                        governor_parameters.T1 = 0
                        governor_parameters.T2 = self.mpc_tg_data['T1'][tg_row_idx]
                        governor_parameters.T3 = self.mpc_tg_data['T3'][tg_row_idx]
                        governor_parameters.dPmax = self.mpc_tg_data['Pup'][tg_row_idx]
                        governor_parameters.dPmin = self.mpc_tg_data['Pdown'][tg_row_idx]
                        governor_parameters.Pmax = self.mpc_tg_data['Pmax'][tg_row_idx]
                        governor_parameters.Pmin = self.mpc_tg_data['Pmin'][tg_row_idx]
                        #governor_parameters.Kbc = self.mpc_tg_data['Kbc'][tg_row_idx]
                        #governor_parameters.Kbc = 10
                        governor.set_parameters(governor_parameters)
                        gen.add_governor(governor)
                    except Exception as e:
                        print("ERROR: " + str(e))
                        raise Exception()
        
        #
        self.dpsimpy_comp_dict[gen_name] = [gen]
        self.dpsimpy_comp_dict[gen_name].append([self.dpsimpy_busses_dict[self.get_node_name(bus_index)]]) # [to bus]
        
    def map_energy_consumer(self, index, bus_index, load_name=None, bus_type = dpsimpy.PowerflowBusType.PQ):
        # check if there is a load connected to PV bus
        load_p = self.mpc_bus_data.at[index,'Pd'] * mw_w
        load_q = self.mpc_bus_data.at[index,'Qd'] * mw_w
        if (bus_type != dpsimpy.PowerflowBusType.PQ or self.domain != Domain.PF):
            if (load_p==0) and (load_q==0):
                # no load must be added
                return
        
        if (load_name==None):
            # increment number of loads by 1
            self.loads_count += 1
            load_name = "load%s" %self.loads_count
        
        # get relevant data
        load_baseV = self.mpc_bus_data.at[index,'baseKV'] * kv_v
        
        # create dpsim components
        load = None
        if self.domain in [Domain.PF, Domain.SP]:
            load = self.dpsimpy_components.Load(load_name, self.log_level)
        else:
           load = self.dpsimpy_components.RXLoad(load_name, self.log_level)
        
        if (self.domain==Domain.EMT):
            load_p = dpsimpy.Math.single_phase_parameter_to_three_phase(load_p/3)
            load_q = dpsimpy.Math.single_phase_parameter_to_three_phase(load_q/3)
        load.set_power(load_p, load_q)
        
        if (self.domain==Domain.PF and bus_type==dpsimpy.PowerflowBusType.PQ):
            load.modify_power_flow_bus_type(bus_type)
        
        self.dpsimpy_comp_dict[load_name] = [load]
        self.dpsimpy_comp_dict[load_name].append([self.dpsimpy_busses_dict[self.get_node_name(bus_index)]]) # [to bus]
    
    def map_shunt(self, index, bus_index):
        # Gs in Matpower represents the MW demanded at V = 1.0 p.u., Bs the MVAr injected at V = 1.0 p.u.
        
        # check if there is a shunt connected to this bus
        Gs = self.mpc_bus_data.at[index,'Gs']
        Bs = self.mpc_bus_data.at[index,'Bs']

        if (Gs==0) and (Bs==0):
            # no shunt connected to this bus
            return
        
        # get node base voltage
        bus_baseV = self.mpc_bus_data.at[index,'baseKV'] * kv_v
        
        # calculate equivalent conductance and susceptance
        # p = −gv2 --> g=-p/v2
        # q = bv2  --> b=q/v2
        gs = Gs*mw_w / (bus_baseV**2)
        bs = Bs*mw_w / (bus_baseV**2)           
        
        # create dpsim component
        shunt = None
        if (self.domain==Domain.PF):
            shunt_name = "Shunt_N" + bus_index
            shunt = self.dpsimpy_components.Shunt(shunt_name, self.log_level)
            shunt.set_parameters(gs, bs)
            shunt.set_base_voltage(bus_baseV)
            self.dpsimpy_comp_dict[shunt_name] = [shunt]
            self.dpsimpy_comp_dict[shunt_name].append([self.dpsimpy_busses_dict[self.get_node_name(bus_index)]])
        else:
            # convert conductance->resistance and calculate equivalent inductance/capacitance
            r = 0
            if (gs != 0):
                r = 1 / gs
                if (self.domain==Domain.EMT):
                    r = dpsimpy.Math.single_phase_parameter_to_three_phase(r)
            shunt_name = "Shunt_Res_N" + bus_index
            shunt_res = self.dpsimpy_components.Resistor(shunt_name, self.log_level)
            shunt_res.set_parameters(r)
            self.dpsimpy_comp_dict[shunt_name] = [shunt_res]
            self.dpsimpy_comp_dict[shunt_name].append([self.dpsimpy_busses_dict[self.get_node_name(bus_index)], self.dpsimpy_components.SimNode.gnd])
            
            # convert susceptance->reactance and calculate equivalent inductance/capacitance
            x = 1 / bs
            c = 0
            l = 0
            if (bs!=0):
                # In Matpower Bs is the injected reactive power 
                if (bs>0):
                    c = 1 / (self.mpc_omega * x)
                    if (self.domain==Domain.EMT):
                        c = dpsimpy.Math.single_phase_parameter_to_three_phase(c)
                    shunt_name = "Shunt_Cap_N" + bus_index
                    shunt_cap = self.dpsimpy_components.Capacitor(shunt_name, self.log_level)
                    shunt_cap.set_parameters(c)
                    self.dpsimpy_comp_dict[shunt_name] = [shunt_cap]
                    self.dpsimpy_comp_dict[shunt_name].append([self.dpsimpy_busses_dict[self.get_node_name(bus_index)], self.dpsimpy_components.SimNode.gnd]) 
                else:
                    l = -x / self.mpc_omega
                    if (self.domain==Domain.EMT):
                        l = dpsimpy.Math.single_phase_parameter_to_three_phase(l)
                    shunt_name = "ShuntInd_N" + bus_index
                    shunt_ind = self.dpsimpy_components.Inductor(shunt_name, self.log_level)
                    shunt_ind.set_parameters(l)
                    self.dpsimpy_comp_dict[shunt_name] = [shunt_ind]
                    self.dpsimpy_comp_dict[shunt_name].append([self.dpsimpy_busses_dict[self.get_node_name(bus_index)], self.dpsimpy_components.SimNode.gnd]) 
        
    def map_network_injection(self, index, bus_index):
        
        self.inj = self.inj + 1
        extnet_name = "extnet%s" %self.inj

        # relevant data from self.mpc_gen_data. Identification with bus number available in mpc_bus_data and mpc_gen_data
        extnet = self.mpc_gen_data.loc[self.mpc_gen_data['bus'] == self.mpc_bus_data.at[index,'bus_i']]
        extnet_baseV = self.mpc_bus_data.at[index,'baseKV']*kv_v
        extnet_v = extnet['Vg']*extnet_baseV
        if (self.domain == Domain.EMT):
            extnet_v = dpsimpy.Math.single_phase_variable_to_three_phase(extnet_v)
            
        self.dpsimpy_comp_dict[extnet_name] = [self.dpsimpy_components.NetworkInjection(extnet_name, self.log_level)]
        self.dpsimpy_comp_dict[extnet_name][0].set_parameters(extnet_v)
        if (self.domain == Domain.PF):
            self.dpsimpy_comp_dict[extnet_name][0].set_base_voltage(extnet_baseV)
            self.dpsimpy_comp_dict[extnet_name][0].modify_power_flow_bus_type(dpsimpy.PowerflowBusType.VD)

        # add connections
        self.dpsimpy_comp_dict[extnet_name].append([self.dpsimpy_busses_dict[self.get_node_name(bus_index)]]) # [to bus]
                
    def create_dpsim_topology(self):
        system_comp = []
        system_nodes = []

        self.system = None
        for key, value in self.dpsimpy_comp_dict.items():
            dpsim_component = value[0]
            connection_nodes = value[1]
            dpsim_component.connect(connection_nodes)

            system_comp.append(dpsim_component)

            for n in connection_nodes:
                if n in system_nodes:
                    continue
                else:
                    system_nodes.append(n)

        self.system = dpsimpy.SystemTopology(self.mpc_freq, system_nodes, system_comp)
    
    def load_mpc(self, frequency=60, domain=Domain.PF, with_pss=True, with_avr=True, with_tg=True):
        """
        Read mpc files and create DPsim topology
        
        @param frequency: system frequency
        @param domain: domain to be used in DPsim
        """
        self.create_dpsim_objects(domain=domain, frequency=frequency, 
                                  with_pss=with_pss, with_avr=with_avr, with_tg=with_tg)
        self.create_dpsim_topology()
        
        return self.system
    
    def init_from_pf_results(self):
        """
        This function use the results stored in the busses of the mpc files to init
        the node voltages a the power of synchronous generators
        It can be used to inizialize dynamic simulations
        """
        for index, bus in self.mpc_bus_data.iterrows():
            bus_index = str(self.mpc_bus_data.at[index,'bus_i']) #index: 0....N-1, bus_index: 1...N
            node_name = self.get_node_name(bus_index)   # nonde_name="N" + bus_index
            
            # get node voltage
            voltage_mag = self.mpc_bus_data.at[index,'Vm']
            voltage_angle = self.mpc_bus_data.at[index,'Va'] * np.pi / 180
            base_voltage = self.mpc_bus_data.at[index,'baseKV']
            voltage_complex = kv_v * base_voltage * voltage_mag * complex(np.cos(voltage_angle), np.sin(voltage_angle))
            
            # search node in dpsim topology and set the initial voltage
            dpsim_node = self.system.node(node_name)
            dpsim_node.set_initial_voltage(voltage_complex)

        # initialize SG
        for index, gen in self.mpc_gen_data.iterrows():
            # get generator name in dpsim
            bus_index = self.mpc_gen_data.at[index,'bus']
            gen_name = "Gen_N" + str(bus_index)
            
            # get active and reactive power
            active_power = self.mpc_gen_data.at[index,'Pg']
            reactive_power = self.mpc_gen_data.at[index,'Qg']
            base_power = self.mpc_gen_data.at[index,'BaseS']
            complex_power = mw_w * complex(active_power, reactive_power)

            #set terminal power of generator in dpsim
            self.system.component(gen_name).get_terminal(index=0).set_power(-complex_power)
    
    def get_pf_results(self):
        pf_results = pd.DataFrame(columns=['Bus', 'Vm [pu]', 'Va [°]', 'P [MW]', 'Q [MVAr]'])
        for i in range(self.mpc_bus_data.shape[0]):
            node_name = "N" + str(self.mpc_bus_data['bus_i'][i]) #ex. N5

            p_gen = 0
            q_gen = 0
            # if PV or Slack bus --> add gen power 
            gen_data = self.mpc_gen_data.loc[self.mpc_gen_data['bus'] == self.mpc_bus_data.at[i,'bus_i']]
            if (gen_data.shape[0] > 0):
                p_gen = gen_data['Pg'].values[0]
                q_gen = gen_data['Qg'].values[0]

            pf_results.loc[i] = ([node_name] 
                         + [self.mpc_bus_data['Vm'][i]] 
                         + [self.mpc_bus_data['Va'][i]]
                         + [p_gen - self.mpc_bus_data['Pd'][i]]
                         + [q_gen - self.mpc_bus_data['Qd'][i]])
            
        return pf_results
  
    def add_3ph_faut(self, node_name, switch_closed=1e-3, switch_open=1e18):
        sw = None
        if (self.domain==Domain.PF):
            raise Exception('Faults can only be used for dynamic simulations!')
        
        fault_name = 'Fault_' + node_name
        sw = self.dpsimpy_components.Switch(fault_name, self.log_level)
        sw.set_parameters(switch_open, switch_closed)
        sw.open()
        self.dpsimpy_comp_dict[fault_name] = [sw]
        self.dpsimpy_comp_dict[fault_name].append([self.dpsimpy_components.SimNode.gnd, self.dpsimpy_busses_dict[node_name]])
        
        return sw