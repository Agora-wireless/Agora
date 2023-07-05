"""
Copyright IBM

This code is Contributed IP and is being provided  according to the limited 
rights accorded in the IBM-Columbia sub-contract agreement for the COSMOS 
Effort of the NSF PAWR program

Defines a class with various top-level methods to control the IBM Phased Array
This class inherits from the PhasedArrayControlCore class
"""

import time
import numpy as np
from PhasedArrayControlCore import PhasedArrayControlCore
import private_methods
from private_methods import ps2therm
from private_methods import closest
from private_methods import phase_repos
from private_methods import phase_map
from private_methods import get_taper_data

class PhasedArrayControl(PhasedArrayControlCore):
    """Class with various top-level methods to control the IBM Phased Array
    
    This class inherits the class PhasedArrayControlCore which has all the low-level methods for the setup and transfer of data between the PC and the Microzed
    on the IBM Phased Array board

    Initialization
    --------------
    See __init__() below
    
    Attributes
    ----------
    fast_write_reg : bool
        True by default
        A variable that improves the speed of register write commands by pushing functionality to the FPGA
        This is only functional when using the baseline FPGA IP (see set_fpga_control_ip())
    fast_read_reg : bool
        True by default
        A variable that improves the speed of register read commands by pushing functionality to the FPGA
        This is only functional when using the baseline FPGA IP (see set_fpga_control_ip())
    buffer_sends : bool
        True by default
        A variable that improves the speed of operation of any script. This buffers any commands on the PC itself to create larger bulk data transfers over
        ethernet of serial interfaced such that the overall speed is significantly improved
    verify : bool
        False by default (can be set by a parameter to the __init__() method)
        This veriable is only set to True for debug purposes. When True, all register writes to the IBM Phased Array are followed by register reads in order to
        verify correct writing to the registers. Wrong register writes are flagged. Note that this option significantly slows down operation

    See Also
    --------
    PhasedArrayControlCore

    Examples
    --------
    examples/setup_betaboard_v1.2.py : Basic operation using __init__(), enable(), steer_beam() and close_port()
    examples/setup_betaboard_v1.2_advanced.py : Advanced operation using switch_beam_index(), start_recording(), stop_recording(), playonce()

    """
    
    def _Create_cmd(self,ic,fe=0):
        """Internal method to configure the SPI command for IBM Phased Array

        This method is used internally within this class, and doesn't need to be used by most users

        Parameters
        ----------
        ic : IC for the SPI command
            Options are 'all' or integer 0,1,2,3
        fe : (optional) Frontend for this SPI command
            Options are 'all' or integer between 0 and 15

        Returns
        -------
        int
            An integer that is passed to the SPI controller on the FPGA
        """
        if ic=='all' and fe=='all':
            return 0b11000
        elif fe=='all':
            return 0b01000 + ic
        elif ic=='all':
            return 0b10000
        else:
            return ic

    def _strobe(self, cmd, flush_conn_buffer=True):
        """Internal method to strobe the SPI command into the IBM Phased Array and get a confirmation that the command has been executed
        
        This methods is used internall within this class, and doesn't need to be used by most users
        
        Parameters
        ----------
        cmd : int
            An integer to specify the SPI command type. This is to be generated using _Create_cmd()
        flush_conn_buffer : (optional) bool (default True)
            see playonce() for detailed explanation of this option

        """
        self._write_register(self.reg_dict["SDPAR"]["cmd_generate_strobe_enable_value_p"]["Address List"][0], 1, cmd, verify=False)
        if flush_conn_buffer:
            self._mr(0)

    def switch_beam_index(self,ic,pol,txrx,beam_index, flush_conn_buffer=True):
        """ Method to switch the beamindex for a given IC, polarization and txrx mode
        
        Note that the txrx is switched for both polarizations of a given IC at the same time
        
        Beam index for the other polarization is preserved to the last used value for the given txrx mode
        
        If the beam index for the other polarization also needs to be switched simultaenously, use switch_beam_indices()

        Parameters
        ----------
        ic : {'all', 0, 1, 2, 3}
            IC(s) to be configured
        pol : {'h', 'v'}
            Polarization for which the beam index is switched (Beam index for the other polarization is not changed)
        txrx : {'tx', 'rx'}
            Mode for the given ic
        beam_index : {0 to 128}
            Beam index to switch to between the 128 rows of on-chip configurations
        flush_conn_buffer : (optional) bool (default True)
            see playonce() for detailed explanation of this option

        See Also
        --------
        The beamtable must be previously configured. For this, use the following methods:
        steer_beam() : Write traditional beam to beam table. Make sure to set switch_immediate flag to False when calling this
        set_arbitrary_beam() : Write arbitraty beam to beam table. Make sure to set switch_immediate flag to False when calling this
        switch_beam_index() : Simultaenously switch beam index for both polarizations
        """
        write_register = self._write_register
        reg_dict = self.reg_dict
        verify = self.verify
        self._beamindices[txrx][pol]=beam_index
        tddbit = int(txrx=='tx')
        write_register(reg_dict["SDPAR"]["cfg_global_beam_index_tdd_p"]["Address List"][0],tddbit+2*self._beamindices[txrx]['v']+256*self._beamindices[txrx]['h'],self._Create_cmd(ic=ic),verify=verify)
        # if pol=='h':
        #     if txrx=='tx':
        #         # TDD TX
        #         write_register(reg_dict["SDPAR"]["cfg_global_beam_index_tdd_p"]["Address List"][0],1+256*beam_index,self._Create_cmd(ic=ic),verify=verify)
        #     else:
        #         # TDD RX
        #         write_register(reg_dict["SDPAR"]["cfg_global_beam_index_tdd_p"]["Address List"][0],0+256*beam_index,self._Create_cmd(ic=ic),verify=verify)
        # if pol=='v':
        #     if txrx=='tx':
        #         # TDD TX
        #         write_register(reg_dict["SDPAR"]["cfg_global_beam_index_tdd_p"]["Address List"][0],1+2*beam_index,self._Create_cmd(ic=ic),verify=verify)
        #     else:
        #         # TDD RX
        #         write_register(reg_dict["SDPAR"]["cfg_global_beam_index_tdd_p"]["Address List"][0],0+2*beam_index,self._Create_cmd(ic=ic),verify=verify)
        # Generate internal strobe
        self._strobe(self._Create_cmd(ic=ic),flush_conn_buffer=flush_conn_buffer)

    def switch_beam_indices(self,ic,txrx,beam_index_h,beam_index_v,flush_conn_buffer=True):
        """ Method to simulateneously switch the beamindices for both polarizations of a given IC, for a given txrx mode
        
        If the beam index for only one polarization needs to be switched, use switch_beam_index()

        Parameters
        ----------
        ic : {'all', 0, 1, 2, 3}
            IC(s) to be configured
        txrx : {'tx', 'rx'}
            Mode for the given ic
        beam_index_h : {0 to 128}
            h-polarization Beam index to switch to between the 128 rows of on-chip configurations
        beam_index_v : {0 to 128}
            v-polarization Beam index to switch to between the 128 rows of on-chip configurations
        flush_conn_buffer : (optional) bool (default True)
            see playonce() for detailed explanation of this option

        See Also
        --------
        The beamtable must be previously configured. For this, use the following methods:
        steer_beam() : Write traditional beam to beam table. Make sure to set switch_immediate flag to False when calling this
        set_arbitrary_beam() : Write arbitraty beam to beam table. Make sure to set switch_immediate flag to False when calling this
        switch_beam_index() : Switch beam index for only one polarization
        """
        write_register = self._write_register
        reg_dict = self.reg_dict
        verify = self.verify
        self._beamindices[txrx]['h']=beam_index_h
        self._beamindices[txrx]['v']=beam_index_v
        tddbit = int(txrx=='tx')
        write_register(reg_dict["SDPAR"]["cfg_global_beam_index_tdd_p"]["Address List"][0],tddbit+2*self._beamindices[txrx]['v']+256*self._beamindices[txrx]['h'],self._Create_cmd(ic=ic),verify=verify)
        # Generate internal strobe
        self._strobe(self._Create_cmd(ic=ic),flush_conn_buffer=flush_conn_buffer)

    def _enable_fe(self,ic,fe_list,pol,txrx,bias=[8,8],flush_conn_buffer=True):
        """Internal method to enable a set of frontends. Please use enable() instead

        This method is used internally within this class, and doesn't need to be used by most users. This doesn't configure all the required registers by
        itself.
        
        Parameters
        ----------
        Refer to enable() for detailed explanation of parameters

        See Also
        --------
        enable() : Fully featured method to enable frontends in a given IC, polarization, txrx mode

        """
        if bias==0:
            pa_bias=[10,10]
            lna_bias=[8,8]
        else:
            pa_bias=bias
            lna_bias=bias
        if len(pa_bias)==2:
            pa_bias=[15&pa_bias[0],0,15&pa_bias[1], 1^(pa_bias[0]>>4),1,1^(pa_bias[1]>>4)]
        elif len(pa_bias)==4:
            pa_bias=[pa_bias[0],0,pa_bias[1], pa_bias[2], 1, pa_bias[3]]
        verify = self.verify
        write_register = self._write_register
        #read_register = self._read_register
        reg_dict = self.reg_dict
        fe_base_addr_p = self.fe_base_addr_p
        fe_offset = self.fe_offset
        write_register(reg_dict["SDPAR"]["cfg_fe_0_rffe_idle_ps_switch_pwrdn_v_p"]["Address List"][0],0,self._Create_cmd(ic=ic,fe='all'),verify=verify)
        if pol=='h':
            # Disabe all fe:s using FE broadcast
            write_register(reg_dict["SDPAR"]["ctx_fe_0_pwrdn_rx_h_rffe_bias_pwrdn_h_p"]["Address List"][0],'h1f',self._Create_cmd(ic=ic,fe='all'),verify=verify)
            write_register(reg_dict["SDPAR"]["crx_fe_0_pwrdn_tx_h_rffe_pa_fast_pwrdn_h_p"]["Address List"][0],'h1f',self._Create_cmd(ic=ic,fe='all'),verify=verify)
            if txrx=='tx':
                # PA bias
                write_register(reg_dict["SDPAR"]["cfg_fe_0_bias_ctrl_tx_h_1_rffe_pa_stage1_bias_h_p"]["Address List"][0], sum([x*y for x,y in zip(pa_bias[0:3],[1, 16, 256])]),self._Create_cmd(ic=ic,fe='all'),verify=verify)
                # Chicken bits
                write_register(reg_dict["SDPAR"]["cfg_fe_0_spare_h_spare_h_p"]["Address List"][0],sum([x*y for x,y in zip(pa_bias[3:6],[4, 2, 1])]),self._Create_cmd(ic=ic,fe='all'),verify=False) # Registers write-only
                # Set TDD PA switches to TX mode using FE broadcast
                write_register(reg_dict["SDPAR"]["crx_fe_0_pwrdn_tx_h_rffe_pa_fast_pwrdn_h_p"]["Address List"][0],int('00111',2),self._Create_cmd(ic=ic,fe='all'),verify=verify)
                for fe in fe_list:
                    # Enable tx_h fes
                    write_register(reg_dict["SDPAR"]["crx_fe_0_pwrdn_tx_h_rffe_pa_fast_pwrdn_h_p"]["Address List"][0]+fe*fe_offset, 'h0',self._Create_cmd(ic=ic),verify=verify)
                    # Enable bias for tx_h fe
                    write_register(reg_dict["SDPAR"]["ctx_fe_0_pwrdn_rx_h_rffe_bias_pwrdn_h_p"]["Address List"][0]+fe*fe_offset, 'h000f',self._Create_cmd(ic=ic),verify=verify)
            else:  # Assume RX mode
                # Lna bias
                write_register(reg_dict["SDPAR"]["cfg_fe_0_bias_ctrl_rx_h_rffe_lna_stage2_bias_h_p"]["Address List"][0],sum([x*y for x,y in zip(lna_bias,[1, 16])]),self._Create_cmd(ic=ic,fe='all'),verify=verify)
                # Set TDD Lna and PS switches to RX mode using FE broadcast
                write_register(reg_dict["SDPAR"]["ctx_fe_0_pwrdn_rx_h_rffe_bias_pwrdn_h_p"]["Address List"][0],int('10011',2),self._Create_cmd(ic=ic,fe='all'),verify=verify)
                for fe in fe_list:
                    write_register(reg_dict["SDPAR"]["ctx_fe_0_pwrdn_rx_h_rffe_bias_pwrdn_h_p"]["Address List"][0]+fe*fe_offset, 0,self._Create_cmd(ic=ic),verify=verify)
        elif pol=='v':# V-polarization?
            # Disabe all fe:s using FE broadcast
            write_register(reg_dict["SDPAR"]["ctx_fe_0_pwrdn_rx_v_rffe_bias_pwrdn_v_p"]["Address List"][0],'h1f',self._Create_cmd(ic=ic,fe='all'),verify=verify)
            write_register(reg_dict["SDPAR"]["crx_fe_0_pwrdn_tx_v_rffe_pa_fast_pwrdn_v_p"]["Address List"][0],'h1f',self._Create_cmd(ic=ic,fe='all'),verify=verify)
            if txrx=='tx':
                # PA bias
                write_register(reg_dict["SDPAR"]["cfg_fe_0_bias_ctrl_tx_v_1_rffe_pa_stage1_bias_v_p"]["Address List"][0], sum([x*y for x,y in zip(pa_bias[0:3],[1, 16, 256])]),self._Create_cmd(ic=ic,fe='all'),verify=verify)
                # Chicken bits
                write_register(reg_dict["SDPAR"]["cfg_fe_0_spare_v_spare_v_p"]["Address List"][0],sum([x*y for x,y in zip(pa_bias[3:6],[4, 2, 1])]),self._Create_cmd(ic=ic,fe='all'),verify=False) # Registers write-only
                # Set TDD PA switches to TX mode using FE broadcast
                write_register(reg_dict["SDPAR"]["crx_fe_0_pwrdn_tx_v_rffe_pa_fast_pwrdn_v_p"]["Address List"][0],int('00111',2),self._Create_cmd(ic=ic,fe='all'),verify=verify)
                # Enable tx_v fes
                for fe in fe_list:
                    write_register(reg_dict["SDPAR"]["crx_fe_0_pwrdn_tx_v_rffe_pa_fast_pwrdn_v_p"]["Address List"][0]+fe*fe_offset, 'h0',self._Create_cmd(ic=ic),verify=verify)
                    # Enable bias for tx_v fe
                    write_register(reg_dict["SDPAR"]["ctx_fe_0_pwrdn_rx_v_rffe_bias_pwrdn_v_p"]["Address List"][0]+fe*fe_offset, 'h000f',self._Create_cmd(ic=ic),verify=verify)
            else: # Assume RX mode
                # Lna bias
                write_register(reg_dict["SDPAR"]["cfg_fe_0_bias_ctrl_rx_v_rffe_lna_stage2_bias_v_p"]["Address List"][0],sum([x*y for x,y in zip(lna_bias,[1, 16])]),self._Create_cmd(ic=ic,fe='all'),verify=verify)
                # Set TDD Lna and PS switches to RX mode using FE broadcast
                write_register(reg_dict["SDPAR"]["ctx_fe_0_pwrdn_rx_v_rffe_bias_pwrdn_v_p"]["Address List"][0],int('10011',2),self._Create_cmd(ic=ic,fe='all'),verify=verify)
                for fe in fe_list:
                    write_register(reg_dict["SDPAR"]["ctx_fe_0_pwrdn_rx_v_rffe_bias_pwrdn_v_p"]["Address List"][0]+fe*fe_offset, 0,self._Create_cmd(ic=ic),verify=verify)
        # Switch beam and generate internal strobe                    
        self.switch_beam_index(ic,pol,txrx,beam_index=self._beamindices[txrx][pol],flush_conn_buffer=flush_conn_buffer)

    def enable(self,ic,fe_list,pol,txrx,bias=[8,8],flush_conn_buffer=True):
        """Method to enable a set of frontends for a given IC(s) and polarization and txrx mode

        This method can be used in the initialization stage of the Phased Array
        
        This method can be run 4 separate times to set different configurations to the 4 ICs, or just once with the ic='all' parameter for faster
        configuration

        This method can also be used for TX/RX switching. For TX/RX mode switching, the used might also need to use switch_beam_index(), switch_beam_indices()
        methods to set beam index, or steer_beam() / set_arbitrary_beam() methods to set a new beam configuration. When TRX switching, if the Phased Array is
        being used for both polarizations simultaneously, please run this method twice once for each polarization, followed by switch_beam_indices().

        Parameters
        ----------
        ic : {'all', 0, 1, 2, 3}
            IC(s) to be enabled. Use 'all' to configure all 4 ICs in the Phased Array to the same settings
        fe_list : array
            Array of front-ends to be enabled. (Values 0 through 15 are valid items in this array)
            Examples: range(16) for all frontends
        pol : {'h', 'v'}
            Polarization that is configured in this enable method.
        txrx : {'tx', 'rx'}
            TXRX mode configuration for this IC(s)
        flush_conn_buffer : (optional) bool (default True)
            see playonce() for detailed explanation of this option

        Returns
        -------
        int or array
            This return value indicates an internal state of a phase detector in the IC. However, this return value is not meaningful and can be ignored.

        Other parameters
        ----------------
        bias : optional
            This is bias settings for the PA (tx mode) and LNA (rx mode) in each frontend.

        See Also
        --------
        switch_beam_index() : This needs to be run after, if the beam_index needs to be updated
        switch_beam_indices() : This needs to be run is beam indices for both polarizations need to be updated
        steer_beam() : Configure the direction and tapering for the beam (not using a previously stored beam in the beamtable)
        set_arbitrary_beam() : Configure an arbitrary beam (phase and gain per element), if the configuration is not already in beamtable
        
        """
        if bias ==0:
            pa_bias=[10, 10]
            lna_bias=[8, 8]
        else:
            pa_bias=bias
            lna_bias=bias
        verify=self.verify
        write_register = self._write_register
        read_register = self._read_register
        reg_dict = self.reg_dict
        fe_base_addr_p = self.fe_base_addr_p
        fe_offset = self.fe_offset
        enable_fe = self._enable_fe
        if ic=='all':
            if self._enabled_ics != [1,1,1,1]:
                enabled_previously = False
                self._enabled_ics = [1,1,1,1]
            else:
                enabled_previously = True
        else:
            if self._enabled_ics[ic] == 1:
                enabled_previously = True
            else:
                enabled_previously = False
                self._enabled_ics[ic] = 1
        if not enabled_previously:
            # print('Not enabled previously. So, initializing IC: %s'%(ic))
            # Override settings
            # Set all trx override bits to disable automatic enabling using broadcast
            write_register(reg_dict["SDPAR"]["cfg_en_tx_v_1_override_en_tx_rfmix_core_1v_p"]["Address List"][0],int('7ff',16),self._Create_cmd(ic='all'),verify=verify)
            write_register(reg_dict["SDPAR"]["cfg_en_tx_h_1_override_en_tx_rfmix_core_1h_p"]["Address List"][0],int('7ff',16),self._Create_cmd(ic='all'),verify=verify)
            write_register(reg_dict["SDPAR"]["cfg_en_rx_v_1_override_en_rx_rfmix_core_1v_p"]["Address List"][0],int('7fff',16),self._Create_cmd(ic='all'),verify=verify)
            write_register(reg_dict["SDPAR"]["cfg_en_rx_h_1_override_en_rx_rfmix_core_1h_p"]["Address List"][0],int('7fff',16),self._Create_cmd(ic='all'),verify=verify)
            write_register(reg_dict["SDPAR"]["cfg_en_tx_v_2_override_en_tx_if_dc_v_p"]["Address List"][0],int('1fff',16),self._Create_cmd(ic='all'),verify=verify)
            write_register(reg_dict["SDPAR"]["cfg_en_tx_h_2_override_en_tx_if_dc_h_p"]["Address List"][0],int('1fff',16),self._Create_cmd(ic='all'),verify=verify)
            write_register(reg_dict["SDPAR"]["cfg_en_rx_v_2_override_en_rx_rfamp_1v_p"]["Address List"][0],int('f',16),self._Create_cmd(ic='all'),verify=verify)
            write_register(reg_dict["SDPAR"]["cfg_en_rx_h_2_override_en_rx_rfamp_1h_p"]["Address List"][0],int('f',16),self._Create_cmd(ic='all'),verify=verify)
            write_register(reg_dict["SDPAR"]["cfg_sel_txrx_override_sel_txrx_1v_p"]["Address List"][0],int('f',16),self._Create_cmd(ic='all'),verify=verify)
            # Set all override bits for fe:s using broadcast (IC+FE)
            write_register(reg_dict["SDPAR"]["cfg_fe_0_pwrdn_rx_v_override_rffe_bias_pwrdn_v_p"]["Address List"][0],int('1f',16), self._Create_cmd(ic='all',fe='all'),verify=verify)
            write_register(reg_dict["SDPAR"]["cfg_fe_0_pwrdn_rx_h_override_rffe_bias_pwrdn_h_p"]["Address List"][0],int('1f',16), self._Create_cmd(ic='all',fe='all'),verify=verify)
            write_register(reg_dict["SDPAR"]["cfg_fe_0_pwrdn_tx_v_override_rffe_ant_pa_switch_1_v_p"]["Address List"][0],int('1f',16), self._Create_cmd(ic='all',fe='all'),verify=verify)
            write_register(reg_dict["SDPAR"]["cfg_fe_0_pwrdn_tx_h_override_rffe_ant_pa_switch_1_h_p"]["Address List"][0],int('1f',16), self._Create_cmd(ic='all',fe='all'),verify=verify)
            # Use external resistor for reference
            write_register(reg_dict["SDPAR"]["cfg_trx_misc_sel_ext_bg_res_p"]["Address List"][0],int('11100000000',2),self._Create_cmd(ic=ic),verify=verify)
            # Enable fe bias
            write_register(reg_dict["SDPAR"]["cfg_fe_misc_en_fe_bg_p"]["Address List"][0],3,self._Create_cmd(ic=ic),verify=verify)
            # Set sbit slope (IC+FE broadcast)
            write_register(reg_dict["SDPAR"]["cfg_fe_0_phase_shift_config_sbit_slope_tx_p"]["Address List"][0],int('100100',2),self._Create_cmd(ic='all',fe='all'),verify=verify)
            # Set trx_tx_dc gain to nominal (0.2dB)
            write_register(reg_dict["SDPAR"]["cfg_gain_tx_dc_gain_tx_if_ampint_h_p"]["Address List"][0],int('8888',16),self._Create_cmd(ic=ic),verify=verify)
            write_register(reg_dict["SDPAR"]["cfg_gain_tx_dc_gain_tx_if_ampint_v_p"]["Address List"][0],int('8888',16),self._Create_cmd(ic=ic),verify=verify)
            # Set trx_rx_if gain to nominal (7.5dB)
            write_register(reg_dict["SDPAR"]["cfg_gain_rx_dc_gain_rx_if_ampint_h_p"]["Address List"][0],int('dddd',16),self._Create_cmd(ic=ic),verify=verify)
            write_register(reg_dict["SDPAR"]["cfg_gain_rx_dc_gain_rx_if_ampint_v_p"]["Address List"][0],int('dddd',16),self._Create_cmd(ic=ic),verify=verify)
        if pol == 'h':
            if txrx =='tx':
                # Disable rx_h
                write_register(reg_dict["SDPAR"]["crx_en_rx_h_1_en_rx_rfmix_core_1h_p"]["Address List"][0],0,self._Create_cmd(ic=ic),verify=verify)
                write_register(reg_dict["SDPAR"]["crx_en_rx_h_2_en_rx_rfamp_core_0h_p"]["Address List"][0],0,self._Create_cmd(ic=ic),verify=verify)
                # Enable trx tx_h. Daisy chain disabled.
                write_register(reg_dict["SDPAR"]["ctx_en_tx_h_1_en_tx_rfmix_core_1h_p"]["Address List"][0],int('7ff',16),self._Create_cmd(ic=ic),verify=verify)
                write_register(reg_dict["SDPAR"]["ctx_en_tx_h_2_en_tx_ifmix_bias_h_p"]["Address List"][0],int('1011111111111',2),self._Create_cmd(ic=ic),verify=verify)
                # sel_txrx	sel_txrx_1v	 '1' = TX / '0' = RX	3	3
                # sel_txrx	sel_txrx_0v	 '1' = TX / '0' = RX	2	2
                # sel_txrx	sel_txrx_1h	 '1' = TX / '0' = RX	1	1
                # sel_txrx	sel_txrx_0h	 '1' = TX / '0' = RX	0	0
                # Read trx txrx switch
                # sw_status=read_register(reg_dict["SDPAR"]["ctx_sel_txrx_sel_txrx_1v_p"]["Address List"][0],ic)
                # Set trx txrx switch suitably.
                # write_register(reg_dict["SDPAR"]["ctx_sel_txrx_sel_txrx_1v_p"]["Address List"][0],(sw_status & (int('1100',2))) | (int('0011',2)),self._Create_cmd(ic=ic),verify=verify)
                write_register(reg_dict["SDPAR"]["ctx_sel_txrx_sel_txrx_1v_p"]["Address List"][0],int('1111',2),self._Create_cmd(ic=ic),verify=verify)
            else: # Assume RX mode
                # Disable tx_h
                write_register(reg_dict["SDPAR"]["ctx_en_tx_h_1_en_tx_rfmix_core_1h_p"]["Address List"][0],0,self._Create_cmd(ic=ic),verify=verify)
                write_register(reg_dict["SDPAR"]["ctx_en_tx_h_2_en_tx_ifmix_bias_h_p"]["Address List"][0],0,self._Create_cmd(ic=ic),verify=verify)
                # Enable trx rx_h. Daisy chain disabled.
                write_register(reg_dict["SDPAR"]["crx_en_rx_h_1_en_rx_rfmix_core_1h_p"]["Address List"][0],'b111111111111011',self._Create_cmd(ic=ic),verify=verify)
                write_register(reg_dict["SDPAR"]["crx_en_rx_h_2_en_rx_rfamp_core_0h_p"]["Address List"][0],'b1111',self._Create_cmd(ic=ic),verify=verify)
                # Read trx txrx switch
                # sw_status=read_register(reg_dict["SDPAR"]["ctx_sel_txrx_sel_txrx_1v_p"]["Address List"][0],ic)
                # Set trx txrx switch suitably.
                # write_register(reg_dict["SDPAR"]["ctx_sel_txrx_sel_txrx_1v_p"]["Address List"][0], sw_status & (int('1100',2)),self._Create_cmd(ic=ic),verify=verify)
                write_register(reg_dict["SDPAR"]["ctx_sel_txrx_sel_txrx_1v_p"]["Address List"][0],int('0000',2),self._Create_cmd(ic=ic),verify=verify)
        elif pol =='v': # V-polarization?
            if txrx == 'tx':
                # Disable rx_h
                write_register(reg_dict["SDPAR"]["crx_en_rx_v_1_en_rx_rfmix_core_1v_p"]["Address List"][0],0,self._Create_cmd(ic=ic),verify=verify)
                write_register(reg_dict["SDPAR"]["crx_en_rx_v_2_en_rx_rfamp_core_0v_p"]["Address List"][0],0,self._Create_cmd(ic=ic),verify=verify)
                # Enable trx tx_v. Daisy chain disabled.
                write_register(reg_dict["SDPAR"]["ctx_en_tx_v_1_en_tx_rfmix_core_1v_p"]["Address List"][0],int('7ff',16),self._Create_cmd(ic=ic),verify=verify)
                write_register(reg_dict["SDPAR"]["ctx_en_tx_v_2_en_tx_ifmix_bias_v_p"]["Address List"][0],int('1011111111111',2),self._Create_cmd(ic=ic),verify=verify)
                # sel_txrx	sel_txrx_1v	 '1' = TX / '0' = RX	3	3
                # sel_txrx	sel_txrx_0v	 '1' = TX / '0' = RX	2	2
                # sel_txrx	sel_txrx_1h	 '1' = TX / '0' = RX	1	1
                # sel_txrx	sel_txrx_0h	 '1' = TX / '0' = RX	0	0
                # Read trx txrx switch
                # sw_status=read_register(reg_dict["SDPAR"]["ctx_sel_txrx_sel_txrx_1v_p"]["Address List"][0],ic)
                # Set trx txrx switch suitably.
                # write_register(reg_dict["SDPAR"]["ctx_sel_txrx_sel_txrx_1v_p"]["Address List"][0],( ( sw_status & (int('0011',2)) ) | (int('1100',2)) ),self._Create_cmd(ic=ic),verify=verify)
                write_register(reg_dict["SDPAR"]["ctx_sel_txrx_sel_txrx_1v_p"]["Address List"][0],int('1111',2),self._Create_cmd(ic=ic),verify=verify)
            else: # Assume RX mode
                # Disable tx_v.
                write_register(reg_dict["SDPAR"]["ctx_en_tx_v_1_en_tx_rfmix_core_1v_p"]["Address List"][0],0,self._Create_cmd(ic=ic),verify=verify)
                write_register(reg_dict["SDPAR"]["ctx_en_tx_v_2_en_tx_ifmix_bias_v_p"]["Address List"][0],0,self._Create_cmd(ic=ic),verify=verify)
                # Enable trx rx_h. Daisy chain disabled.
                write_register(reg_dict["SDPAR"]["crx_en_rx_v_1_en_rx_rfmix_core_1v_p"]["Address List"][0],int('111111111111011',2),self._Create_cmd(ic=ic),verify=verify)
                write_register(reg_dict["SDPAR"]["crx_en_rx_v_2_en_rx_rfamp_core_0v_p"]["Address List"][0],int('1111',2),self._Create_cmd(ic=ic),verify=verify)
                # Read trx txrx switch
                # sw_status=read_register(reg_dict["SDPAR"]["ctx_sel_txrx_sel_txrx_1v_p"]["Address List"][0],ic)
                # Set trx txrx switch suitably.
                # write_register(reg_dict["SDPAR"]["ctx_sel_txrx_sel_txrx_1v_p"]["Address List"][0],(sw_status & (int('0011',2))),self._Create_cmd(ic=ic),verify=verify)
                write_register(reg_dict["SDPAR"]["ctx_sel_txrx_sel_txrx_1v_p"]["Address List"][0],int('0000',2),self._Create_cmd(ic=ic),verify=verify)

        if not enabled_previously:
            # Enable LO (not lo daisy chain amplifier), bias, lodist (x1 dist disabled)
            write_register(reg_dict["SDPAR"]["cfg_en_lo_en_trx_bias_p"]["Address List"][0],int('1011111100',2),self._Create_cmd(ic=ic),verify=verify)
            write_register(reg_dict["SDPAR"]["cfg_en_lodist_en_lo_dist_p"]["Address List"][0],int('0ccd',16),self._Create_cmd(ic=ic),verify=verify)
            # Fiddling with lox5 leakage
            # write_register(reg_dict["SDPAR"]["cfg_ctrl_lo_ctrl_lo_buf_5ghz_p"]["Address List"][0],int('f3ff',16),self._Create_cmd(ic=ic),verify=verify)
            if ic=='all':
                ics=[0,1,2,3]
            else:
                ics=[ic]
            self.steer_beam(pol=pol,txrx=txrx,theta=0,phi=0,taper=('None', -13),ps_v=[16,16,16,16],pi_v=[0,1,1,0],ics=ics,beam_index=0,switch_immediate=False,verbose=False)

        if txrx == 'tx':
            enable_fe(ic,fe_list,pol,txrx,pa_bias,flush_conn_buffer=flush_conn_buffer)
        else:
            enable_fe(ic,fe_list,pol,txrx,lna_bias,flush_conn_buffer=flush_conn_buffer)

        det_phases = 0
        if not enabled_previously:
            # Generate internal strobe
            write_register(reg_dict["SDPAR"]["cmd_generate_strobe_enable_value_p"]["Address List"][0], 1,self._Create_cmd(ic=ic),verify=False)
            # Lock phase detector when initializing
            # Enable x1 distribution
            write_register(reg_dict["SDPAR"]["cfg_en_lodist_en_lo_dist_p"]["Address List"][0],int('3fff',16),self._Create_cmd(ic=ic),verify=verify)
            # Generate internal strobe
            write_register(reg_dict["SDPAR"]["cmd_generate_strobe_enable_value_p"]["Address List"][0], 1,self._Create_cmd(ic=ic),verify=False)
            # Now set to automatic lock
            write_register(reg_dict["SDPAR"]["cfg_phase_det_ctrl_en_phase_det_lock_p"]["Address List"][0],int('100000000',2),self._Create_cmd(ic=ic),verify=verify)
            # Generate internal strobe
            write_register(reg_dict["SDPAR"]["cmd_generate_strobe_enable_value_p"]["Address List"][0], 1,self._Create_cmd(ic=ic),verify=False)
            # Detect Phase Lock
            write_register(reg_dict["SDPAR"]["cmd_phase_det_lock_phase_det_lock_p"]["Address List"][0],1,self._Create_cmd(ic=ic),verify=False)
            # Generate internal strobe
            write_register(reg_dict["SDPAR"]["cmd_generate_strobe_enable_value_p"]["Address List"][0], 1,self._Create_cmd(ic=ic),verify=False)
            self._mr(0)
            time.sleep(0.001)
            # Read detected phases
            if ic=='all':
                det_phases = [0,0,0,0]
                for i in range(4):
                    det_phases[i]=read_register(reg_dict["SDPAR"]["sts_phase_det_rx_ifmix_lopd_phase_h_p"]["Address List"][0],i)
            else:
                det_phases=read_register(reg_dict["SDPAR"]["sts_phase_det_rx_ifmix_lopd_phase_h_p"]["Address List"][0],ic)
            # Disable x1 distribution
            write_register(reg_dict["SDPAR"]["cfg_en_lodist_en_lo_dist_p"]["Address List"][0],int('0ccd',16),self._Create_cmd(ic=ic),verify=verify)
            # Generate internal strobe and do a self._mr(0)
            self._strobe(cmd=self._Create_cmd(ic=ic))
        return det_phases

    def _write_sram(self,ic,fe,pol,txrx,st1_bias,st2_bias,sw,phase_sh,beam_index=0,max_flat=0):
        """Internal method to write data to the SRAM of an IC. This method is not intended for most users. set_arbitrary_beam() or steer_beam() should be used instead.

        """
        verify=self.verify
        write_register = self._write_register
        reg_dict = self.reg_dict
        fe_base_addr_p = self.fe_base_addr_p
        fe_offset = self.fe_offset
        word=st1_bias*2048+st2_bias*128+sw*64+phase_sh
        # Disable sram direct and set sbit_slope=4
        if not self.maxflat_configured:
            write_register(reg_dict["SDPAR"]["cfg_fe_0_phase_shift_config_sbit_slope_tx_p"]["Address List"][0],int('100100',2),self._Create_cmd(ic='all',fe='all'),verify=verify)
            self.maxflat_configured = True
        if txrx == 'tx':
            if pol == 'h':
                write_register(fe_base_addr_p+fe_offset*fe+reg_dict["SDPAR"]["rffe_tx_pivga_stage1_bias_h_p"]["Address List"][0]+beam_index,word,self._Create_cmd(ic=ic),verify=verify)
            else: # Assume v
                write_register(fe_base_addr_p+fe_offset*fe+reg_dict["SDPAR"]["rffe_tx_pivga_stage1_bias_v_p"]["Address List"][0]+beam_index,word,self._Create_cmd(ic=ic),verify=verify)
        else: 
            if pol == 'h':
                write_register(fe_base_addr_p+fe_offset*fe+reg_dict["SDPAR"]["rffe_rx_pivga_stage1_bias_h_p"]["Address List"][0]+beam_index,word,self._Create_cmd(ic=ic),verify=verify)
            else: #Assume v
                write_register(fe_base_addr_p+fe_offset*fe+reg_dict["SDPAR"]["rffe_rx_pivga_stage1_bias_v_p"]["Address List"][0]+beam_index,word,self._Create_cmd(ic=ic),verify=verify)
        if max_flat:
            # Use direct phase shifter control and control s-bits in such a way as
            # to achieve maximal amplitude flatness
            # Encode to thermometer code
            ps_th=ps2therm(phase_sh)
            # Set sbit slope+sram override
            write_register(reg_dict["SDPAR"]["cfg_fe_0_phase_shift_config_sbit_slope_tx_p"]["Address List"][0]+fe_offset*fe,int('1100100',2),self._Create_cmd(ic=ic),verify=verify)
            self.maxflat_configured = False
            if pol =='h':
                write_register(reg_dict["SDPAR"]["cfg_fe_0_ps_direct_h_0_ps_direct_h_0_p"]["Address List"][0]+fe_offset*fe,int(ps_th[-16:-1],2),self._Create_cmd(ic=ic),verify=verify)
                write_register(reg_dict["SDPAR"]["cfg_fe_0_ps_direct_h_1_ps_direct_h_1_p"]["Address List"][0]+fe_offset*fe,int(ps_th[-32:-17],2),self._Create_cmd(ic=ic),verify=verify)
                write_register(reg_dict["SDPAR"]["cfg_fe_0_ps_direct_h_2_ps_direct_h_2_p"]["Address List"][0]+fe_offset*fe,int(ps_th[5:-1]+ps_th[1:-33],2),self._Create_cmd(ic=ic),verify=verify)
                write_register(reg_dict["SDPAR"]["cfg_fe_0_ps_direct_h_3_ps_direct_h_3_p"]["Address List"][0]+fe_offset*fe,int(ps_th[-22:-7],2),self._Create_cmd(ic=ic),verify=verify)
                write_register(reg_dict["SDPAR"]["cfg_fe_0_ps_direct_h_4_ps_direct_h_4_p"]["Address List"][0]+fe_offset*fe,int(ps_th[-38:-23],2),self._Create_cmd(ic=ic),verify=verify)
                write_register(reg_dict["SDPAR"]["cfg_fe_0_ps_direct_h_5_ps_direct_h_5_p"]["Address List"][0]+fe_offset*fe,int(ps_th[1:-39],2),self._Create_cmd(ic=ic),verify=verify)
            else: # Assume v
                write_register(reg_dict["SDPAR"]["cfg_fe_0_ps_direct_v_0_ps_direct_v_0_p"]["Address List"][0]+fe_offset*fe,int(ps_th[-16:-1],2),self._Create_cmd(ic=ic),verify=verify)
                write_register(reg_dict["SDPAR"]["cfg_fe_0_ps_direct_v_1_ps_direct_v_1_p"]["Address List"][0]+fe_offset*fe,int(ps_th[-32:-17],2),self._Create_cmd(ic=ic),verify=verify)
                write_register(reg_dict["SDPAR"]["cfg_fe_0_ps_direct_v_2_ps_direct_v_2_p"]["Address List"][0]+fe_offset*fe,int(ps_th[-6:-1]+ps_th[1:-33],2),self._Create_cmd(ic=ic),verify=verify)
                write_register(reg_dict["SDPAR"]["cfg_fe_0_ps_direct_v_3_ps_direct_v_3_p"]["Address List"][0]+fe_offset*fe,int(ps_th[-22:-7],2),self._Create_cmd(ic=ic),verify=verify)
                write_register(reg_dict["SDPAR"]["cfg_fe_0_ps_direct_v_4_ps_direct_v_4_p"]["Address List"][0]+fe_offset*fe,int(ps_th[-38:-23],2),self._Create_cmd(ic=ic),verify=verify)
                write_register(reg_dict["SDPAR"]["cfg_fe_0_ps_direct_v_5_ps_direct_v_5_p"]["Address List"][0]+fe_offset*fe,int(ps_th[1:-39],2),self._Create_cmd(ic=ic),verify=verify)
            # Generate internal strobe
            write_register(reg_dict["SDPAR"]["cmd_generate_strobe_enable_value_p"]["Address List"][0],1,self._Create_cmd(ic=ic),verify=False)

    def set_arbitrary_beam(self,pol,txrx,phase_settings,taper_settings,ps_v=[16,16,16,16],pi_v=[0,1,1,0],ics=[0],beam_index=0,switch_immediate=True,verbose=False):
        """Advanced use function to set arbitrary beam patterns by setting the phase and gain for each element separately.

        For simpler use cases of pointing a beam at a given direction with or without Taylor tapering, please use steer_beam()
        
        Parameters (different from those in steer_beam())
        ----------
        phase_settings : 8x8 array of floats
            This has to be a 8x8 array of floating point phases in degrees. Quantization and mapping to the phase shifter is internal to this method
            As an example, the function private_methods.phase_map() is used by steer_beam() to generate this
            Even if only 1 IC is used, this array should be an 8x8 one for this function to work properly. Fill in zeros as needed. The ICs assigned within the
            8x8 array is given by the shape [[IC0,IC1],[IC3,IC2]]. See below for a more detailed description
        taper_settings : 8x8 array of floats
            This has to be a 8x8 array of floating point gains in dB. Quantization and mapping to VGA settings is internal to this method
            Values from [-12,0] are valid entried in each element of the array
            As an example, the function private_methods.get_taper_data() is used by steer_beam() to generate this
            Similar to above, the 8x8 array is needed even if only 1 IC is configured. Fill in zeros as needed
        
        Note that if certain frontends are not enabled using the enable() method, then the corresponding frontends will not transmit/receive energy. So, if a
        user needs to create a beam using 9 elements, they would need to turn-on the corresponding frontends, and then create a taper_settings and
        phase_settings array of size 8x8 (values for non-enabled frontends dont matter, and values for ICs not referenced in this method call don't matter, but
        the full 8x8x array still needs to be provided for the set_arbitrary_beam() method to do the correct mapping).

        Also note that the mapping of Front-ends and ICs in the 8x8 array is given below. Users can use this to turn on the corresponding frontends and ICs to
        get the desired sub-array shape:

        IC map in the 8x8 array
        -----------------------
        [0, 0, 0, 0, 1, 1, 1, 1,
         0, 0, 0, 0, 1, 1, 1, 1,
         0, 0, 0, 0, 1, 1, 1, 1,
         0, 0, 0, 0, 1, 1, 1, 1,
         3, 3, 3, 3, 2, 2, 2, 2,
         3, 3, 3, 3, 2, 2, 2, 2,
         3, 3, 3, 3, 2, 2, 2, 2,
         3, 3, 3, 3, 2, 2, 2, 2]

        Front-end map in the 8x8 array
        ------------------------------
        [15, 14,  6,  7, 15, 14,  6,  7,
         13, 12,  4,  5, 13, 12,  4,  5,
         10, 11,  3,  2, 10, 11,  3,  2,
          8,  9,  1,  0,  8,  9,  1,  0,
          0,  1,  9,  8,  0,  1,  9,  8,
          2,  3, 11, 10,  2,  3, 11, 10,
          5,  4, 12, 13,  5,  4, 12, 13,
          7,  6, 14, 15,  7,  6, 14, 15]

        See Also
        --------
        steer_beam() : Simpler interface with pointing direction and Taylor tapering parameters. Internally, set_arbitrary_beam() is called

        """
        write_sram = self._write_sram
        #    use tuples  instead of list for the followin variable because 1. they seem to be constant, 2. they are all same types
        cmap=[0,0,0,0,1,1,1,1,
              0,0,0,0,1,1,1,1,
              0,0,0,0,1,1,1,1,
              0,0,0,0,1,1,1,1,
              3,3,3,3,2,2,2,2,
              3,3,3,3,2,2,2,2,
              3,3,3,3,2,2,2,2,
              3,3,3,3,2,2,2,2]
        amap=[15,14,6,7,15,14,6,7,
              13,12,4,5,13,12,4,5,
              10,11,3,2,10,11,3,2,
              8,9,1,0,8,9,1,0,
              0,1,9,8,0,1,9,8,
              2,3,11,10,2,3,11,10,
              5,4,12,13,5,4,12,13,
              7,6,14,15,7,6,14,15]

        # correct pi_v_flip for PAWR board routing
        pi_v_flip = [0,0,0,0]
        if self.board_name=='pawr_beta' and txrx=='tx' and pol=='h':
            pi_v_flip = [0,1,0,1]
        for i in range(4):
            pi_v[i] = (pi_v_flip[i]+pi_v[i])%2
        
        # Load gain cal
        loaded_VGA_cal=self.loaded_VGA_cal
        if pol=='h':
            loaded_nocal=self.loaded_nocal_h
        elif pol=='v':
            loaded_nocal=self.loaded_nocal_v
        # commets by Jenny for Python, no Linear indices but reshaper to 1D array seems to be interpreted as index = n*row + column, where n is number of elements in each row
        # commets by Jenny for Python, so no trainsport is needed here, but need to reshape to !D array to be indexed
        taper_settings = taper_settings.reshape(1,taper_settings.shape[0]*taper_settings.shape[1])
        phase_settings = phase_settings.reshape(1,phase_settings.shape[0]*phase_settings.shape[1])
        for ic in ics:
            disp_text = 'IC # '+str(ic)
            if verbose == 1:
                print(disp_text)
            ps0 = ps_v[ic]
            pi0 = pi_v[ic]
            pi_n=loaded_nocal['pi_n']
            #    st1_bias_n=loaded_nocal['st1_bias_n']
            st2_bias_n=loaded_nocal['st2_bias_n']
            if pi0 == 1:
                pi_n = 1-pi_n
            ps_step = 4.87
            Frontend = 0
            # phase = phase_for_16element_beamscan_ic(theta,phi,ic);
            idx1=[i for i,val in enumerate(cmap) if val==ic]
            idx2=[i for i,val in enumerate([amap[i] for i in idx1]) if val==Frontend]
            idx = [idx1[i] for i in idx2]
            taper = taper_settings[0,idx[0]]
            [val, st1_bias_bs] = closest(loaded_VGA_cal['VGA_ctrl'], taper)
            # st1_bias_bs = st1_bias_bs - 1; this is deleted by Jenny because the index of array in python is 1 less than matlab 
            phase = phase_settings[0,idx[0]]
            phase = phase % 360 # to shift phase to positive number
            phase_set = np.round(phase/ps_step).astype(int)
            ps_off = 0
            ps_bs = ps0 + ps_off + phase_set
            [ps_bs, pi_bs] = phase_repos(ps_bs, pi_n[0,Frontend])
            disp_text='Frontend # '+str(Frontend)+' ps setting '+str(ps_bs)+' pi_setting '+str(pi_bs)+' taper setting '+str(st1_bias_bs)+' for gain '+str(taper)
            if verbose == 1:
                print(disp_text)
            write_sram(ic,Frontend,pol,txrx,st1_bias_bs,st2_bias_n[0,Frontend],pi_bs,ps_bs,beam_index,0)
            for Frontend in range(1,16):
                idx1=[i for i,val in enumerate(cmap) if val==ic]
                idx2=[i for i,val in enumerate([amap[i] for i in idx1]) if val==Frontend]
                idx = [idx1[i] for i in idx2]
                taper = taper_settings[0,idx[0]]
                [val, st1_bias_bs] = closest(loaded_VGA_cal['VGA_ctrl'], taper)
                #        st1_bias_bs = st1_bias_bs - 1; this is deleted by Jenny because the index of array in python is 1 less than matlab 
                phase = phase_settings[0,idx[0]]
                phase = phase % 360 #% to shift phase to positive number
                phase_set = np.round(phase/ps_step).astype(int)
                ps_off = 0
                ps_bs = ps0 + ps_off + phase_set
                [ps_bs, pi_bs] = phase_repos(ps_bs, pi_n[0,Frontend])
                disp_text='Frontend # '+str(Frontend)+' ps setting '+str(ps_bs)+' pi_setting '+str(pi_bs)+' taper setting '+str(st1_bias_bs)+' for gain '+str(taper)
                if verbose == 1:
                    print(disp_text)
                write_sram(ic,Frontend,pol,txrx,st1_bias_bs,st2_bias_n[0,Frontend],pi_bs,ps_bs,beam_index,0)
        # Switch beam_index, strobe and mr(0)
        if switch_immediate:
            if ics == [0,1,2,3]:
                self.switch_beam_index(ic='all',pol=pol,txrx=txrx,beam_index=beam_index)
            else:
                for ic in ics:
                    self.switch_beam_index(ic=ic,pol=pol,txrx=txrx,beam_index=beam_index)

    def steer_beam(self,pol,txrx,theta,phi,taper=('None', -13),ps_v=[16,16,16,16],pi_v=[0,1,1,0],ics=[0],beam_index=0,switch_immediate=True,verbose=False):
        """Steers the beam to a given direction and with/without Taylor tapering for a given set of IC(s), polarization and TX/RX mode
        
        When TX/RX switching, Note that the enable() function needs to be run first with appropriate settings, in order to configure the rest of the IC before
        using this method.

        This method can also be used to write to the beamtables in the IC without actually switching the beam (using the switch_immediate parameter). When used
        to fill beam tables, note that the beam table entires for TX and RX mode are different, and so, the user needs to fill both. Once the beam tables are
        filled, only switch_beam_index() or switch_beam_indices() methods are needed, significantly speeding up operation.

        To configure arbitrary beams, or different shaped sub-arrays, please use set_arbitrary_beam() method

        Parameters
        ----------
        pol : {'h', 'v'}
            Polarization configured
        txrx : {'tx', 'rx'}
            TX/RX mode configured here. 
        theta : float
            Polar angle (degrees) in the range of (-60, 60) is supported by the array
        phi : float
            Azimuthal angle (degrees) in the range of (-90, 90) is supported by the array
        taper : (optional) tuple of (taper_name, side_lobe_level)
            Defaults to ('None', -13)
            taper_name : {'None', 'taper_16', 'taper_64'} supported
                'None' applies no tapering. In this mode, side_lobe_level is ignored. This is the default
                'taper_16' applies a 4x4 element Taylor tapering function with the given side_lobe_level specification. Use this for 16-element patterns
                'taper_64' applies a 8x8 element Taylor tapering function with the given side_lobe_level specification. Use this for 64-element patterns
            side_lobe_level : float
                Numbers below -13 make sense. Typical number is -20 for a taylor window supported by this Phased Array
        ics : (optional) array or list
            Defaults to only configuring IC 0
            An array or list of ICs that are to be steered / beamtable configured
        beam_index : (optional) integer in range(0,128)
            Default = 0
            The beam_index that is written into. Note that there is a separate beam table for TX and RX modes, and separate beam tables per polarization
        switch_immediate : (optional) bool
            Default = True
            This specifies whether the phased array immediately switches to this beam direction, or if only the beam table entry is updated. If False,
            switch_beam_indices() or switch_beam_index() needs to be run at a later stage

        Other optional paramaters
        -------------------------
        ps_v : 4x1 array or list with each in range(0,42)
            This sets the calibration for phase mismatch (5 degree units of correction) between individual ICs
            The PAWR board nominally doesn't require this calibration
            Default is [16,16,16,16] implying no calibration
        pi_v : 4x1 array of list with each in {0,1}
            This sets the calibration for phase mismatch (180 degree correction) between individual ICs
            Default is [0,1,1,0] due to the layout of the PCB and module
        verbose : (optional) bool
            Default = False
            Setting to true adds some verbose output used for debugging. Most users don't need to use this

        See Also
        --------
        enable() : The ICs used in steerbeam need to be first enabled in the correct TX/RX mode and polarization
        set_arbitrary_beam() : This generic function can be used to set arbitrary phase and gain for each element instead of being set by `theta`,`phi`,`taper`
        switch_beam_index() : Once the beam table is configured, only this is needed to switch the beam index for a single polarization
        switch_beam_indices() : Once the beam table is configured, only this is needed to switch the beam index for both polarizations

        """
        phase_settings=phase_map(theta,phi)
        if taper[0] == 'taper_16':
            taper_settings = get_taper_data(16, taper[1])
        elif taper[0] == 'taper_64':
            taper_settings = get_taper_data(64, taper[1])
        else:
            taper_settings = np.zeros([8,8])-3
        self.set_arbitrary_beam(pol,txrx,phase_settings,taper_settings,ps_v,pi_v,ics,beam_index,switch_immediate,verbose)
