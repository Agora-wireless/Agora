"""Copyright IBM

This code is Contributed IP and is being provided  according to the limited 
rights accorded in the IBM-Columbia sub-contract agreement for the COSMOS 
Effort of the NSF PAWR program

A set of utilities for controlling various features on the IBM PAWR board. These include reading of a unique ID on each board, configuring the PLL for LO
generation, controlling of an ADC to measure the power consumption in various domains in the board.

"""

import numpy as np
import time
import PhasedArrayControl

class adcData:
    """Class to represent the ADC data, print the ADC values in a nice format, and to generate the current values in each domain of the IBM PAWR Phased
    Array boatd to the user. PAWRBoard.get_adc_vals() returns this class object as an output.

    Key Attributes
    --------------
    channels : A dictionary of ADC channels. This includes 10 current sensor values and 2 reference voltage channels (0V and 1.8V to verify operation)
    sense_res : The values of sense resistors used in the current sensors for the 10 current sensing channels
    vref : Reference voltage used for the ADC. This is 2.5V in the IBM PAWR board
    data : Raw ADC values that are filled in by the PAWRBoard.get_adc_vals() method
    
    Key Methods
    -----------
    get_currents_dict() : 
        This method returns a dictionary of the currents in the 10 domains in the IBM PAWR board. These are '1v2', '1v5', '1v8', '2v7_0', '2v7_1', '2v7_2', '2v7_3',
        '3v3_pll', '5v_uzed' and '12v'. The value in the '12v' is the current drawn from the 12V power input to the board.
    print_data() :
        This method prints a formatted table on the output for viewing the ADC readings and current sensor values in all the channels of the ADC.
    """
    
    def __init__(self, vref = 2.5):
        """Class instantiation of the attributes. This class is not to be instantiated by the user. A class object is returned by the PAWRBoard.get_adc_vals() method"""
        self.channels = {
            0 : "1v2",
            1 : "1v5",
            2 : "1v8",
            3 : "2v7_0",
            4 : "2v7_1",
            5 : "2v7_2",
            6 : "2v7_3",
            7 : "3v3_pll",
            8 : "5v_uzed",
            9 : "12v",
            10: "0V",
            11: "1V8"
        }
        self.sense_res = [0.02, 0.005, 0.02, 0.005, 0.005, 0.005, 0.005, 0.02, 0.01, 0.003]
        self.data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.vref = vref

    def _set_adc_data(self, data):
        """Internally used method by the PAWRBoard.get_adc_vals() method. Users don't need to use this method"""
        self.data = data

    def get_currents_dict(self):
        """This method returns a dictionary of the currents in the 10 domains in the IBM PAWR board. These are:
        '1v2', '1v5', '1v8', '2v7_0', '2v7_1', '2v7_2', '2v7_3', '3v3_pll', '5v_uzed' and '12v'.
        The value in the '12v' is the total current drawn from the 12V power input to the board.

        The user can use this method from on the adcData class object returned by the PAWRBoard.get_adc_vals() method to get the current values
        """
        currents_dict = {}
        for i in range(10):
            adc = self.data[i]
            volt = adc*self.vref/1023
            curr = (volt/100.0)/self.sense_res[i]
            currents_dict[self.channels.get(i)]=curr
        return currents_dict
    
    def print_data(self):
        """This method prints a formatted table on the output for viewing the ADC readings and current sensor values in all the channels of the ADC.
        
        """
        print('Index\tName\tADC\tVolt.\tCurr.')
        for i in range(10):
            adc = self.data[i]
            volt = adc*self.vref/1023
            curr = (volt/100.0)/self.sense_res[i]
            print('%u\t%s\t%u\t%0.3f\t%0.3f'%(i,self.channels.get(i),adc,volt,curr))
        for i in range(10,12):
            adc = self.data[i]
            volt = adc*self.vref/1023
            print('%u\t%s\t%u\t%0.3f\tx'%(i,self.channels.get(i),adc,volt))

class PAWRBoard:
    """A top-level class for various utilities on the IBM PAWR board. This uses the communication channel setup by the PhasedArrayControl class
    
    Attributes
    ----------
    adc_initialized : bool
        Specifies the internal state of the class, whether the ADC has been configured for reading the 12 channels. User shouldn't change this
    verify : bool
        A variable that indicates whether certain registers are read back after writes. This functions similar to the verify attribute in the PhasedArrayControl
        class, however, it is not tested extensively, and may not work. The recommendation is for the user to set it to False always

    Example
    -------
    See examples/setup_betaboard_v1.2.py

    """
    
    def __init__(self, paam_object, verify=False):
        """Class initialization method

        Parameters
        ----------
        paam_object : PhasedArrayControl object
            See example code where the PhasedArrayControl object is first initialized, and this object is passed as an input to this PAWRBoard class
            initialization
        verify : (optional) bool
            A variable that indicates whether certain registers are read back after writes. This functions similar to the verify attribute in the
            PhasedArrayControl class, however, it is not tested extensively, and may not work. The recommendation is for the user to set it to False always
        """
        self.c = paam_object
        self.verify = verify
        self.adc_initialized = False

    ## LO Switch ##
    def set_lo_switch(self, external=False):
        """Method to set the LO switch on the IBM PAWR board
        
        Parameters
        ----------
        external : bool
            If True, the LO to the IBM Phased Array ICs needs to be provided by the user
            If False, the LO to the IBM Phased Array ICs is from the on-board PLL. Make sure to powerdown the on-board PLL in this case using pll_powerdown()

        """
        a = 0 if external else 1
        self.c._gpio_write(4, a, verify=self.verify)
        _ = self.c._mr(0)
        
    def get_lo_switch(self, verbose=False):
        """Method to get the state of the LO switch on the IBM PAWR board
        
        Parameters
        ----------
        verbose : (optional) bool (default : False)
            If True, prints a descriptive string on the state of the LO switch
        
        Returns
        -------
        lo_switch_state : {0,1}. See set_lo_switch() for definition of the switch state

        """
        a = 0x01 & self.c._gpio_read(4)
        if verbose:
            print('LO switch: PLL' if a else 'LO switch: External')
        return a

    ## PAAM ID ##
    def get_paam_id(self, verbose=False):
        """Method to get the value of the unique ID for the IBM PAWR board
        
        Parameters
        ----------
        verbose : (optional) bool (default : False)
            If True, prints a descriptive string on the value of the PAAM ID
        
        Returns
        -------
        id : integer
            6 bit ID value implies the id could range from 0 to 63

        """
        a = 0x3F & self.c._gpio_read(0)
        if verbose:
            print('PAAM ID: 0x%2X'%(a))
        return a

    ## IF Switches ##
    def set_if_tx_h(self, combine):
        """Method to choose whether the IF input for the TX-mode h-polarization are common for all 4 ICs (64-element TX v-polarization) or kept separate (4 16-element
        beams for TX h-polarization)
        
        Parameters
        ----------
        combine : bool
            If True, the IF is combined for 64-element operation
            If False, 4 separate IF ports are used for 16-element operation in each IC

        """
        a = 0x0FFF & self.c._gpio_read(8)
        b = a if combine else 0xF000|a
        self.c._gpio_write(8, b, verify=self.verify)
        _ = self.c._mr(0)
        
    def set_if_tx_v(self, combine):
        """Method to choose whether the IF input for the TX-mode v-polarization are common for all 4 ICs (64-element TX v-polarization) or kept separate (4 16-element
        beams for TX v-polarization)

        Parameters
        ----------
        combine : bool
            If True, the IF is combined for 64-element operation
            If False, 4 separate IF ports are used for 16-element operation in each IC

        """
        a = 0xF0FF & self.c._gpio_read(8)
        b = a if combine else 0x0F00|a
        self.c._gpio_write(8, b, verify=self.verify)
        _ = self.c._mr(0)
        
    def set_if_rx_h(self, combine):
        """Method to choose whether the IF output for the RX-mode h-polarization are combined from all 4 ICs (64-element RX h-polarization) or kept separate (4
        16-element beams for RX h-polarization)
        
        Parameters
        ----------
        combine : bool
            If True, the IF is combined for 64-element operation
            If False, 4 separate IF ports are used for 16-element operation in each IC

        """
        a = 0xFF0F & self.c._gpio_read(8)
        b = a if combine else 0x00F0|a
        self.c._gpio_write(8, b, verify=self.verify)
        _ = self.c._mr(0)
        
    def set_if_rx_v(self, combine):
        """Method to choose whether the IF output for the RX-mode v-polarization are combined from all 4 ICs (64-element RX v-polarization) or kept separate (4
        16-element beams for RX v-polarization)
        
        Parameters
        ----------
        combine : bool
            If True, the IF is combined for 64-element operation
            If False, 4 separate IF ports are used for 16-element operation in each IC

        """
        a = 0xFFF0 & self.c._gpio_read(8)
        b = a if combine else 0x000F|a
        self.c._gpio_write(8, b, verify=self.verify)
        _ = self.c._mr(0)
        
    def get_if_switches(self, verbose=False):
        """Method to get the state of the IF switches on the IBM PAWR board
        
        Parameters
        ----------
        verbose : (optional) bool (default : False)
            If True, prints a descriptive string on the state of the IF switches
        
        Returns
        -------
        if_switch_state : 16-bit integer
            The switch values are 1 if separate IF inputs/outputs are used for each IC (16-element operation), and 0 if they are combined (64 element operation)
            The bits in the return value are formatted as :
                15:12 - 4 bits for the TX-mode h-polarization IF input
                11:8  - 4 bits for the TX-mode v-polarization IF input
                7:4   - 4 bits for the RX-mode h-polarization IF output
                3:0   - 4 bits for the RX-mode v-polarization IF output

        """
        a = 0xFFFF & self.c._gpio_read(8)
        if verbose:
            print('IF Switches TX_H: 0x%1X'%(0x0F & (a>>12)))
            print('IF Switches TX_V: 0x%1X'%(0x0F & (a>>8)))
            print('IF Switches RX_H: 0x%1X'%(0x0F & (a>>4)))
            print('IF Switches RX_V: 0x%1X'%(0x0F & (a>>0)))
        return a
    
    ## ADC ##
    def setup_adc(self):
        """Method to initialize the ADC on the IBM PAWR board. A class object of adcData is also initialized. 

        See example code where this is used.
        """
        self.c._adc_write(0x3C00)
        self.c._adc_write(0x3000)
        self.adc_initialized = True
        self.adcData = adcData()
        _ = self.c._mr(0)
        
    def get_adc_vals(self, verbose=False):
        """Method to read the 12 ADC channels, and return an adcDaca object with all the values. A user can optionally print a formatted table to the ADC readings
        and the current values using the verbose flag. Using the returned object, the user can then run the adcData.get_currents_dict() method to get a dictionary
        of current readings which can then be further processed.

        """
        if not self.adc_initialized:
            self.setup_adc()
        data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        for i in range(12):
            a = self.c._adc_read(0)
            idx = a>>12
            val = (a & 0x0FFF)>>2
            data[idx] = val
        self.adcData._set_adc_data(data)
        if verbose:
            self.adcData.print_data()
        return self.adcData
    
    ## PLL ##
    def pll_init(self, cfo=0):
        """Method to initialize the PLL on the IBM PAWR board. Make sure to set the LO switch using set_lo_switch(external=False) to use the PLL output. If using
        external LO input, make sure to powerdown the PLL using the pll_powerdown() method
                
        See example code where this is used.

        """
        self.pll_powerdown(True)
        self.pll_powerdown(True) # Just to be safe. Likely not needed
        time.sleep(0.01)
        self.pll_reset()
        self.pll_reset() # just to be safe. Likely not needed
        time.sleep(0.01)
        self._pll_defaults(cfo=cfo)
        time.sleep(0.01)
        self._pll_cal(fcal=True)
        self._pll_cal(acal=True)
        self._pll_cal(fcal=True) 
        self._pll_cal()
        time.sleep(0.01)
        #pawrBoard.pll_read_all_regs(verbose=True)

    def pll_powerdown(self,powerdown=True):
        """Method to powerdown the on-board PLL on the IBM PAWR board."""
        self.c._pll_write(0x00, 0x2200+powerdown, verify=self.verify)
        _ = self.c._mr(0)
        
    def pll_reset(self):
        """Method to reset the on-board PLL on the IBM PAWR board. User doesn't need to run this if the pll_init() is run"""
        self.c._pll_write(0x00, 0x2202, verify=self.verify)
        _ = self.c._mr(0)
        
    def pll_read_all_regs(self, verbose=False):
        """Optional method to read all the registers in the on-board PLL on the IBM PAWR board. The verbose flag can be used to print all the registers. This method
        returns a 65-long list of 16-bit integer values

        """
        regs = []
        for i in range(65):
            reg = self.c._pll_read(i)
            regs.append(reg)
            if verbose:
                print('0x%02X\t0x%04X'%(i, self.c._pll_read(i)))
        return regs

    def _pll_defaults(self, cfo=0):
        """Internally used in the pll_init() method to set default values to all the registers in the PLL on the IBM PAWR board. It configures the PLL to output 5.17GHz
        signal such that, with a IF at 3.02GHz, the RF is at 28GHz. The user doesn't typically need to use this method. Advanced users can copy this method and
        change it different values for the PLL registers. However, this is not recommented.

        """
        delta_numerator = round(cfo/200.0/6)
        numerator_new = 850000+delta_numerator

        pw = self.c._pll_write

        pw(0x00, 0x2200) #reset, powerdn, acal_en, fcal_en, muxout_sel
        pw(0x01, 0x0808)
        pw(0x02, 0x0500)
        pw(0x04, 0x1943)
        pw(0x07, 0x28B2)
        pw(0x08, 0x1084)
        pw(0x09, 0x0B02) #osc_2x
        pw(0x0A, 0x10D8) #ref_mult
        pw(0x0B, 0x0018) #pll_r
        pw(0x0C, 0x7002) #pll_r_pre
        pw(0x0D, 0x4000)
        pw(0x0E, 0x0FFD) #cp_current
        pw(0x13, 0x0965)
        pw(0x14, 0x012C)
        pw(0x16, 0x2300)
        pw(0x17, 0x8842)
        pw(0x18, 0x0509)
        pw(0x19, 0x0000)
        pw(0x1C, 0x2924)
        pw(0x1D, 0x0084)
        pw(0x1E, 0x0034)
        pw(0x1F, 0x0481)
        pw(0x20, 0x210A)
        pw(0x21, 0x2A0A)
        pw(0x22, 0xC3CA)
        pw(0x23, 0x021D)
        pw(0x24, 0x0011)
        pw(0x25, 0x4000) #pll_n_pre
        pw(0x26, 0x0032) #pll_n (integer)
        pw(0x27, 0x8204)
        pw(0x28, 0x000F) # pw(0x28, 0x0000) #fracn den MSB
        pw(0x29, 0x4240) # pw(0x29, 0x2710) #fracn den LSB
        pw(0x2A, 0x0000)
        pw(0x2B, 0x0000)
        pw(0x2C, numerator_new//(0x10000)) # pw(0x2C, 0x0000) #fracn num MSB
        pw(0x2D, numerator_new%(0x10000)) # pw(0x2D, 0x2134) #fracn num LSB
        pw(0x2E, 0x0FA3) #outa_pow
        pw(0x2F, 0x08CF)
        pw(0x30, 0x03FC)
        pw(0x3B, 0x0000) #muxout drive
        pw(0x3D, 0x0001) #lockdetect_type
        pw(0x3E, 0x0000)
        pw(0x40, 0x0077) #acal_fast, fcal_fast
        _ = self.c._mr(0)

    def _pll_cal(self, fcal=False, acal=False):
        """Internally used method to calibrate the frequency and ampitude of the PLL on the IBM PAWR Board. Please see pll_init() for a typical calibration routing with
        frequency calibration, amplitude calibration, frequency calibration followed by turning off of the calibration. The user typically doesn't need to use this
        method.

        """
        self.c._pll_write(0x00, 0x2200 + (fcal<<3) + (acal<<4))
        _ = self.c._mr(0)
