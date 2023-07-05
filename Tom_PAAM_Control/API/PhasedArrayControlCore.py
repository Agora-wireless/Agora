
"""
Copyright IBM

This code is Contributed IP and is being provided  according to the limited 
rights accorded in the IBM-Columbia sub-contract agreement for the COSMOS 
Effort of the NSF PAWR program

Defines a class with various lower-level methods to communicate to the IBM Phased Array
This class is inherited by the top-level PhasedArrayControl class which defined methods to control the IBM Phased Array
This class should not be used by users directly. Please use PhasedArrayControl instead
"""
import time
import json
import socket
import serial
import numpy as np
import os

class PhasedArrayControlCore:
    """Class with low-level methods to control the communication between the PC and the MicroZed board that then controls the IBM Phased Array ICs and various
    support functions on the board

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

    Examples
    --------
    examples/setup_betaboard_v1.2.py : Basic operation using __init__(), enable(), steer_beam() and close_port()
    examples/setup_betaboard_v1.2_advanced.py : Advanced operation using switch_beam_index(), start_recording(), stop_recording(), playonce()

    """
    
    def __init__(self, conn='Ethernet', ip='192.168.0.102',
                 serial_port='/dev/ttyUSB0',
                 conn_timeout=1,
                 default_fpga_ip=0,
                 board_name='pawr_beta',
                 verify=False):
        """Initialization method for the class. This sets up the connection between PC and Microzed, resets the phased array, and initializes various state variables
        used by the low-level methods in this class and the top-level methods in the PhasedArrayControl class.

        Users do not use PhasedArrayControlCore class directly. Please use from the top-level PhasedArrayControl class, as used in the Examples

        Parameters
        ----------
        conn : {'Ethernet', 'Serial'} (optional)
            Defined whether the connection to the Microzed is through Ethernet (default, fast) or through a Serial connection (slower)
        ip : (optional) string
            IP address of the Microzed when conn='Ethernet'. Default is '192.168.0.102'
        serial_port : (optional) string
            Serial port device name when conn='Serial'. Default is '/dev/ttyUSB0'
        conn_timeout : (optional) float
            Connection timeout parameter for the Ethernet / Serial connections. Default is 1second. This needs to be adjusted based on the network and PC. If
            this parameter is insufficient, this __init__ method fails
        default_fpga_ip : {0,1} (optional)
            Default FPGA IP used to control the IBM Phased Array. 0 (default) is baseline IP. 1 is a advanced IP with additional features
            Use set_fpga_control_ip() to change this after initialization of the PhasedArrayControl
            The advanced IP gives access to a suite of methods to improve speed of commonly-used top-level methods by enabling recording and playback of these
            methods from the FPGA itself. See start_recording(), stop_recording(), playonce(), playloop()
        board_name : (optional) Defaults to 'pawr_beta'
            Do not change this parameter. This is only for internal testing during development of this API
        verify : {True, False} (optional)
            This veriable is only set to True for debug purposes. When True, all register writes to the IBM Phased Array are followed by register reads in order
            to verify correct writing to the registers. Wrong register writes are flagged. Note that this option significantly slows down operation.

        """
        with open(os.path.join(os.path.dirname(__file__),'SDPAR_map.json')) as json_file:
            self.reg_dict = json.load(json_file)
        # Chip base addresses and offsets
        self.base_addess=int('43c00000',16)
        self.fe_base_addr_p=32768
        self.fe_offset=2048
        # Ethernet connection data
        self.BUFFER_SIZE=1024*2
        # Other chip/zynq data

        # One time cal information
        #self.loaded_VGA_cal=loadmat('VGA_cal.mat')
        #self.loaded_nocal_h=loadmat('no_cal_h.mat')
        #self.loaded_nocal_v=loadmat('no_cal_v.mat')
        self.loaded_VGA_cal = {'VGA_ctrl': np.array([[-12.67826667,  -9.059     ,  -7.04426667,  -5.73986667,
                                                      -4.79813333,  -4.0842    ,  -3.5272    ,  -3.08146667,
                                                      -2.69026667,  -2.35353333,  -2.11073333,  -1.87253333,
                                                      -1.64246667,  -1.48913333,  -1.2872    ,  -1.1662    ,
                                                      -1.0328    ,  -0.89906667,  -0.77993333,  -0.71006667,
                                                      -0.59446667,  -0.54513333,  -0.47146667,  -0.381     ,
                                                      -0.31846667,  -0.258     ,  -0.19706667,  -0.18346667,
                                                      -0.0998    ,  -0.067     ,  -0.0372    ,   0.        ]])}
        self.loaded_nocal_h = {'pi_n': np.array([[0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1]]),
                               'st1_bias_n': np.array([[16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16]]),
                               'st2_bias_n': np.array([[8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8]])
                               }
        self.loaded_nocal_v = {'pi_n': np.array([[0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1]]),
                               'st1_bias_n': np.array([[16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16]]),
                               'st2_bias_n': np.array([[8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8]])
                               }

        self.verify=False # before reset and init, verify isn't meaningful

        self.conn=conn
        self.ip=ip
        self.port=23
        self.serial_port=serial_port
        self.conn_timeout = conn_timeout

        self.kvint_ctrl_status_value=0
        self.is_open = False
        self.fast_write_reg = True
        self.fast_read_reg = True
        self.buffer_sends = False # before init for stable connection setup
        self.buffer=b''

        self.board_name = board_name
        self.default_fpga_ip = default_fpga_ip # 0 (default) for baseline IP, 1 for advanced IP
        self.__recording_ongoing = False
        self.__playloop_ongoing = False

        self._enabled_ics = [0, 0, 0, 0]
        self._beamindices = {'tx':{'h':0,'v':0},'rx':{'h':0,'v':0}}
        self.maxflat_configured = False

        self._init_fpga_kvint_2()
        self.buffer_sends = True # after init for fast performance
        self.verify=verify # after init, verify set from class instantiation
        
    def _open_port(self):
        """Internally used method to open Ethernet or Serial port. Users do not use this function"""
        if self.is_open:
            print("Zynq port is already open. Close port and run _init_fpga_kvint_2().")
            return
        if self.conn == 'Ethernet':
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.connect((self.ip, self.port))
                s.settimeout(self.conn_timeout)
            except:
                print('Socket with IP=%s and port=%d cannot be opened. Please verify.'%(self.ip,self.port))
                raise
        elif self.conn == 'Serial':
            try:
                s = serial.Serial(self.serial_port, bytesize=8, baudrate=115200, timeout=self.conn_timeout)
            except:
                print('Serial connection on port=%s cannot be opened. Please verify.'%(self.serial_port))
                raise
        else:
            raise RuntimeError('Only Ethernet is implemented currently. Please try again later.')
        time.sleep(0.3*self.conn_timeout)
        self.is_open = True
        self.zynq_port = s

    def _change_timeout(self, transaction_byte_length=0):
        """Internally used method to update the connection timeout temporarily for large transactions. Users do not need to use this"""
        if self.conn == 'Ethernet':
            max_bytes_rate = 300e3 # limited by Microzed processing
        else:
            max_bytes_rate = 100e3 # limited by serial baud rate
        timeout_needed = 2.0*transaction_byte_length/max_bytes_rate # safety factor of 2.0
        if timeout_needed < self.conn_timeout:
            timeout_needed = self.conn_timeout
        # print('Changing timeout to :', timeout_needed)
        if self.conn == 'Ethernet':
            self.zynq_port.settimeout(timeout_needed)
        else:
            self.zynq_port.timeout = timeout_needed
        
    def close_port(self):
        """This method is used to close the Ethernet or Serial port connection to the Microzed

        Always end your scripts with a call to this method

        """
        if not self.is_open:
            return
        self.buffer_sends = False
        if self.conn=='Serial':
            self._send_str_byte(bytes([4])) # Ctrl-D to exit sdpar_prog.py
            time.sleep(0.1)
            self._send_str_byte(bytes([3])) # Ctrl-C to exit sdpar_prog.py if it is stuck.
            time.sleep(0.1)                
            self._send_str_byte(bytes([4])) # Ctrl-D to logout
            time.sleep(0.1)
            dummy=self._clear_read_buffer()
            #print(dummy)
        self.zynq_port.close()
        print("Closed port to Zynq")
        time.sleep(0.1)
        self.is_open = False

    def _init_fpga_kvint_2(self):
        """Internally used method to initialize the Petalinux on the Microzed, reset the phased array and check basic life of the 4 ICs in the phased array. This
        starts a server on the Microzed that receives commands from the API and sends back the responses. The Microzed controls the IBM Phased Array, ADC for
        power measurement, reading ID for the board, PLL control to set the LO for the board

        Users do not need to use this method. This is called by the __init__() method

        """
        if self.is_open:
            print("Zynq port is already open. Close port and run _init_fpga_kvint_2().")
            return
        self._open_port()
        print("Opened port to FPGA")
        self._init_FPGA()
        self.kvint_ctrl_status_value += int('30000',16)
        self._mw(offset=8, data=self.kvint_ctrl_status_value,verify=self.verify)
        self.set_fpga_control_ip(self.default_fpga_ip)
        self.assert_and_release_reset()
        print("Reset the Phased Array")
        #self._write_register(self.reg_dict["SDPAR"]["cfg_extra_output_cycles_value_p"]["Address List"][0],3,'b10000')
        self._write_register(self.reg_dict["SDPAR"]["cfg_read_address_value_p"]["Address List"][0],3,'b10000')
        resp = []
        resp.append(self._read_register(self.reg_dict["SDPAR"]["cfg_read_address_value_p"]["Address List"][0],0))
        resp.append(self._read_register(self.reg_dict["SDPAR"]["cfg_read_address_value_p"]["Address List"][0],1))
        resp.append(self._read_register(self.reg_dict["SDPAR"]["cfg_read_address_value_p"]["Address List"][0],2))
        resp.append(self._read_register(self.reg_dict["SDPAR"]["cfg_read_address_value_p"]["Address List"][0],3))
        for ic in range(4):
            if resp[ic] != 3:
                print('ERROR for IC %d. Please check cable and power connectivity'%(ic))
            else:
                print('Initialization of IC %d was successful'%(ic))

    def _init_FPGA(self):
        """Internally used method to initialize Petalinux on the Microzed. Users do not need to use this method. This is called by _init_fpga_kvint_2()

        """
        if self.conn=='Ethernet' and self.port!=23:
            print('Connection is on a generic socket. Please ensure sdpar_server.py is running')
            return
        else:
            if self.conn=='Serial':
                self._send_str_byte(bytes([4])) # Ctrl-D to exit sdpar_prog.py if it was running.
                time.sleep(0.1)
                self._send_str_byte(bytes([3])) # Ctrl-C to exit sdpar_prog.py if it is stuck.
                time.sleep(0.1)                
                self._send_str_byte(bytes([4])) # Ctrl-D to logout if it was previously logged in.
                time.sleep(0.1)
                dummy = self._clear_read_buffer()
                #print(dummy)
            self._send_str_byte('root\n')
            time.sleep(0.1)
            self._send_str_byte('root\n')
            time.sleep(0.1)
            print("Logged in to Petalinux")
            dummy = self._clear_read_buffer()
            #print(dummy)
            self._send_str_byte('stty -echo \n')
            time.sleep(0.1)
            #self._send_str_byte('cd /run/media/sda1/sdpar_python \n')
            self._send_str_byte('cd /run/media/mmcblk0p1/sdpar_python \n')
            time.sleep(0.1)
            self._send_str_byte('./sdpar_prog.py \n')
            time.sleep(0.1)
            print("Started command parser on Zynq")
            dummy = self._clear_read_buffer()
            #print(dummy)
            if self.conn=='Serial':
                dummy = self._clear_read_buffer()
                #print(dummy)
            self._send_str_byte('/version\n')
            dummy = self._get_str_byte()
            #print(dummy)
            print('sdpar_prog.py /version:', dummy.decode().rstrip())
        # cmd_data = 'echo fclk0 > /sys/class/xdevcfg/xdevcfg/device/fclk_export\n' 
        # _send_str_byte(cmd_data1)
        # cmd_data2='echo 1 > /sys/class/fclk/fclk0/enable\n'
        # _send_str_byte(cmd_data2)
        # zynq_clk_freq='10000000'
        # print("zynq_clk_freq=",zynq_clk_freq)
        # cmd_data3='echo '+zynq_clk_freq+' > /sys/class/fclk/fclk0/set_rate\n'
        # _send_str_byte(cmd_data3)

    def assert_and_release_reset(self):
        """Method to reset the IBM phased array. All 4 ICs of the phased array are reset together since they all share the same reset signal.

        This method is called through the __init__() method anyway, so users don't typically need to use this method.

        """
        # Old IP reset
        reset_bit_mask = int('8000',16)
        self.kvint_ctrl_status_value = self.kvint_ctrl_status_value & (~reset_bit_mask)
        self._mw(offset=8, data=self.kvint_ctrl_status_value, verify=self.verify)
        self.kvint_ctrl_status_value = self.kvint_ctrl_status_value | reset_bit_mask
        self._mw(offset=8, data=self.kvint_ctrl_status_value, verify=self.verify)
        # advanced IP reset
        self._send_str_byte('resetn \n')
        self._mr(0) # Flush buffer
        # reset state variables
        self._enabled_ics = [0,0,0,0]
        self._beamindices = {'tx':{'h':0,'v':0},'rx':{'h':0,'v':0}}
        self.maxflat_configured = False
        self.__recording_ongoing = False
        self.__playloop_ongoing = False

    def set_fpga_control_ip(self, advanced=False):
        """Method to set the version of IP used for controlling the IBM Phased array ICs.

        Parameters
        ----------
        advanced : {True, False} (optional)
            FPGA IP used to control the IBM Phased Array. False implies baseline IP is used. True implies an advanced IP is used
            The advanced IP gives access to a suite of methods to improve speed of commonly-used top-level methods by enabling recording and playback of these
            methods from the FPGA itself. See start_recording(), stop_recording(), playonce(), playloop()

        """
        if advanced:
            if self.__recording_ongoing or self.__playloop_ongoing:
                print('Already using advanced FPGA control IP')
            else:
                self.__fpga_control_ip = 1
                self._send_str_byte('ifs 1\n')
                print('Using advanced FPGA control IP')
        else:
            if self.__recording_ongoing:
                print('ERROR: recording is ongoing. Cannot switch to baseline FPGA IP')
            elif self.__playloop_ongoing:
                print('ERROR: playloop ongoing. Cannot switch to baseline FPGA IP')
            else:
                self.__fpga_control_ip = 0
                self._send_str_byte('ifs 0\n')
                print('Using baseline FPGA control IP')
        self._mr(0) # Clear connection buffer between PC and MicroZed
        
    def _write_register(self,address,data,cmd=0,verify=False):
        """Internally used method to write registers in the IBM phased array. Users typically do not need to use this function"""
        # Process data string if necessary
        if type(data) == str:
            if data[0] == 'h': #Hex string
                data= int(data[1:],16)
            elif data[0] == 'b': #Binary string
                data=int(data[1:],2)
            else: #%Error
                print("Data in a wrong format. Please reenter") #added by Jenny
                return   
        # Process cmd string if necessary
        if type(cmd) == str:
            if cmd[0] == 'h': #Hex string
                cmd=int(cmd[1:],16)
            elif cmd[0] == 'b': #Binary string
                cmd=int(cmd[1:],2)
            else: #Error
                print("cmd in a wrong format. Please reenter")
                return
        if self.__fpga_control_ip == 0:
            master_address=(cmd & (2**4+2**3) ) << 15   # Mask broadcast
            master_address=master_address+((cmd & (2**1+2**0)) << 16)     # Mask out IC address
            master_address=master_address+address       # Add register (field) address
            strobe = self.kvint_ctrl_status_value | 1
            #print(master_address, data, strobe)
            if self.fast_write_reg:
                self._send_str_byte('wr '+str(master_address)+str(' ')+str(data)+str(' ')+str(strobe)+'\n')
            else:
                self._mw(0,master_address)
                self._mw(4,data)
                self._mw(8,strobe)
        else:
            if self.__playloop_ongoing:
                print('ERROR: playloop is ongoing. write_reg ignored. Please run stoploop() first')
                verify=False
            elif self.__recording_ongoing:
                self._send_str_byte('record_append %d %d %d\n'%(cmd, address, data))
                if verify:
                    print('WARN: recording is ongoing. So, write_reg verify forced to False')
                    verify = False
            else:
                self._send_str_byte('wrn %d %d %d\n'%(cmd, address, data))
        if verify:
            if cmd<4:
                readback = self._read_register(address,cmd)
                if readback != data:
                    print("ERROR verifying in write_register, [address, ic, data, readback]:", address, cmd, data, readback)
            elif (cmd&0b11000 == 0b10000):
                for ic in range(4):
                    readback = self._read_register(address,ic)
                    if readback != data:
                        print("ERROR verifying in write_register_ic=all, [address, ic, data, readback]:", address, ic, data, readback)
            elif (cmd&0b11000 == 0b01000):
                ic = cmd&0b00011
                for fe in range(16):
                    readback = self._read_register(address+self.fe_offset*fe,ic)
                    if readback != data:
                        print("ERROR verifying in write_register_fe=all, [address, ic, fe, data, readback]:", address, ic, fe, data, readback)
            elif (cmd&0b11000 == 0b11000):
                for ic in range(4):
                    for fe in range(16):
                        readback = self._read_register(address+self.fe_offset*fe,ic)
                        if readback != data:
                            print("ERROR verifying in write_register_ic=all_fe=all, [address, ic, fe, data, readback]:", address, ic, fe, data, readback)
            else:
                print("WARNING: write_register verify is not implemented for cmd=", cmd)
        return address

    def _read_register(self,address,ic):
        """Internally used method to read registers in the IBM phased array. Users typically do not need to use this function"""
        if self.__fpga_control_ip == 0:
            master_address=address+65536*ic      # Write address
            strobe = self.kvint_ctrl_status_value | 2
            #print(master_address, strobe)
            if self.fast_read_reg:        
                self._send_str_byte('rr '+str(master_address)+str(' ')+str(strobe)+'\n')
                return int(self._get_str_byte()[2:10].decode(),16)
            else:
                self._mw(0,master_address);
                self._mw(8,strobe)     # Read enable
                flag=0
                count=0
                while flag ==0:
                    data=self._mr(8)
                    flag=data&16
                    count=count+1
                    if count>5:
                        print("flag=",flag,"read times=",count)
                    if count>20:
                        print("WARNING: Stuck in register reading, flag=",flag)
                        return -1
                    if data&32:         #Check read timeout
                        print("WARNING: Read timeout on Zynq")
                        return -1
                # Data available. Read it.
                data=self._mr(4)
                read_data=data >> 16
                return read_data
        else:
            if self.__recording_ongoing:
                print('WARN: Recording is ongoing. read_register commands ignored')
                return 0
            elif self.__playloop_ongoing:
                print('ERROR: playloop is ongoing. read_register cannot be done. Run stoploop()')
                return -1
            else:
                self._send_str_byte('rrn %d %d\n'%(ic, address))
                ret_data = self._get_str_byte()
                return int(ret_data[2:10].decode(),16)

    def start_recording(self, start=0):
        """Advanced method to start a recording of write_register commands in a buffer on the FPGA when the advanced FPGA IP is used.

        This method doesn't do anything when using the baseline FPGA IP. Use set_fpga_control_ip(advanced=True) first.

        The FPGA has a recording buffer of depth 1638, and the user can record the write_register operations used in a commonly used top-level method and play
        it back for a much faster operation (since the data doesn't need to be communicated over slow Ethernet/Serial connections). The FPGA executes each
        write_register command from its memory in just 120ns.

        After this, all write_register commands are not actually sent to the Phased Array, but only stored in the FPGA buffer. the stop_recording() method stops
        the recording, and the playonce() and playloop() methods replay this buffer. 

        Note that any top-level methods with read_register operations will not work properly if recorded.
        
        Parameters
        ----------
        start : integer in range(0,1638)

            The start address within the buffer on the FPGA where the current recording starts. It is a users responsibility to make sure unique addresses are
            used

        See Also
        --------
        stop_recording() : To stop recording in the FPGA buffer
        playonce() : Single execution of a recording in the FPGA buffer. Equivalent to a single execution of the corresponding top-level method/functions
        playloop() : Looped execution of a recording in the FPGA buffer. Equivalent to a continuous loop to the corresponding top-level method/functions
        
        List of top-level methods that are compatible with the recording and playback operations. Any combination of these can also be recorded:
        PhasedArrayControl.enable()
        PhasedArrayControl.steer_beam()
        PhasedArrayControl.set_arbitrary_beam()
        PhasedArrayControl.switch_beam_index()
        PhasedArrayControl.switch_beam_indices()

        """
        if self.__fpga_control_ip == 1:
            if self.__playloop_ongoing:
                print('ERROR: playloop is ongoing. Please run stoploop() first')
            else:
                if start>1637:
                    print('ERROR: start of recording should be less than 1638')
                    return
                elif start>=1600:
                    print('WARN: start of recording=%d is close to the maximum=1637'%(start))
                self._record_reset(start=start)
                self.__recording_ongoing = True
        else:
            print('ERROR: start_recording() is only supported with the advanced FPGA IP mode')

    def stop_recording(self):
        """Advanced method to stop a recording that was started by the start_recording() method.

        This method only works with the advanced FPGA IP. This method doesn't do anything when using the baseline FPGA IP. Use
        set_fpga_control_ip(advanced=True) first to use this feature

        Once the recording is stopped, a user is able to playback the buffer once, or in a loop, right from the FPGA, thus increasing speed.
        Returns
        -------
        record_len : integer
            When the recording is stopped, this method returns the length of buffer stored in the FPGA memory. This information can be used by the user to
            determine the start address for the next recording so that the previous recording is not corrupted. This is the user's responsibility to manage
            memory.

        See Also
        --------
        start_recording() : To start a new recording in the FPGA buffer.
        playonce() : Single execution of a recording in the FPGA buffer. Equivalent to a single execution of the corresponding top-level method/functions
        playloop() : Looped execution of a recording in the FPGA buffer. Equivalent to a continuous loop to the corresponding top-level method/functions
        
        List of top-level methods that are compatible with the recording and playback operations. Any combination of these can also be recorded:
        PhasedArrayControl.enable()
        PhasedArrayControl.steer_beam()
        PhasedArrayControl.set_arbitrary_beam()
        PhasedArrayControl.switch_beam_index()
        PhasedArrayControl.switch_beam_indices()

        """
        if self.__fpga_control_ip == 1:
            self.__recording_ongoing = False
            return self._get_record_len()
        else:
            print('ERROR: playonce() is only supported with the advanced FPGA IP mode')    

    def playonce(self,length,start=0,flush_conn_buffer=True):
        """Advanced method to playback a recording that was created by the start_recording() and stop_recording() methods

        This method only works with the advanced FPGA IP. This method doesn't do anything when using the baseline FPGA IP. Use
        set_fpga_control_ip(advanced=True) first to use this feature

        Note that the stop_recording() function needs to be executed before running playonce() or playloop()

        Parameters
        ----------
        length : integer
            Length of buffer that is played back from the FPGA memory. The playback completes in length*120ns when the FPGA clock is 50MHz (nominal)
        start : integer in range (0,1638)
            Start address in the FPGA buffer
        flush_conn_buffer : (optional) bool (default True)
            Normally, when the top-level attribute buffer_sends is True, the PC buffers all write commands so that they can be sent in bulk. Thus, some methods
            like this method will not be sent over to the MicroZed and Phased Array IC because the method only has writes normally. If flush_conn_buffer is
            True, at the end of this method, the buffer is flushed and commands sent to the FPGA. If flush_conn_buffer is False, this is not done, and the
            commands are not sent until a read is needed. This option can be used to speed up operation when the user wants to queue up a number of such
            methods. Consider these examples below:
            Example 1 takes 0.9ms in a test run because both are buffered and sent in bulk
            >>> playonce(..., flush_conn_buffer=False)
            >>> playonce(..., flush_conn_buffer=True)
            Example 2 takes 1.6ms in a test run because both are sent separately, taking up double the ethernet latency
            >>> playonce(..., flush_conn_buffer=True)
            >>> playonce(..., flush_conn_buffer=True)

        See Also
        --------
        start_recording() : To start a new recording in the FPGA buffer.
        stop_recording() : To stop recording in the FPGA buffer. This is needed before running playonce()
        playloop() : Looped execution of a recording in the FPGA buffer. Equivalent to a continuous loop to the corresponding top-level method/functions
        
        List of top-level methods that are compatible with the recording and playback operations. Any combination of these can also be recorded:
        PhasedArrayControl.enable()
        PhasedArrayControl.steer_beam()
        PhasedArrayControl.set_arbitrary_beam()
        PhasedArrayControl.switch_beam_index()
        PhasedArrayControl.switch_beam_indices()

        """
        if self.__fpga_control_ip == 1:
            if self.__recording_ongoing:
                print('ERROR: Please run stop_recording() before playonce()')
            elif self.__playloop_ongoing:
                print('ERROR: playloop is already ongoing. Please run stoploop() and then playonce()')
            elif (start+length)>13106:
                print('ERROR: start+length cannot exceed 13106')
            elif start>1637:
                print('ERROR: start cannot exceed 1637')
            else:
                self._send_str_byte('playonce %d %d\n'%(start,length))
                if flush_conn_buffer:
                    self._mr(0)
        else:
            print('ERROR: playonce() is only supported with the advanced FPGA IP mode')

    def playloop(self,length,start=0, flush_conn_buffer=True):
        """Advanced method to playback, in a continuous loop, a recording that was created by the start_recording() and stop_recording() methods

        This method only works with the advanced FPGA IP. This method doesn't do anything when using the baseline FPGA IP. Use
        set_fpga_control_ip(advanced=True) first to use this feature

        Note that the stop_recording() function needs to be executed before running playonce() or playloop()

        This continuous loop is stopped with execution of the stoploop() method.

        Parameters
        ----------
        length : integer
            Length of buffer that is played back from the FPGA memory. The playback completes in length*120ns when the FPGA clock is 50MHz (nominal)
        start : integer in range (0,1638)
            Start address in the FPGA buffer
        flush_conn_buffer : (optional) bool (default True)
            see playonce() for detailed explanation of this option

        See Also
        --------
        stoploop() : Method to stop the playback loop.
        start_recording() : To start a new recording in the FPGA buffer.
        stop_recording() : To stop recording in the FPGA buffer. This is needed before running playonce()
        playonce() : Single execution of a recording in the FPGA buffer. Equivalent to a single execution of the corresponding top-level method/functions
        
        List of top-level methods that are compatible with the recording and playback operations. Any combination of these can also be recorded:
        PhasedArrayControl.enable()
        PhasedArrayControl.steer_beam()
        PhasedArrayControl.set_arbitrary_beam()
        PhasedArrayControl.switch_beam_index()
        PhasedArrayControl.switch_beam_indices()

        """
        if self.__fpga_control_ip == 1:
            if self.__recording_ongoing:
                print('ERROR: Please run stop_recording() before playloop()')
            elif self.__playloop_ongoing:
                print('ERROR: playloop is already ongoing')
            elif (start+length)>13106:
                print('ERROR: start+length cannot exceed 13106')
            elif start>1637:
                print('ERROR: start cannot exceed 1637')
            else:
                self.__playloop_ongoing = True
                self._send_str_byte('playloop %d %d\n'%(start,length))
                if flush_conn_buffer:
                    self._mr(0) # To clear buffer
        else:
            print('ERROR: playloop() is only supported with the advanced FPGA IP mode')

    def stoploop(self, flush_conn_buffer=True):
        """Advanced method to stop the continuously looping playback from the FPGA buffer initiated by the playloop() method.

        This method only works with the advanced FPGA IP. This method doesn't do anything when using the baseline FPGA IP. Use
        set_fpga_control_ip(advanced=True) first to use this feature

        Parameters
        ----------
        flush_conn_buffer : (optional) bool (default True)
            see playonce() for detailed explanation of this option

        See Also
        --------
        playonce() : Single execution of a recording in the FPGA buffer. Equivalent to a single execution of the corresponding top-level method/functions
        playloop() : Looped execution of a recording in the FPGA buffer. Equivalent to a continuous loop to the corresponding top-level method/functions

        """
        if self.__fpga_control_ip == 1:
            if not self.__playloop_ongoing:
                print('ERROR: playloop is not ongoing. Please run playloop() first')
            else:
                self.__playloop_ongoing = False
                self._send_str_byte('stoploop\n')
                if flush_conn_buffer:
                    self._mr(0) # To flush buffer
        else:
            print('ERROR: stoploop() is only supported with the advanced FPGA IP mode')

    def _record_reset(self, start):
        """Internall used method to reset the recording length variable in the FPGA buffer. A new start address is provided as an input. Users do not need to use this
        method
        """
        self._send_str_byte('record_reset %d\n'%(start))
        
    def _get_record_len(self):
        """Internally used method to get the current length of recording started by the start_recording() method and subsequent write_register commands
        
        User don't need to use this method
        """
        if self.__fpga_control_ip == 1:
            self._send_str_byte('get_record_len\n')
            return int(self._get_str_byte()[2:10].decode(),16)
        else:
            print('ERROR: get_record_len() is only supported with the advanced FPGA IP mode')
            return -1

    def _send_str_byte(self, write_data,force=False):
        """Internally used method to send a byte array over the Ethernet/Serial connection. Users don't need to use this method"""
        #print(write_data)
        if isinstance(write_data,str):
            write_data=write_data.encode()
        self.buffer+=write_data
        if self.buffer_sends and not force:
            return
        try:
            if self.conn=='Ethernet':
                self.zynq_port.send(self.buffer)
            elif self.conn=='Serial':
                self.zynq_port.write(self.buffer)
            else:
                raise RuntimeError("_send_str_byte not implemented for connection type:",self.conn)
        except:
            print("zynq_port.send failed")
            raise
        self.buffer=b''

    def _clear_read_buffer(self):
        """Internally used method to clear the receive buffer for the Ethernet/Serial connection. Users don't need to use this method"""
        time.sleep(0.5*self.conn_timeout)
        alldata=b''
        try:
            readdata=self._get_str_byte()
            while len(readdata) != 0:
                time.sleep(0.5*self.conn_timeout)
                alldata+=readdata
                readdata=self._get_str_byte()
        except socket.timeout:
            pass
        return alldata
    
    def _get_str_byte(self):
        """Internally used method to receive a string from the Ethernet/Serial connections. Users don't need to use this method"""
        timeout_updated = False
        if len(self.buffer)>0:
            if len(self.buffer)<10000:
                self._send_str_byte(b'',force=True)
            else:
                self._change_timeout(transaction_byte_length=len(self.buffer))
                timeout_updated = True
                buffer = self.buffer
                self.buffer = b''
                start_index = 0
                while (start_index+10000 < len(buffer)):
                    self.buffer = buffer[start_index:start_index+10000]
                    self._send_str_byte('',force=True)
                    start_index = start_index+10000
                self.buffer = buffer[start_index:]
                self._send_str_byte('',force=True)
        if self.conn=='Ethernet':
            readdata=b''
            while not (len(readdata)>0 and readdata[-1]==b'\n'[0]):
                readdata+=self.zynq_port.recv(self.BUFFER_SIZE)
        elif self.conn=='Serial':
            readdata=self.zynq_port.readline()
        else:
            raise RuntimeError("_send_str_byte not implemented for connection type:",self.conn)
        if timeout_updated:
            self._change_timeout(transaction_byte_length=0) # reset to original
        return readdata.replace(b'\r\n',b'\n')

    def _mw(self, offset, data, verify=False):
        """Internally used low-level method. Users don't need to use this method"""
        if not self.is_open:
            print("Zynq port is closed. Reinitialize.")
            return
        addr = offset
        self._send_str_byte('mw '+str(addr)+str(' ')+str(data)+'\n')
        if verify:
            readdata=self._mr(offset)
            if readdata != data:
                print('WARNING: readback data not equal to write in data:', str(offset), str(data), str(readdata))

    def _mr(self, offset):
        """Internally used low-level method. Users don't typically need to use this method"""

        if not self.is_open:
            print("Zynq port is closed. Reinitialize.")
            return
        addr = offset
        self._send_str_byte('mr '+str(addr)+'\n')
        read_bytes = self._get_str_byte()
        return int(read_bytes[2:10].decode(),16)
        #return int(read_bytes.decode())

    def _gpio_write(self, offset, data, verify=False):
        """Internally used method for writing to GPIO on the PAWR board. Users don't typically need to use this method.
        This method is used by the PAWRBoardUtils module to set control switches.

        """
        if not self.is_open:
            print("Zynq port is closed. Reinitialize.")
            return
        self._send_str_byte('gw '+str(offset)+str(' ')+str(data)+'\n')
        if verify:
            readdata=self._gpio_read(offset)
            if readdata != data:
                print('WARNING: GPIO readback data not equal to write in data:', str(offset), str(data), str(readdata))

    def _gpio_read(self, offset):
        """Internally used method for reading from GPIO on the PAWR board. Users don't typically need to use this method.
        This method is used by the PAWRBoardUtils module to read an ID for the board.

        """
        if not self.is_open:
            print("Zynq port is closed. Reinitialize.")
            return
        self._send_str_byte('gr '+str(offset)+'\n')
        read_bytes = self._get_str_byte()
        return int(read_bytes[2:10].decode(),16)

    def _pll_write(self, address, data, verify=False):
        """Internally used method for writing to the PLL on the PAWR board using SPI. Users don't typically need to use this method.
        This method is used by the PAWRBoardUtils module to configure the PLL.

        """
        if not self.is_open:
            print("Zynq port is closed. Reinitialize.")
            return
        if data>0xFFFF:
            print("PLL data can only be 16 bits.")
        if address>0x7F:
            print("PLL address can only be 7 bits.")
        command = (address<<16) + data
        self._send_str_byte('pw '+str(command)+'\n')
        if verify:
            readdata=self._pll_read(address)
            if readdata != data:
                print('WARNING: PLL readback data not equal to write in data:', str(address), str(data), str(readdata))

    def _pll_read(self, address):
        """Internally used method for reading from the PLL on the PAWR board using SPI. Users don't typically need to use this method.
        This method is used by the PAWRBoardUtils module to read registers in the PLL.

        """
        if not self.is_open:
            print("Zynq port is closed. Reinitialize.")
            return
        if address>0x7F:
            print("PLL address can only be 7 bits.")
        command = 0x800000 + (address<<16)
        self._send_str_byte('pr '+str(command)+'\n')
        read_bytes = self._get_str_byte()
        return int(read_bytes[2:10].decode(),16)

    def _adc_write(self, command):
        """Internally used method for writing to the ADC on the PAWR board using SPI. Users don't typically need to use this method.
        This method is used by the PAWRBoardUtils module to configure the ADC for measuring current and voltage values.

        """
        if not self.is_open:
            print("Zynq port is closed. Reinitialize.")
            return
        if command>0xFFFF:
            print("ADC command can only be 16 bits.")
        self._send_str_byte('aw '+str(command)+'\n')

    def _adc_read(self, command):
        """Internally used method for reading from the ADC on the PAWR board using SPI. Users don't typically need to use this method.
        This method is used by the PAWRBoardUtils module to read the ADC values.

        """
        if not self.is_open:
            print("Zynq port is closed. Reinitialize.")
            return
        if command>0xFFFF:
            print("ADC command can only be 16 bits.")
        self._send_str_byte('ar '+str(command)+'\n')
        read_bytes = self._get_str_byte()
        return int(read_bytes[2:10].decode(),16)

