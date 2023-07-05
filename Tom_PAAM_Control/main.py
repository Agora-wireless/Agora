import numpy as np
import time

import sys
sys.path.append('./API')
from API import PhasedArrayControl
from API import myPAWRBoardUtils



if __name__ == "__main__":
    ###### Don't modify the codes above #####

    addrUE = 'rfdev2-2'
    cfoBS = 97400 # modify
    addrBS = 'rfdev2-1'
    cfoUE = 0
    icTx = [0] # 1 deleted
    icRx = [1] # 3 deleted
    polarDL = ['h','v']
    polarUL = ['h','v']
    elemSetBS = [list(range(16)), list(range(16)), list(range(16)), list(range(16))]
    elemSetUE = [list(range(16)), list(range(16)), list(range(16)), list(range(16))]
#     elemSet = list(range(8)) # [0, 1, 2, 3] # 16 changed to 8

    ###### Don't modify the codes below #####

    phaseMap = np.zeros((8, 8))
    gainMap = -8 * np.ones((8, 8))

    # Base Station
    device = PhasedArrayControl.PhasedArrayControl(
        conn='Ethernet', ip=addrBS, conn_timeout=1, default_fpga_ip=0, verify=True)
    board = myPAWRBoardUtils.PAWRBoard(device)
    board.set_lo_switch(external=False)
    board.set_if_tx_h(combine=False) 
    board.set_if_tx_v(combine=False)
    board.set_if_rx_h(combine=False)
    board.set_if_rx_v(combine=False)
    board.pll_init(cfo=cfoBS)

    for ic in icTx:
        device.enable(ic=ic, fe_list=elemSetBS[ic], pol=polarDL[0], txrx='tx')
        device.enable(ic=ic, fe_list=elemSetBS[ic], pol=polarDL[1], txrx='tx')
    device.set_arbitrary_beam(
        phase_settings=phaseMap, taper_settings=gainMap, ics=icTx,
        pol=polarDL[0], txrx='tx',
        ps_v=[16, 16, 16, 16], pi_v=[0, 1, 1, 0], 
        beam_index=0, switch_immediate=True, verbose=False)
    device.set_arbitrary_beam(
        phase_settings=phaseMap, taper_settings=gainMap, ics=icTx,
        pol=polarDL[1], txrx='tx',
        ps_v=[16, 16, 16, 16], pi_v=[0, 1, 1, 0],
        beam_index=1, switch_immediate=True, verbose=False)        

    for ic in icRx:
        device.enable(ic=ic, fe_list=elemSetBS[ic], pol=polarUL[0], txrx='rx')
        device.enable(ic=ic, fe_list=elemSetBS[ic], pol=polarUL[1], txrx='rx')
    device.set_arbitrary_beam(
        phase_settings=phaseMap, taper_settings=gainMap, ics=icRx,
        pol=polarUL[0], txrx='rx',
        ps_v=[16, 16, 16, 16], pi_v=[0, 1, 1, 0], 
        beam_index=0, switch_immediate=True, verbose=False)
    device.set_arbitrary_beam(
        phase_settings=phaseMap, taper_settings=gainMap, ics=icRx,
        pol=polarUL[1], txrx='rx',
        ps_v=[16, 16, 16, 16], pi_v=[0, 1, 1, 0],
        beam_index=1, switch_immediate=True, verbose=False)

    board.get_paam_id(verbose=False)
    board.get_lo_switch(verbose=False)
    board.get_if_switches(verbose=False)
    adcData = board.get_adc_vals(verbose=False)
    adcCurrents = adcData.get_currents_dict()
    for icIdx in range(4):
        icCurrent = adcCurrents['2v7_'+str(icIdx)]
        icStatus = "Off" if icCurrent < 0.75 else "On"
        print("IC"+str(icIdx)+" current: "+str(icCurrent)+" status: "+icStatus)

    device.close_port()

    # User Equipment
    device = PhasedArrayControl.PhasedArrayControl(
        conn='Ethernet', ip=addrUE, conn_timeout=1, default_fpga_ip=0, verify=True)
    board = myPAWRBoardUtils.PAWRBoard(device)
    board.set_lo_switch(external=False)
    board.set_if_tx_h(combine=False) 
    board.set_if_tx_v(combine=False)
    board.set_if_rx_h(combine=False)
    board.set_if_rx_v(combine=False)
    board.pll_init(cfo=cfoUE)

    for ic in icRx:
        device.enable(ic=ic, fe_list=elemSetUE[ic], pol=polarDL[0], txrx='rx')
        device.enable(ic=ic, fe_list=elemSetUE[ic], pol=polarDL[1], txrx='rx')
    device.set_arbitrary_beam(
        phase_settings=phaseMap, taper_settings=gainMap, ics=icRx,
        pol=polarDL[0], txrx='rx',
        ps_v=[16, 16, 16, 16], pi_v=[0, 1, 1, 0], 
        beam_index=0, switch_immediate=True, verbose=False)
    device.set_arbitrary_beam(
        phase_settings=phaseMap, taper_settings=gainMap, ics=icRx,
        pol=polarDL[1], txrx='rx',
        ps_v=[16, 16, 16, 16], pi_v=[0, 1, 1, 0],
        beam_index=1, switch_immediate=True, verbose=False)
    for ic in icTx:
        device.enable(ic=ic, fe_list=elemSetUE[ic], pol=polarUL[0], txrx='tx')
        device.enable(ic=ic, fe_list=elemSetUE[ic], pol=polarUL[1], txrx='tx')
    device.set_arbitrary_beam(
        phase_settings=phaseMap, taper_settings=gainMap, ics=icTx,
        pol=polarUL[0], txrx='tx',
        ps_v=[16, 16, 16, 16], pi_v=[0, 1, 1, 0], 
        beam_index=0, switch_immediate=True, verbose=False)
    device.set_arbitrary_beam(
        phase_settings=phaseMap, taper_settings=gainMap, ics=icTx,
        pol=polarUL[1], txrx='tx',
        ps_v=[16, 16, 16, 16], pi_v=[0, 1, 1, 0],
        beam_index=1, switch_immediate=True, verbose=False)
    board.get_paam_id(verbose=False)
    board.get_lo_switch(verbose=False)
    board.get_if_switches(verbose=False)
    adcData = board.get_adc_vals(verbose=False)
    adcCurrents = adcData.get_currents_dict()
    for icIdx in range(4):
        icCurrent = adcCurrents['2v7_'+str(icIdx)]
        icStatus = "Off" if icCurrent < 0.50 else "On"
        print("IC"+str(icIdx)+" current: "+str(icCurrent)+" status: "+icStatus)
        
    device.close_port()



    print('Experiment Started!')
    while True:
        try:
            time.sleep(0.1)
        except KeyboardInterrupt:
            break



    # Base Station
    device = PhasedArrayControl.PhasedArrayControl(
        conn='Ethernet', ip=addrBS, conn_timeout=1, default_fpga_ip=0, verify=False)

    board = myPAWRBoardUtils.PAWRBoard(device)
    board.set_lo_switch(external=False)
    board.set_if_tx_h(combine=False) 
    board.set_if_tx_v(combine=False)
    board.set_if_rx_h(combine=False)
    board.set_if_rx_v(combine=False)
    board.pll_init()

    board.get_paam_id(verbose=False)
    board.get_lo_switch(verbose=False)
    board.get_if_switches(verbose=False)
    adcData = board.get_adc_vals(verbose=False)
    adcCurrents = adcData.get_currents_dict()
    for icIdx in range(4):
        icCurrent = adcCurrents['2v7_'+str(icIdx)]
        icStatus = "Off" if icCurrent < 1.0 else "On"
        print("IC"+str(icIdx)+" current: "+str(icCurrent)+" status: "+icStatus)

    device.close_port()

    # User Equipment
    device = PhasedArrayControl.PhasedArrayControl(
        conn='Ethernet', ip=addrUE, conn_timeout=1, default_fpga_ip=0, verify=False)

    board = myPAWRBoardUtils.PAWRBoard(device)
    board.set_lo_switch(external=False)
    board.set_if_tx_h(combine=False) 
    board.set_if_tx_v(combine=False)
    board.set_if_rx_h(combine=False)
    board.set_if_rx_v(combine=False)
    board.pll_init()

    board.get_paam_id(verbose=False)
    board.get_lo_switch(verbose=False)
    board.get_if_switches(verbose=False)
    adcData = board.get_adc_vals(verbose=False)
    adcCurrents = adcData.get_currents_dict()
    for icIdx in range(4):
        icCurrent = adcCurrents['2v7_'+str(icIdx)]
        icStatus = "Off" if icCurrent < 1.0 else "On"
        print("IC"+str(icIdx)+" current: "+str(icCurrent)+" status: "+icStatus)

    device.close_port()
