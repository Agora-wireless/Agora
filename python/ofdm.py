import numpy as np

#from matplotlib.pyplot import *
#import matplotlib.animation as animation

def genLTS(upsample_mult = 1):
    #Assemble the LTS
    freq_bot = np.array([0,0,0,0,0,0,1,1,-1,-1,1,1,-1,1,-1,1,1,1,1,1,1,-1,-1,1,1,-1,1,-1,1,1,1,1])
    freq_top = np.array([1,-1,-1,1,1,-1,1,-1,1,-1,-1,-1,-1,-1,1,1,-1,-1,1,-1,1,-1,1,1,1,1,0,0,0,0,0])
    freq = np.concatenate((freq_bot, [0], freq_top))
  
    temp = np.fft.fftshift(freq) 
    signal = np.fft.ifft(temp, axis=0)
          
    #signal = np.fft.ifft(np.fft.fftshift(freq), axis=0)
    end = len(signal)
    signal = signal/np.absolute(signal).max() 
    #Now affix the cyclic prefix
    signal = np.concatenate((signal[end - 32:], signal, signal))
  
    
    #Now, we up sample with linear interpolation.
    #With Argos we shouldn't actually have to do this. Might come in handy
    #anyway?

    r = signal.real
    i = signal.imag
    t = r*r + i*i
    
    
    #plot(t);

    #show()

    return signal

def ModQPSK(index):
        iq = 0 + 0j;
        #return iq;
        if(index == 3): iq = 1 - 1j#1 -1j;
        if(index == 2): iq = -1 - 1j #-1 -1j;
        
        if(index == 1): iq = -1 + 1j #-1 +1j;
        if(index == 0): iq = 1 + 1j #1 +1j;

        iq = iq * np.sqrt(2)/2;
        return iq

def Mod16QAM(index):
        iq = 0 + 0j;
        #return iq;
        if(index == 15): iq = -3 -3j;
        if(index == 14): iq = -3 -1j;
        if(index == 11): iq = -3 +3j;
        if(index == 10): iq = -3 +1j;

        if(index == 13): iq = -1 -3j;
        if(index == 12): iq = -1 -1j;
        if(index == 9): iq = -1 +3j;
        if(index == 8): iq = -1 +1j;
        
        if(index == 7): iq = +3 -3j;
        if(index == 6): iq = +3 -1j;
        if(index == 3): iq = +3 +3j;
        if(index == 2): iq = +3 +1j;
        
        if(index == 5): iq = +1 -3j;
        if(index == 4): iq = +1 -1j;
        if(index == 1): iq = +1 +3j;
        if(index == 0): iq = +1 +1j;

        iq = (iq / 3) * np.sqrt(2)/2;
        return iq

def genDATA_test(data_index = 0): #16QAM
        
        #SC_IND_DATA = [2:7 9:21 23:27 39:43 45:57 59:64];
        
        iq = np.zeros((64), dtype = complex) 
        #data_index = 0;
        
        
        # 48 OFDM constellations
        #for x in range(1,7) + range(8,21) + range(22,27) + range(38,43) + range(44,57) + range(58,64):
        for x in list(range(1,7)) + list(range(8,21)) + list(range(22,27)) + list(range(38,43)) + list(range(44,57)) + list(range(58,64)):
            iq[x] = Mod16QAM(data_index % 16)
            data_index = data_index + 1

        # Pilots
        iq[7] = 1;
        iq[21] = 1j;
        iq[43] = -1;
        iq[57] = -1j;

        # IFFT
        signal = np.fft.ifft(iq,axis=0);
        signal = signal/np.absolute(signal).max()
        # Add CP
        end = len(signal)
        signal = np.concatenate((signal[end - 16:], signal))


        #print iq
        #print signal
        
        r = signal.real
        i = signal.imag
        t = r*r + i*i
        #plot(t);

        #show()
        


        return signal
        
        
def genOFDM_Brust_test2():

        LTS = genLTS()
        D0 = genDATA_test(data_index = 0)
        D1 = genDATA_test(data_index = 48 + 4)  # bias for debug
        D2 = genDATA_test(data_index = 96 + 8 )
        D3 = genDATA_test(data_index = 144 + 12)

        signal = np.concatenate((D0,D0,D0,D0,D0,D0))

        r = signal.real
        i = signal.imag
        t = r*r + i*i
        #plot(t)
        #show()
        
        p = np.angle(signal,1)
        #plot(p)
        #show()
        
        return signal
def genOFDM_Brust_test():

        LTS = genLTS()
        D0 = genDATA_test(data_index = 0)
        D1 = genDATA_test(data_index = 48 + 4)  # bias for debug
        D2 = genDATA_test(data_index = 96 + 8 )
        D3 = genDATA_test(data_index = 144 + 12)

        signal = np.concatenate((LTS, D0, D1, D2, D3))

        r = signal.real
        i = signal.imag
        t = r*r + i*i
        #plot(t)
        #show()
        
        p = np.angle(signal,1)
        #plot(p)
        #show()
        
        return signal


def genOFDM_Brust_test_22Data():

        LTS = genLTS()
        D0 = genDATA_test(data_index = 0)
        D1 = genDATA_test(data_index = 48 + 4)  # bias for debug
        D2 = genDATA_test(data_index = 96 + 8 )
        D3 = genDATA_test(data_index = 144 + 12)

        signal = np.concatenate((LTS, D0, D0, D0, D0,
                                 D0, D1, D2, D3,
                                 D0, D1, D2, D3,
                                 D0, D1, D2, D3,
                                 D0, D1, D2, D3,
                                 D0, D1))

        r = signal.real
        i = signal.imag
        t = r*r + i*i
        #plot(t)
        #show()
        
        p = np.angle(signal,1)
        #plot(p)
        #show()
        
        return signal
        
        
def genDATA(data,offset): #16QAM
        
        #SC_IND_DATA = [2:7 9:21 23:27 39:43 45:57 59:64];
        
        iq = np.zeros((64), dtype = complex) 
        #data_index = 0;
        
        
        # 48 OFDM constellations
        #for x in range(1,7) + range(8,21) + range(22,27) + range(38,43) + range(44,57) + range(58,64):
        t = 0;        
        for x in list(range(1,7)) + list(range(8,21)) + list(range(22,27)) + list(range(38,43)) + list(range(44,57)) + list(range(58,64)):
            value = data[offset+t*4] *8 + data[offset+t*4+1]*4 + data[offset+t*4+2]*2 + data[offset+t*4+3];
            iq[x] = Mod16QAM(value)
            t = t+1;
            

        # Pilots
        iq[7] = 1;
        iq[21] = 1j;
        iq[43] = -1;
        iq[57] = -1j;

        # IFFT
        signal = np.fft.ifft(iq,axis=0);
        signal = signal/np.absolute(signal).max()
        # Add CP
        end = len(signal)
        signal = np.concatenate((signal[end - 16:], signal))
        


        return signal
        
def genDATAQPSK(data,offset): 
        
        #SC_IND_DATA = [2:7 9:21 23:27 39:43 45:57 59:64];
        
        iq = np.zeros((64), dtype = complex) 
        
        # 48 OFDM constellations
        #for x in range(1,7) + range(8,21) + range(22,27) + range(38,43) + range(44,57) + range(58,64):
        t = 0
        for x in list(range(1,7)) + list(range(8,21)) + list(range(22,27)) + list(range(38,43)) + list(range(44,57)) + list(range(58,64)):
            value = data[offset+t*2] *2 + data[offset+t*2+1]
            iq[x] = ModQPSK(value)
            t = t+1

        # Pilots
        iq[7] = 1;
        iq[21] = 1j;
        iq[43] = -1;
        iq[57] = -1j;
        #iq[7] = 1
        #iq[21] = 1
        #iq[43] = 1
        #iq[57] = -1

        # IFFT
        signal = np.fft.ifft(iq,axis=0);
        signal = signal/np.absolute(signal).max()
        # Add CP
        end = len(signal)
        signal = np.concatenate((signal[end - 16:], signal))
        
        return signal

def genDATA4QAMTest(data,offset): #16QAM
        
        #SC_IND_DATA = [2:7 9:21 23:27 39:43 45:57 59:64];
        
        iq = np.zeros((64), dtype = complex) 
        #data_index = 0;
        
        
        # 48 OFDM constellations
        #for x in range(1,7) + range(8,21) + range(22,27) + range(38,43) + range(44,57) + range(58,64):
        t = 0
        for x in list(range(1,7)) + list(range(8,21)) + list(range(22,27)) + list(range(38,43)) + list(range(44,57)) + list(range(58,64)):
            if(t == 0):iq[x] = 15
            if(t == 1):iq[x] = 11
            if(t == 2):iq[x] = 7
            if(t == 3):iq[x] = 3                      
            t = t+1
            if(t==4):t=0
            
            

        # Pilots
        iq[7] = 1;
        iq[21] = 1j;
        iq[43] = -1;
        iq[57] = -1j;

        # IFFT
        signal = np.fft.ifft(iq,axis=0);
        signal = signal/np.absolute(signal).max()
        # Add CP
        end = len(signal)
        signal = np.concatenate((signal[end - 16:], signal))
        


        return signal
        
def genOFDM_Brust_turbo_10Data(name):
    InputData = np.zeros(192*22);
    l = 0;
    #with open("C:/documents/visual studio 2015/Projects/TurboCode/TurboCode/Decoded Stream.txt") as f:
    with open(name) as f:
        while True:
            c = f.read(1)
            if not c:
                #print("End of file")
                break
            #print("Read a character:", c, l)
            if c == '0' :
                InputData[l] = 0;
            if c == '1' :
                InputData[l] = 1;
            l = l+1;
            
    print(l,InputData[l-1],InputData[l-2]);
    
    LTS = genLTS()
    
    
    D0 = genDATA(InputData,0);
    D1 = genDATA(InputData,192);
    D2 = genDATA(InputData,192*2);
    D3 = genDATA(InputData,192*3);
    D4 = genDATA(InputData,192*4);
    
    D5 = genDATA(InputData,192*5);
    D6 = genDATA(InputData,192*6);
    D7 = genDATA(InputData,192*7);
    D8 = genDATA(InputData,192*8);
    D9 = genDATA(InputData,192*9);
    

    signal = np.concatenate((LTS, D0, D1, D2, D3, D4,
                                 D5, D6, D7, D8,
                                 D9));

    
            
    return signal
    
def genOFDM_Brust_turbo_20Data_Test(name):
    InputData = np.zeros(192*22);
    l = 0;
    #with open("C:/documents/visual studio 2015/Projects/TurboCode/TurboCode/Decoded Stream.txt") as f:
    with open(name) as f:
        while True:
            c = f.read(1)
            if not c:
                #print("End of file")
                break
            #print("Read a character:", c, l)
            if c == '0' :
                InputData[l] = 0;
            if c == '1' :
                InputData[l] = 1;
            l = l+1;
            
    print(l,InputData[l-1],InputData[l-2]);
    
    LTS = genLTS()
    
    
    D0 = genDATA4QAMTest(InputData,0);
    D1 = genDATA4QAMTest(InputData,192);
    D2 = genDATA4QAMTest(InputData,192*2);
    D3 = genDATA4QAMTest(InputData,192*3);
    D4 = genDATA4QAMTest(InputData,192*4);
    
    D5 = genDATA4QAMTest(InputData,192*5);
    D6 = genDATA4QAMTest(InputData,192*6);
    D7 = genDATA4QAMTest(InputData,192*7);
    D8 = genDATA4QAMTest(InputData,192*8);
    D9 = genDATA4QAMTest(InputData,192*9);
    
    D10 = genDATA4QAMTest(InputData,192*10);
    D11 = genDATA4QAMTest(InputData,192*11);
    D12 = genDATA4QAMTest(InputData,192*12);
    D13 = genDATA4QAMTest(InputData,192*13);
    D14 = genDATA4QAMTest(InputData,192*14);
    
    D15 = genDATA4QAMTest(InputData,192*15);
    D16 = genDATA4QAMTest(InputData,192*16);
    D17 = genDATA4QAMTest(InputData,192*17);
    D18 = genDATA4QAMTest(InputData,192*18);
    D19 = genDATA4QAMTest(InputData,192*19);

    signal = np.concatenate((LTS, D0, D1, D2, D3, D4,
                                 D5, D6, D7, D8,
                                 D9, D10, D11, D12,
                                 D13, D14, D15, D16,
                                 D17, D18, D19 ));

    
            
    return signal
    
def genOFDM_Brust_turbo_20Data_QPSK(name,ant):
    InputData = np.zeros(192*22);
    l = 0;
    #with open("C:/documents/visual studio 2015/Projects/TurboCode/TurboCode/Decoded Stream.txt") as f:
    with open(name) as f:
        while True:
            c = f.read(1)
            if not c:
                #print("End of file")
                break
            #print("Read a character:", c, l)
            if c == '0' :
                InputData[l] = 0;
            if c == '1' :
                InputData[l] = 1;
            l = l+1;
            
    print(l,InputData[l-1],InputData[l-2]);
    
    LTS = genLTS()
    
    D0 = genDATAQPSK(InputData,0);
    D1 = genDATAQPSK(InputData,96);
    D2 = genDATAQPSK(InputData,96*2);
    D3 = genDATAQPSK(InputData,96*3);
    D4 = genDATAQPSK(InputData,96*4);
    
    D5 = genDATAQPSK(InputData,96*5);
    D6 = genDATAQPSK(InputData,96*6);
    D7 = genDATAQPSK(InputData,96*7);
    D8 = genDATAQPSK(InputData,96*8);
    D9 = genDATAQPSK(InputData,96*9);
    
    D10 = genDATAQPSK(InputData,96*10);
    D11 = genDATAQPSK(InputData,96*11);
    D12 = genDATAQPSK(InputData,96*12);
    D13 = genDATAQPSK(InputData,96*13);
    D14 = genDATAQPSK(InputData,96*14);
    
    D15 = genDATAQPSK(InputData,96*15);
    D16 = genDATAQPSK(InputData,96*16);
    D17 = genDATAQPSK(InputData,96*17);
    D18 = genDATAQPSK(InputData,96*18);
    D19 = genDATAQPSK(InputData,96*19);
    z = np.zeros((80), dtype = complex) 

    signal = np.concatenate((LTS, LTS, D0, D1, D2, D3, D4,
                                 D5, D6, D7, D8,
                                 D9, D10, D11, D12,
                                 D13, D14, D15, D16,
                                 D17, D18, D19 )) * .5;

    return signal
    
def genOFDM_Brust_turbo_20Data(name,ant):
    InputData = np.zeros(192*22);
    l = 0;
    #with open("C:/documents/visual studio 2015/Projects/TurboCode/TurboCode/Decoded Stream.txt") as f:
    with open(name) as f:
        while True:
            c = f.read(1)
            if not c:
                #print("End of file")
                break
            #print("Read a character:", c, l)
            if c == '0' :
                InputData[l] = 0;
            if c == '1' :
                InputData[l] = 1;
            l = l+1;
            
    print(l,InputData[l-1],InputData[l-2]);
    
    LTS = genLTS()
    
    #LTS = np.concatenate(LTS[160-ant*8:],LTS[0:160-ant*8])
    
    
    D0 = genDATA(InputData,0);
    D1 = genDATA(InputData,192);
    D2 = genDATA(InputData,192*2);
    D3 = genDATA(InputData,192*3);
    D4 = genDATA(InputData,192*4);
    
    D5 = genDATA(InputData,192*5);
    D6 = genDATA(InputData,192*6);
    D7 = genDATA(InputData,192*7);
    D8 = genDATA(InputData,192*8);
    D9 = genDATA(InputData,192*9);
    
    D10 = genDATA(InputData,192*10);
    D11 = genDATA(InputData,192*11);
    D12 = genDATA(InputData,192*12);
    D13 = genDATA(InputData,192*13);
    D14 = genDATA(InputData,192*14);
    
    D15 = genDATA(InputData,192*15);
    D16 = genDATA(InputData,192*16);
    D17 = genDATA(InputData,192*17);
    D18 = genDATA(InputData,192*18);
    D19 = genDATA(InputData,192*19);

    signal = np.concatenate((LTS,LTS, D0, D1, D2, D3, D4,
                                 D5, D6, D7, D8,
                                 D9, D10, D11, D12,
                                 D13, D14, D15, D16,
                                 D17, D18, D19 ));
                                 #Two LTS!!!
    
            
    return signal    
    
def genOFDM_Brust_turbo_20Data_local_input(InputData,ant):
    
            
    
    LTS = genLTS()
    
#    Temp = np.zeros(8*ant, dtype = complex)
#    
#    Temp = LTS[0:8*ant]
#    LTS[0:160-8*ant] = LTS[8*ant:160]
#    LTS[160-8*ant:160] = Temp
    
    
    D0 = genDATA(InputData,0);
    D1 = genDATA(InputData,192);
    D2 = genDATA(InputData,192*2);
    D3 = genDATA(InputData,192*3);
    D4 = genDATA(InputData,192*4);
    
    D5 = genDATA(InputData,192*5);
    D6 = genDATA(InputData,192*6);
    D7 = genDATA(InputData,192*7);
    D8 = genDATA(InputData,192*8);
    D9 = genDATA(InputData,192*9);
    
    D10 = genDATA(InputData,192*10);
    D11 = genDATA(InputData,192*11);
    D12 = genDATA(InputData,192*12);
    D13 = genDATA(InputData,192*13);
    D14 = genDATA(InputData,192*14);
    
    D15 = genDATA(InputData,192*15);
    D16 = genDATA(InputData,192*16);
    D17 = genDATA(InputData,192*17);
    D18 = genDATA(InputData,192*18);
    D19 = genDATA(InputData,192*19);

    signal = np.concatenate((LTS, D0, D1, D2, D3, D4,
                                 D5, D6, D7, D8,
                                 D9, D10, D11, D12,
                                 D13, D14, D15, D16,
                                 D17, D18, D19 ));

    
            
    return signal    
    
    
def genOFDM_Brust_turbo_10Data_Fake():
    InputData = np.zeros(192*22);
    l = 0;
    #with open("C:/documents/visual studio 2015/Projects/TurboCode/TurboCode/Decoded Stream.txt") as f:
    with open("DecodedStream") as f:
        while True:
            c = f.read(1)
            if not c:
                #print("End of file")
                break
            #print("Read a character:", c, l)
            if c == '0' :
                InputData[l] = 0;
            if c == '1' :
                InputData[l] = 1;
            l = l+1;
            
    print(l,InputData[l-1],InputData[l-2]);
    
    LTS = genLTS()
    
    
    D0 = genDATA(InputData,0);
    D1 = genDATA(InputData,0);
    D2 = genDATA(InputData,0);
    D3 = genDATA(InputData,0);
    D4 = genDATA(InputData,0);
    
    D5 = genDATA(InputData,0);
    D6 = genDATA(InputData,0);
    D7 = genDATA(InputData,0);
    D8 = genDATA(InputData,0);
    D9 = genDATA(InputData,0);
    

    signal = np.concatenate((LTS, D0, D1, D2, D3, D4,
                                 D5, D6, D7, D8,
                                 D9));

    
            
    return signal
#genDATA_test()
genOFDM_Brust_test()
