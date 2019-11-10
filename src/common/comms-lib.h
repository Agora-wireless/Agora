/*

 Communications Library:
   a) Generate pilot/preamble sequences
   b) OFDM modulation

---------------------------------------------------------------------
 Copyright (c) 2018-2019, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 Author(s): Rahman Doost-Mohamamdy: doost@rice.edu
	    Oscar Bejarano: obejarano@rice.edu
---------------------------------------------------------------------
*/


#include <algorithm>
#include <iostream>
#include <complex.h>
#include <stdio.h>  /* for fprintf */
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include <fstream>      // std::ifstream
#include <math.h>
#include <string.h> /* for memcpy */
//#include "fft.h"
#include "mufft/fft.h"

class CommsLib
{
public:

    enum SequenceType
    {
	STS_SEQ,
	LTS_SEQ,
	LTS_F_SEQ,
	LTE_ZADOFF_CHU,
	GOLD_IFFT,
	HADAMARD	
    };

    enum ModulationOrder
    {
        QPSK = 2,
        QAM16 = 4,
        QAM64 = 6
    };

    CommsLib(std::string);
    ~CommsLib();

    static std::vector<std::vector<double> > getSequence(int N, int type);
    static std::vector<std::complex<float>> modulate(std::vector<int8_t>, int);
    static std::vector<int> getDataSc(int fftSize);
    static std::vector<int> getNullSc(int fftSize);
    static std::vector<int> getPilotScInd(int fftSize);
    static std::vector<std::complex<float>> getPilotSc(int fftSize);
    static std::vector<std::complex<float>> FFT(std::vector<std::complex<float>>, int);
    static std::vector<std::complex<float>> IFFT(std::vector<std::complex<float>>, int);
    static size_t find_pilot_seq(std::vector<std::complex<double>> iq, std::vector<std::complex<double>> pilot, size_t seqLen);
    static int findLTS(std::vector<std::complex<double>> iq, int seqLen);
    static std::vector<double> convolve(std::vector<std::complex<double>> const &f, std::vector<std::complex<double>> const &g);
    static std::vector<std::complex<double>> csign(std::vector<std::complex<double>> iq);
    static void meshgrid(std::vector<int> x_in, std::vector<int> y_in, std::vector<std::vector<int>> &x, std::vector<std::vector<int>> &y);

//private:
//    static inline float** init_qpsk();
//    static inline float** init_qam16();
//    static inline float** init_qam64();

};
