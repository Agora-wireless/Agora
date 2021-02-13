all: matrix fft modulation

matrix:
	g++ -o test_matrix test_matrix.cc cpu_attach.cc -std=c++11 -w -O3 -march=native -g -larmadillo -Wl,--no-as-needed -lmkl_intel_lp64 -lmkl_sequential -lmkl_core -lpthread -lm -ldl
fft:
	g++ -o test_fft_mkl test_fft_mkl.cc cpu_attach.cc -std=c++11 -w -O3 -march=native -Wl,--no-as-needed -lmkl_intel_ilp64 -lmkl_sequential -lmkl_core -lpthread -lm -ldl -fext-numeric-literals

modulation:
	g++ -I../../src/common -I/opt/FlexRAN-FEC-SDK-19-04/sdk/source/phy/lib_common -o test_modulation test_modulation.cc ../../src/common/modulation.cc ../../src/common/modulation_srslte.cc -std=c++11 -w -O3 -march=native 
clean:
	rm test_matrix test_fft_mkl test_modulation
