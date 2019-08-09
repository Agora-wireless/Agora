/*

 utility functions for file and text processing.

---------------------------------------------------------------------
 Copyright (c) 2018-2019, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
---------------------------------------------------------------------
*/

#include "utils.h"

int pin_to_core(int core_id) {
    int num_cores = sysconf(_SC_NPROCESSORS_ONLN);
    if (core_id < 0 || core_id >= num_cores)
        return -1;

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);

    pthread_t current_thread = pthread_self();    
    return pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);
}

std::vector<std::complex<int16_t>> Utils::double_to_int16(std::vector<std::vector<double>> in)
{
    int len = in[0].size();
    std::vector<std::complex<int16_t>> out(len, 0); 
    for (int i = 0; i < len; i++)
        out[i] = std::complex<int16_t>( (int16_t)(in[0][i]*32768), (int16_t)(in[1][i]*32768) );
    return out;
}

std::vector<uint32_t> Utils::cint16_to_uint32(std::vector<std::complex<int16_t>> in, bool conj, std::string order)
{
    std::vector<uint32_t> out (in.size(), 0);
    for (int i = 0; i < in.size(); i++)
    {
       uint16_t re = (uint16_t)in[i].real(); 
       uint16_t im = (uint16_t)(conj ? -in[i].imag() : in[i].imag());
       if (order == "iq")
           out[i] = (uint32_t)re << 16 | im;
       else if (order == "qi")
           out[i] = (uint32_t)im << 16 | re;
    }
    return out;
}

std::vector<uint32_t> Utils::cfloat32_to_uint32(std::vector<std::complex<float>> in, bool conj, std::string order)
{
    std::vector<uint32_t> out (in.size(), 0);
    for (int i = 0; i < in.size(); i++)
    {
       uint16_t re = (uint16_t)(int16_t(in[i].real()*32768.0)); 
       uint16_t im = (uint16_t)(int16_t((conj ? -in[i].imag() : in[i].imag())*32768));
       if (order == "iq")
           out[i] = (uint32_t)re << 16 | im;
       else if (order == "qi")
           out[i] = (uint32_t)im << 16 | re;
    }
    return out;
}

std::vector<std::vector<size_t>> Utils::loadSymbols(std::vector<std::string> frames, char sym)
{
    std::vector<std::vector<size_t>> symId;
    int frameSize = frames.size();
    symId.resize(frameSize);
    for(int f = 0; f < frameSize; f++)
    {
        std::string fr = frames[f]; 
        for (int g = 0; g < fr.size(); g++)
        {
            if (fr[g] == sym){
                symId[f].push_back(g);
            }
        }
    }
    return symId;
}

void Utils::loadDevices(std::string filename, std::vector<std::string> &data)
{
   std::string line;
   std::ifstream myfile (filename, std::ifstream::in);
   if (myfile.is_open())
   {
     while ( getline (myfile,line) )
     {
       //line.erase( std::remove (line.begin(), line.end(), ' '), line.end()); 
       data.push_back(line);
       std::cout << line << '\n';
     }
     myfile.close();
   }
 
   else printf("Unable to open device file %s\n", filename.c_str()); 
}

void Utils::loadData(const char* filename, std::vector<std::complex<int16_t>> &data, int samples)
{
    FILE* fp = fopen(filename,"r");
    data.resize(samples);
    float real, imag;
    for(int i = 0; i < samples; i++)
    {
    	fscanf(fp, "%f %f", &real, &imag);
        data[i] = std::complex<int16_t>(int16_t(real*32768), int16_t(imag*32768));
    }
    
    fclose(fp);
}

void Utils::loadData(const char* filename, std::vector<unsigned> &data, int samples)
{
    FILE* fp = fopen(filename,"r");
    data.resize(samples);
    unsigned sample;
    for(int i = 0; i < samples; i++)
    {
        fscanf(fp, "%u", &data[i]);
    }
    
    fclose(fp);
}

void Utils::loadTDDConfig(const std::string filename, std::string &jconfig)
{
    std::string line;
    std::ifstream configFile(filename);
    if (configFile.is_open())
    {
        while ( getline (configFile,line) )
        {
          jconfig += line;
        }
        configFile.close();
    }

    else printf("Unable to open config file %s\n", filename.c_str()); 

}

std::vector<std::string> Utils::split(const std::string& s, char delimiter)
{
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter))
    {
        tokens.push_back(token);
    }
    return tokens;
}

void Utils::printVector(std::vector<std::complex<int16_t>> &data)
{
    for(int i = 0; i < data.size(); i++)
    {
        std::cout << real(data.at(i)) << " " << imag(data.at(i)) << std::endl;
    }
}


