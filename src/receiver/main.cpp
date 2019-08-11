/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 * 
 */
#include "packageReceiver.hpp"
#include "millipede.hpp"

int main(int argc, char const *argv[])
{
    Config *cfg = new Config("data/tddconfig.json");
    Millipede *millipede_cli;
    int ret;
    try
    {
      SignalHandler signalHandler;

      // Register signal handler to handle kill signal
      signalHandler.setupSignalHandlers();
      millipede_cli = new Millipede(cfg);
      millipede_cli->start();
      ret = EXIT_SUCCESS;
    }
    catch (SignalException& e)
    {
      std::cerr << "SignalException: " << e.what() << std::endl;
      ret = EXIT_FAILURE;
    }


    return ret;
}
