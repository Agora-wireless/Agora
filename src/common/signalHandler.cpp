// http://www.yolinux.com/TUTORIALS/C++Signals.html

#include <errno.h>
#include <signal.h>

#include "signalHandler.hpp"

bool SignalHandler::mbGotExitSignal = false;

/**
* Default Contructor.
*/
SignalHandler::SignalHandler()
{
}

/**
* Destructor.
*/
SignalHandler::~SignalHandler()
{
}

/**
* Returns the bool flag indicating whether we received an exit signal
* @return Flag indicating shutdown of program
*/
bool SignalHandler::gotExitSignal()
{
    return mbGotExitSignal;
}

/**
* Sets the bool flag indicating whether we received an exit signal
*/
void SignalHandler::setExitSignal(bool _bExitSignal)
{
    mbGotExitSignal = _bExitSignal;
}

/**
* Sets exit signal to true.
* @param[in] _ignored Not used but required by function prototype
*                     to match required handler.
*/
void SignalHandler::exitSignalHandler(int)
{
    mbGotExitSignal = true;
}

/**
* Set up the signal handlers for CTRL-C.
*/
void SignalHandler::setupSignalHandlers()
{
    if (signal((int)SIGINT, SignalHandler::exitSignalHandler) == SIG_ERR) {
        throw SignalException("!!!!! Error setting up signal handlers !!!!!");
    }
}
