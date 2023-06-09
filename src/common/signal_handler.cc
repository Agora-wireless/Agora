// http://www.yolinux.com/TUTORIALS/C++Signals.html

#include "signal_handler.h"

#include <cerrno>
#include <csignal>

bool SignalHandler::mb_got_exit_signal = false;

/**
 * Default Contructor.
 */
SignalHandler::SignalHandler() = default;

/**
 * Destructor.
 */

/**
 * Returns the bool flag indicating whether we received an exit signal
 * @return Flag indicating shutdown of program
 */
bool SignalHandler::GotExitSignal() { return mb_got_exit_signal; }

/**
 * Sets the bool flag indicating whether we received an exit signal
 */
void SignalHandler::SetExitSignal(bool _bExitSignal) {
  mb_got_exit_signal = _bExitSignal;
}

/**
 * Sets exit signal to true.
 * @param[in] _ignored Not used but required by function prototype
 *                     to match required handler.
 */
void SignalHandler::ExitSignalHandler(int /*unused*/) {
  mb_got_exit_signal = true;
}

/**
 * Set up the signal handlers for CTRL-C.
 */
void SignalHandler::SetupSignalHandlers() {
  if (std::signal((int)SIGINT, SignalHandler::ExitSignalHandler) == SIG_ERR) {
    throw SignalException("!!!!! Error setting up signal handlers !!!!!");
  }
}
