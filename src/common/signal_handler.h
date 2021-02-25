// http://www.yolinux.com/TUTORIALS/C++Signals.html

#ifndef SIGNALHANDLER_H_
#define SIGNALHANDLER_H_
#include <stdexcept>
using std::runtime_error;

class SignalException : public runtime_error {
 public:
  explicit SignalException(const std::string& _message)
      : std::runtime_error(_message) {}
};

class SignalHandler {
 protected:
  static bool mb_got_exit_signal;

 public:
  SignalHandler();
  ~SignalHandler() = default;

  static bool GotExitSignal();
  static void SetExitSignal(bool _bExitSignal);

  void SetupSignalHandlers();
  static void ExitSignalHandler(int _ignored);
};
#endif  // SIGNALHANDLER_H_
