#include "gflags/gflags.h"
#include "radio_set_calibrate.h"

DEFINE_string(
    conf_file,
    TOSTRING(PROJECT_DIRECTORY) "/files/config/ci/tddconfig-sim-both.json",
    "Config filename");

DEFINE_string(type, "digital", "Calibration Type: digital|analog");

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage("conf_file : set the configuration filename");
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  std::string conf_file;
  std::string calibration_type;

  // For backwards compatibility
  if (argc == 3) {
    conf_file = std::string(argv[1]);
    calibration_type = std::string(argv[2]);
    std::printf(
        "User: Setting configuration filename to %s and calibration type to "
        "%s\n",
        conf_file.c_str(), calibration_type.c_str());
  } else {
    conf_file = FLAGS_conf_file;
    calibration_type = FLAGS_type;
  }

  std::unique_ptr<Config> cfg = std::make_unique<Config>(conf_file.c_str());
  std::unique_ptr<RadioSetCalibrate> calib =
      std::make_unique<RadioSetCalibrate>(cfg.get(), calibration_type);
  if (calibration_type == "digital") {
    calib->CalibrateSampleOffset();
  } else if (calibration_type == "analog") {
    if (cfg->Channel().find('A') != std::string::npos) {
      calib->DciqCalibrationProc(0);
    }
    if (cfg->Channel().find('B') != std::string::npos) {
      calib->DciqCalibrationProc(1);
    }
  }
  gflags::ShutDownCommandLineFlags();
  return 0;
}
