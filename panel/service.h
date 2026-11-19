#pragma once

#include <sys/types.h>

#include <string>

struct WorkerStatus {
  bool fresh = false;
  std::string state;
  std::string message;
  int hands = 0;
  double rate_hz = 0;
  std::string left, right;
};

struct HandState {
  bool installed = false;
  std::string active;
  bool enabled = false;
  WorkerStatus worker;
  std::string switching;
  std::string problem;
  bool outline = false;
  bool mask = false;
  double system_hold_s = 5;

  bool on() const { return enabled || active == "active" || active == "activating"; }
};

bool set_driver_tune(const std::string& key, const std::string& value);

class Service {
 public:
  explicit Service(std::string unit);
  void refresh(HandState& state) const;
  bool start_switch(bool on, HandState& state);
  bool finished(HandState& state);
  bool switching() const { return child_ > 0; }

 private:
  std::string unit_, status_path_;
  pid_t child_ = -1;
};
