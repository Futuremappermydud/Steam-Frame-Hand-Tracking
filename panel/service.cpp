#include "service.h"

#include <spawn.h>
#include <sys/wait.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <sstream>

extern char** environ;

namespace {

std::string json_field(const std::string& json, const std::string& name) {
  const std::string key = "\"" + name + "\":";
  size_t at = json.find(key);
  if (at == std::string::npos) return {};
  at += key.size();
  while (at < json.size() && json[at] == ' ') ++at;
  if (at < json.size() && json[at] == '"') {
    std::string out;
    for (size_t i = at + 1; i < json.size() && json[i] != '"'; ++i) {
      if (json[i] == '\\' && i + 1 < json.size()) {
        ++i;
        out += json[i] == 'n' ? ' ' : json[i];
      } else {
        out += json[i];
      }
    }
    return out;
  }
  const size_t end = json.find_first_of(",}", at);
  return json.substr(at, end == std::string::npos ? std::string::npos : end - at);
}

}  // namespace

bool set_driver_tune(const std::string& key, const std::string& value) {
  const char* runtime = std::getenv("XDG_RUNTIME_DIR");
  if (!runtime) return false;
  const std::string path = std::string(runtime) + "/frame_hand_ray.tune", tmp = path + ".tmp";
  std::string kept;
  {
    std::ifstream in(path);
    std::string line;
    while (std::getline(in, line))
      if (line.compare(0, key.size() + 1, key + "=") != 0 && !line.empty()) kept += line + "\n";
  }
  {
    std::ofstream out(tmp, std::ios::trunc);
    if (!out) return false;
    out << kept << key << "=" << value << "\n";
  }
  return std::rename(tmp.c_str(), path.c_str()) == 0;
}

Service::Service(std::string unit) : unit_(std::move(unit)) {
  const char* runtime = std::getenv("XDG_RUNTIME_DIR");
  if (runtime) status_path_ = std::string(runtime) + "/frame-hand-tracking/status.json";
}

void Service::refresh(HandState& state) const {
  const std::string command =
      "systemctl --user show -p LoadState -p ActiveState -p UnitFileState -- " + unit_ + " 2>/dev/null";
  FILE* pipe = popen(command.c_str(), "r");
  state.installed = false;
  if (pipe) {
    char line[256];
    while (std::fgets(line, sizeof line, pipe)) {
      std::string s(line);
      while (!s.empty() && (s.back() == '\n' || s.back() == '\r')) s.pop_back();
      const size_t eq = s.find('=');
      if (eq == std::string::npos) continue;
      const std::string key = s.substr(0, eq), value = s.substr(eq + 1);
      if (key == "LoadState")
        state.installed = value == "loaded";
      else if (key == "ActiveState")
        state.active = value;
      else if (key == "UnitFileState")
        state.enabled = value == "enabled";
    }
    pclose(pipe);
  }

  state.worker = WorkerStatus{};
  std::ifstream in(status_path_);
  if (status_path_.empty() || !in) return;
  std::stringstream ss;
  ss << in.rdbuf();
  const std::string json = ss.str();
  WorkerStatus& w = state.worker;
  w.state = json_field(json, "state");
  w.message = json_field(json, "message");
  w.hands = std::atoi(json_field(json, "hands").c_str());
  w.rate_hz = std::atof(json_field(json, "rate_hz").c_str());
  w.left = json_field(json, "left");
  w.right = json_field(json, "right");
  const double updated = std::atof(json_field(json, "updated_unix").c_str());
  timespec now{};
  clock_gettime(CLOCK_REALTIME, &now);
  const double age = double(now.tv_sec) + now.tv_nsec * 1e-9 - updated;
  w.fresh = age >= -1 && age < 5 && w.state != "stopped";
}

bool Service::start_switch(bool on, HandState& state) {
  if (child_ > 0) return false;
  const char* argv[] = {"systemctl", "--user", on ? "enable" : "disable", "--now", "--", unit_.c_str(), nullptr};
  pid_t pid = -1;
  const int error = posix_spawnp(&pid, "systemctl", nullptr, nullptr, const_cast<char* const*>(argv), environ);
  if (error != 0) {
    state.problem = "cannot run systemctl";
    return false;
  }
  child_ = pid;
  state.switching = on ? "on" : "off";
  state.problem.clear();
  std::fprintf(stderr, "turning hand tracking %s\n", on ? "on" : "off");
  return true;
}

bool Service::finished(HandState& state) {
  if (child_ <= 0) return false;
  int status = 0;
  const pid_t done = waitpid(child_, &status, WNOHANG);
  if (done == 0 || (done < 0 && errno == EINTR)) return false;
  child_ = -1;
  if (done < 0 || !WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    state.problem = "turning it " + state.switching + " failed (systemctl --user status " + unit_ + ")";
    std::fprintf(stderr, "%s\n", state.problem.c_str());
  }
  state.switching.clear();
  return true;
}
