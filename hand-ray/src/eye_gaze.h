#pragma once

#include <fcntl.h>
#include <linux/futex.h>
#include <pthread.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>

#include "hand_ray_model.h"

namespace fhr {

struct GazeSample {
  int64_t received_ns = 0;
  Vec3 direction{0, 0, -1};
  double openness = 0;
};

class EyeGaze {
 public:
  ~EyeGaze() { stop(); }
  void start() {
    if (running_) return;
    running_ = true;
    thread_ = std::thread([this] { loop(); });
  }
  void stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
    close_shm();
  }
  bool running() const { return running_; }
  bool latest(GazeSample& out) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_) return false;
    out = latest_;
    return true;
  }
  std::string problem() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return problem_;
  }

 private:
  static constexpr const char* kPath = "/dev/shm/eye-server.mmap";
  struct Layout {
    uint32_t version;
    size_t size, record;
  };
  static constexpr Layout kLayouts[2] = {{4, 0x4f21a, 0x152}, {5, 0x4f21f, 0x157}};
  static constexpr size_t kSequence = 0x38, kRequested = 0x3c, kMutex = 0x08;

  static int64_t now_ns() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return int64_t(ts.tv_sec) * 1'000'000'000 + ts.tv_nsec;
  }
  void set_problem(const std::string& p) {
    std::lock_guard<std::mutex> lock(mutex_);
    problem_ = p;
  }

  bool open_shm() {
    const int fd = open(kPath, O_RDWR | O_CLOEXEC);
    if (fd < 0) {
      set_problem(std::string(kPath) + ": " + std::strerror(errno));
      return false;
    }
    struct stat st{};
    uint32_t version = 0;
    if (fstat(fd, &st) != 0 || pread(fd, &version, 4, 0) != 4) {
      close(fd);
      set_problem("cannot read the eye shared memory");
      return false;
    }
    const Layout* layout = nullptr;
    for (const Layout& l : kLayouts)
      if (l.version == version) layout = &l;
    if (!layout || size_t(st.st_size) < layout->size) {
      close(fd);
      set_problem("unsupported eye shared memory (version " + std::to_string(version) + ")");
      return false;
    }
    void* map = mmap(nullptr, layout->size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (map == MAP_FAILED) {
      set_problem("cannot map the eye shared memory");
      return false;
    }
    base_ = static_cast<uint8_t*>(map);
    layout_ = *layout;
    inode_ = st.st_ino;
    if (read_u32(0) != layout_.version || read_u32(4) != 1) {
      close_shm();
      set_problem("eye shared memory not ready");
      return false;
    }
    set_problem("");
    return true;
  }
  void close_shm() {
    if (base_) munmap(base_, layout_.size);
    base_ = nullptr;
  }
  bool stale() const {
    struct stat st{};
    return read_u32(0) != layout_.version || stat(kPath, &st) != 0 || st.st_ino != inode_;
  }
  uint32_t read_u32(size_t at) const {
    return __atomic_load_n(reinterpret_cast<const uint32_t*>(base_ + at), __ATOMIC_ACQUIRE);
  }
  void request_sample() { __atomic_store_n(reinterpret_cast<uint32_t*>(base_ + kRequested), 1u, __ATOMIC_RELEASE); }
  pthread_mutex_t* shm_mutex() { return reinterpret_cast<pthread_mutex_t*>(base_ + kMutex); }
  bool lock_shm() {
    const int code = pthread_mutex_lock(shm_mutex());
    if (code == EOWNERDEAD) {
      pthread_mutex_consistent(shm_mutex());
      return true;
    }
    return code == 0;
  }

  void take_sample() {
    uint8_t record[0x81];
    std::memcpy(record, base_ + layout_.record, sizeof record);
    request_sample();
    uint32_t state;
    std::memcpy(&state, record, 4);
    if (state != 1) return;
    float g[2][3], open[2];
    std::memcpy(g, record + 0x0d, sizeof g);
    std::memcpy(open, record + 0x79, sizeof open);
    Vec3 sum{};
    for (int e = 0; e < 2; ++e) {
      const Vec3 v{g[e][0], g[e][1], g[e][2]};
      if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z) || norm(v) < 0.5) return;
      sum = sum + normalized(v);
    }
    if (!std::isfinite(open[0]) || !std::isfinite(open[1])) return;
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = {now_ns(), normalized(sum), std::max(open[0], open[1])};
    have_ = true;
  }

  void loop() {
    uint32_t seen = 0;
    bool first = true;
    while (running_) {
      if (!base_) {
        if (!open_shm()) {
          sleep_ms(2000);
          continue;
        }
        first = true;
      }
      if (stale()) {
        close_shm();
        continue;
      }
      if (!lock_shm()) {
        close_shm();
        sleep_ms(2000);
        continue;
      }
      const uint32_t sequence = read_u32(kSequence);
      if (first) {
        seen = sequence;
        first = false;
      }
      if (sequence != seen) {
        seen = sequence;
        take_sample();
        pthread_mutex_unlock(shm_mutex());
        continue;
      }
      request_sample();
      pthread_mutex_unlock(shm_mutex());
      timespec timeout{0, 100'000'000};
      syscall(SYS_futex, reinterpret_cast<uint32_t*>(base_ + kSequence), FUTEX_WAIT, seen, &timeout, nullptr, 0);
    }
  }
  void sleep_ms(int ms) {
    for (int t = 0; t < ms && running_; t += 100) std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::atomic<bool> running_{false};
  std::thread thread_;
  mutable std::mutex mutex_;
  GazeSample latest_;
  bool have_ = false;
  std::string problem_;
  uint8_t* base_ = nullptr;
  Layout layout_{};
  ino_t inode_ = 0;
};

}  // namespace fhr
