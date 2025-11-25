// From https://github.com/mnpk/profc

#pragma once
#include <stdio.h>
#include <functional>
#include <chrono>
#include <thread>
#include <mutex>
#include <set>

#define PROFC_NODE(name)                              \
  static ProfileNode __node##__LINE__(name);          \
  TheNodeList::Instance().AddNode(&__node##__LINE__); \
  ScopedTimer __timer##__LINE__(std::bind(            \
      &ProfileNode::Accumulate, &__node##__LINE__, std::placeholders::_1));

#define PROFC_PRINT() \
  TheNodeList::Instance().Print();


class ProfileNode {
 public:
  explicit ProfileNode(const std::string& name)
      : name_(name), count_(0), elapsed_us_(0), max_us_(0) {
  }

  void Accumulate(std::chrono::microseconds us) {
    std::lock_guard<std::mutex> guard(the_lock_);
    
    count_++;
    elapsed_us_ += us;

    // Update max if current call is larger
    if (us.count() > max_us_) {
      max_us_ = us.count();
    }

    auto total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_us_).count();
    avg_ms_ = (count_ == 0) ? 0 : (total_ms / count_);

  }

  void Print() {
    // Print: name, count, total ms, average ms, and max ms
    printf(
                    // avg(ms)  max(ms)
        "%-20s %10d%8.0f %8.0f\n",
        name_.c_str(),
        count_,
        avg_ms_,
        max_us_ / 1000.);
  }

 public:
  std::string name_;
  int count_;
  std::chrono::microseconds elapsed_us_;
  std::mutex the_lock_;

  // Store the maximum microseconds of any single call
  double max_us_;
  double avg_ms_;
};

class ScopedTimer {
 public:
  explicit ScopedTimer(std::function<void(std::chrono::microseconds)> callback)
      : callback_(callback) {
    start_ = std::chrono::system_clock::now();
  }
  ~ScopedTimer() {
    auto end = std::chrono::system_clock::now();
    auto elapsed = end - start_;
    callback_(std::chrono::duration_cast<std::chrono::microseconds>(elapsed));
  }
  ScopedTimer(const ScopedTimer&) = delete;
  ScopedTimer& operator=(const ScopedTimer&) = delete;

 private:
  std::function<void(std::chrono::microseconds)> callback_;
  std::chrono::time_point<std::chrono::system_clock> start_;
};

class TheNodeList {
 public:
  void AddNode(ProfileNode* node) {
    nodes_.insert(node);
  }
  ~TheNodeList() = default;

  static TheNodeList& Instance() {
    static TheNodeList nodes;
    return nodes;
  }

  void Print() {
    printf("\033[2J\033[H");
    printf("----------------------------------------------------\n");
    printf("name                      count  avg(ms)  max(ms)\n");
    printf("----------------------------------------------------\n");

    double total_ms = 0.0;
    double total_max = 0.0;
    for (auto node : nodes_) {
        node->Print();
        total_ms += node->avg_ms_;
        total_max += node->max_us_;
    }

    printf("----------------------------------------------------\n");
    printf("%-26s     %8.0f %8.0f\n", "SUM", total_ms, total_max / 1000.);
  }


 private:
  std::set<ProfileNode*> nodes_;
};