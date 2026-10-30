#pragma once
#include <string>
#include <vector>

namespace aero {

struct Event {
  double t;
  std::string id;
  std::string type;     // e.g. ALT_JUMP, SPEED_RANGE, DATA_DROPOUT, ARRIVED
  std::string message;
};

class EventLog {
 public:
  void add(double t, const std::string& id, const std::string& type, const std::string& msg);
  const std::vector<Event>& events() const { return events_; }
  bool writeCsv(const std::string& path) const;

 private:
  std::vector<Event> events_;
};

}  // namespace aero
