#include "aero/event_log.hpp"
#include <fstream>

namespace aero {

void EventLog::add(double t, const std::string& id, const std::string& type,
                   const std::string& msg) {
  events_.push_back({t, id, type, msg});
}

bool EventLog::writeCsv(const std::string& path) const {
  std::ofstream f(path);
  if (!f) return false;
  f << "t,id,type,message\n";
  for (const auto& e : events_)
    f << e.t << ',' << e.id << ',' << e.type << ",\"" << e.message << "\"\n";
  return true;
}

}  // namespace aero
