#ifndef DEBUGTIMER_H
#define DEBUGTIMER_H

#include <chrono>
#include <map>
#include <string>

class DebugTimer {
     public:
          DebugTimer() {

          }

          void startTimer(std::string name) {
               timers[name] = timeSinceEpoch();
          }

          //returns millisecons elapsed since startTimer was called
          double endTimer(std::string name) {
               double duration = timeSinceEpoch() - timers[name];
               timers.erase(name);
               return duration;
          }
     private:
          std::map<std::string, double> timers;

          double timeSinceEpoch() {
               return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
          }
};

static DebugTimer debug_timer = DebugTimer();

#endif