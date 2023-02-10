#ifndef TIMER_H
#define TIMER_H

#include <chrono>
#include <map>
#include <string>

class Timer {
     public:
          Timer() {

          }

          void startTimer(std::string name) {
               timers[name] = timeSinceEpoch();
          }

          //returns millisecons elapsed since startTimer was called and reset the timer specified
          double resetTimer(std::string name) {
               double duration = timeSinceEpoch() - timers[name];
               timers[name] = timeSinceEpoch();
               return duration;
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

static Timer debug_timer = Timer();

#endif