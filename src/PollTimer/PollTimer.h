/* 
      PollTimer
      Copyright (C) 2016  Phillip J Schmidt
      
         This program is free software: you can redistribute it and/or modify
         it under the terms of the GNU General Public License as published by
         the Free Software Foundation, either version 3 of the License, or
         (at your option) any later version.
         
         This program is distributed in the hope that it will be useful,
         but WITHOUT ANY WARRANTY; without even the implied warranty of
         MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
         GNU General Public License for more details.
         
         You should have received a copy of the GNU General Public License
         along with this program.  If not, see <http://www.gnu.org/licenses/>
         
 */


#ifndef PollTimer_h
   #define PollTimer_h

   #include <arduino.h>

   class PollTimer
   {
      public:
      
         PollTimer(unsigned long Hz);
         bool check();
         bool precheck(uint32_t earlyTime);
         void start();
         void start(uint32_t delay);

         void collectStats();
         uint32_t getMaxTime();
         uint32_t getMinTime();
         uint32_t getAvgTime();
         uint32_t getCount();
         uint32_t getAvgLate();
         uint32_t getMaxLateTime();
         float getPctCPU();
         void displayStats();
         void resetStats();

         unsigned long us();	// return microseconds
         float dt();				// return decimal seconds
         
         
      private:
         uint32_t period_us;	// microseconds
         float period_dt;	   // decimal seconds
         
         uint32_t nextExecute;
         uint32_t lateTimeCollector, avgCollector;
         uint32_t statsTimeStart, maxTime, maxLateTime, minTime;
         uint32_t lateCount, statsCount;
   };

#endif