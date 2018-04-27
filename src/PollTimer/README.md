# PollTimer
Poll Timer for simple perodic task timing in Arduino projects

## Features
* Task execution at regular intervals without the complexity of interupts or poor coding practices like delays
* Oject based design for a unlimited number of concurrent timers
* Handles timer roll-over for seamless use over long periods of time
* Deterministic timing system insure that future executions are not effected by variations in the call time of previous exectutions
* Has optional staticstics collection to aid in the analysis task operation

### Basic Usage
#### Object Creation
```
PollTimer yourTimer( callFequencyHz ); // Desired frequency is passed as an integer
```

#### Timer Start
```
yourTimer.start();  // run to set timing system to start now
```

#### Timer Check
```
if( yourTimer.check() ) // this will return TRUE at the correct interval to insure the desired execution frequency
{
  executeYourCode();
}
```
* If this check is excessively delayed, it will return TRUE on multiple consecutive calls until the execution count has caught up with the number of times the function should have been executed.
* This check should be executed as often as possible.  It will only return TRUE at the correct times, so there is no downside to checking repeatedly.

### Timer Statistics
#### Stats Collection
```
if( yourTimer.check() )
{
  executeYourCode();
  
  yourTimer.collectStats();
  
  executeOtherCode();
}
```
* This will measure the time from when the when `check()` returned true to when this function is called.  In this case the stats collection will include the time to execute `executeYourCode();` but it will not include `executeOtherCode();`
* The time required to collect stats will not be included in the results
* If `collectStats()` is never called for an object, there will be no stats collection overhead.  Once it is called once, `check()` will collect the stats related to execution count and late start times.  These are fast, but not free.  If low overhead is absolutely critical for a specific timer, avoid collecting stats for that one.  This determined on a per timer basis, so collecting stats on one does not affect any other.

#### Stats Display
```
yourTimer.displayStats();
```
* This will send the results collected to the Serial port.  The output will look something like this:
```
RUN MIN:  20
RUN AVG:  22
RUN MAX:  36
RUN CNT:  4679
LATE AVG: 122
LATE MAX: 3140
CPU PCT : 45.68
```
* The units for time based results are microseconds
* Using the serial port with a standard Arduino product is fairly time consuming.  Be careful of overloading the CPU if executing this at a high frequency.
* If you only care about a few specific stats, you can get their value with the following functions:
```
uint32_t getMaxTime();
uint32_t getMinTime();
uint32_t getAvgTime();
uint32_t getCount();
uint32_t getAvgLate();
uint32_t getMaxLateTime();
float getPctCPU();
```
* These functions return the requested value, but they do not send it to the Serial port.
* Example to display only the CPU percent in the Serial monitor:
```
Serial.print("CPU Percent: ");
Serial.println( yourTimer.getPctCPU() );
```

#### Stats Reset
```
yourTimer.resetStats();
```
* This resets all of the staticstic catagories at the same time
* Due to the interconnected nature of the data, there is no plan to allow reseting only one catagory without affecting the rest
