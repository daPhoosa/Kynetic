/*
   Kynetic CNC Control Software
   Copyright (C) 2017 Phillip Schmidt

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

#include <PollTimer.h>           // https://github.com/daPhoosa/PollTimer                 --instal to libraries diectory


// **** POLL TIMERS ****
PollTimer motionControl(MOTION_CONTROL_HZ);
PollTimer blockExecute(BLOCK_EXECUTE_HZ);
PollTimer buttonsAndUI(BUTTONS_UI_HZ);
PollTimer motionUpdate(1); // must be 1Hz, always
PollTimer maintenance(MAINTENANCE_HZ);

void startPollTimers()
{
   motionControl.start();
   blockExecute.start();
   buttonsAndUI.start();
   motionUpdate.start();
   maintenance.start();
}
