/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#include "config.h"

int main(void)
{
  // Initialise System Hardware
  INIT_SYSTEM();
  // Initialise Object's Dictionary
  InitOD();
  // Initialise PHY Interfaces
  PHY1_Init();
#ifdef PHY2_Init
  PHY2_Init();
#endif  //  PHY2_Init
  // Initialize MQTTSN
  MQTTSN_Init();

  vTaskStartScheduler();

  while(1);
}
