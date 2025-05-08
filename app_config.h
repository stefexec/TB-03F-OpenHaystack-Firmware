#pragma once

/* Enable C linkage for C++ Compilers: */
#if defined(__cplusplus)
extern "C" {
#endif


/////////////////// FEATURE SELECT /////////////////////////////////
#define BLE_APP_PM_ENABLE					1
#define PM_DEEPSLEEP_RETENTION_ENABLE		1
#define TEST_CONN_CURRENT_ENABLE            0 //test curr, disable keyscan concerned
#define BLE_REMOTE_SECURITY_ENABLE      	0
#define BLT_SOFTWARE_TIMER_ENABLE 			1


/////////////////// Clock  /////////////////////////////////
#define CLOCK_SYS_CLOCK_HZ  	24000000

enum{
	CLOCK_SYS_CLOCK_1S = CLOCK_SYS_CLOCK_HZ,
	CLOCK_SYS_CLOCK_1MS = (CLOCK_SYS_CLOCK_1S / 1000),
	CLOCK_SYS_CLOCK_1US = (CLOCK_SYS_CLOCK_1S / 1000000),
};

/////////////////// watchdog  //////////////////////////////
#define MODULE_WATCHDOG_ENABLE		0
#define WATCHDOG_INIT_TIMEOUT		500  //ms

/////////////open SWS digital pullup to prevent MCU err, this is must ////////////
#define PA7_DATA_OUT			1

#include "vendor/common/default_config.h"

/* Disable C linkage for C++ Compilers: */
#if defined(__cplusplus)
}
#endif
