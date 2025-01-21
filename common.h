#ifndef __COMMON_H__
#define __COMMON_H__

#include <stdint.h>
#include <stdarg.h>
#include <setjmp.h>
#include <semaphore.h>
#include <pthread.h>
#include <errno.h>

#include <linux/un.h>

#include <net/if.h>

typedef struct
{
	int disable_periodic_timer;
	int oneshot_timer_msec;
	int verbose;
} settings_t;
extern settings_t settings;

#define EPOLL_MAX_EVENTS		32
#define UDS_BUFFER_SIZE			2048
#define FINE_GUIDANCE_SIZE		sizeof(FineGuidanceAnglesResponse)
#define TELEMETRY_REGULAR_SIZE	sizeof(RegularTelemetry)
#define TELEMETRY_MAIN_SIZE		sizeof(MainTelemetry)

#define CAN_PACKAGE_MAX			2048
#define CAN_QUEUE_LOW_SIZE		(CAN_PACKAGE_MAX / 8)
#define CAN_QUEUE_HIGH_SIZE		(256 / 8)

#define ERROR_MESSAGE			" Error (%d) %s"

int  log_open( void );
void log_close( void );
void log_info( const char* fmt, ... );
void log_warn( const char* fmt, ... );
void log_err( const char* fmt, ... );
void log_errno( const char* msg );

#endif
