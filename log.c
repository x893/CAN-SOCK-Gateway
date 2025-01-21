#define _POSIX_SOURCE
#define __USE_POSIX2
#define __USE_BSD
#define _DEFAULT_SOURCE

#include <locale.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <syslog.h>

#include "common.h"

extern int daemonize;

#define CBLEN 256

//!!! #define USE_BAICAL_P7

#ifdef USE_BAICAL_P7

	#define WCBLEN 512

	// Baical-P7
	#include "GTypes.h"
	#include "P7_Cproxy.h"

	static hP7_Client g_hClient = NULL;
	static hP7_Trace g_hTrace = NULL;

#else

	#define P7_TRACE_LEVEL_INFO		2
	#define P7_TRACE_LEVEL_WARNING	3
	#define P7_TRACE_LEVEL_ERROR	4

#endif

int log_open( void )
{
	if ( daemonize )
	{
		openlog( "can_service", LOG_PID, LOG_LOCAL0 );
	}
#ifdef USE_BAICAL_P7
	else
	{
        g_hTrace = NULL;
        g_hClient = NULL;

        P7_Set_Crash_Handler( );
		g_hClient = P7_Client_Create( TM( "/P7.Sink=Console" ) );
		if ( g_hClient == 0 )
		{
			log_err( "P7 log initialization error" );
		}
		else
		{
			g_hTrace = P7_Trace_Create( g_hClient, TM( "TraceChannel" ), NULL );
			if ( g_hTrace == 0 )
			{
				log_err( "P7 log initialization error" );
			}
		}
	}
#endif
}

void log_close( void )
{
	if ( daemonize )
	{
		closelog( );
	}
#ifdef USE_BAICAL_P7
	else
	{
		if ( g_hTrace != NULL )
		{
			P7_Trace_Release( g_hTrace );
			g_hTrace = NULL;
		}

		if ( g_hClient != NULL )
		{
			P7_Client_Flush( g_hClient );
			P7_Clr_Crash_Handler( );
			g_hClient = NULL;
		}
	}
#endif
}

void log_custom( uint32_t level, const char* fmt, va_list ap )
{
#ifndef USE_BAICAL_P7
	const char* prefix;
	if ( level == P7_TRACE_LEVEL_INFO )
		prefix = "Info : %s\n";
	else if ( level == P7_TRACE_LEVEL_WARNING )
		prefix = "Warn : %s\n";
	else if ( level == P7_TRACE_LEVEL_ERROR )
		prefix = "Error: %s\n";
	else
		return;
#endif

	char cbuf[CBLEN];
	vsnprintf( cbuf, CBLEN, fmt, ap );

#ifdef USE_BAICAL_P7
	P7_Trace_Add( g_hTrace, 0, level, NULL, 0, NULL, NULL, TM( "%hs" ), cbuf );
#else
	printf( prefix, cbuf );
#endif
}

void log_info( const char* fmt, ... )
{
	va_list ap;
	va_start( ap, fmt );
	if ( daemonize )
		vsyslog( LOG_INFO, fmt, ap );
	else
		log_custom( P7_TRACE_LEVEL_INFO, fmt, ap );
	va_end( ap );
}

void log_warn( const char* fmt, ... )
{
	va_list ap;
	va_start( ap, fmt );
	if ( daemonize )
		vsyslog( LOG_WARNING, fmt, ap );
	else
		log_custom( P7_TRACE_LEVEL_WARNING, fmt, ap );
	va_end( ap );
}

void log_err( const char* fmt, ... )
{
	va_list ap;
	va_start( ap, fmt );
	if ( daemonize )
		vsyslog( LOG_ERR, fmt, ap );
	else
		log_custom( P7_TRACE_LEVEL_ERROR, fmt, ap );
	va_end( ap );
}

void log_errno( const char* msg )
{
	log_err( "%s" ERROR_MESSAGE, msg, errno, strerror( errno ) );
}
