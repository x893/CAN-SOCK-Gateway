#pragma region Includes

#define _POSIX_SOURCE
#define __USE_POSIX2
#define _POSIX_C_SOURCE 199309L

#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>

#include <net/if.h>

#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <sys/signalfd.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/uio.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/net_tstamp.h>

#include "common.h"
#include "endpoints.h"

#pragma endregion

#pragma region External functions

int can_initialize( int epoll_fd );
int can_get_epool_size( void );
int can_process_high( int fd );
int can_process_middle( int fd );
int can_process_low( int fd );
void can_close( void );

int uds_get_epool_size( void );
int uds_initialize( int epoll_fd );
int uds_process_high( int fd );
int uds_process_middle( int fd );
int uds_process_low( int fd );
void uds_close( void );
#pragma endregion

#pragma region Variables

int daemonize = 0;
static const char* pidname = "/var/run/can_service.pid";
struct epoll_event epoll_events[EPOLL_MAX_EVENTS];
static int sig_fd = -1;
static volatile int isRunning = 0;

settings_t settings = {
	.disable_periodic_timer = 0,
	.oneshot_timer_msec = 50,
	.verbose = 0
};

#pragma endregion

/*
static int sem_raise(sem_t* sem)
{
	int sval;

	if (sem_getvalue(sem, &sval) == (-1))
		return -1;
	if (sval == 0)
		return sem_post(sem);
	return 0;
}
*/

#pragma region Create signals
/**
 * @brief Create signals file descriptor and add it to epoll
 */
int signals_create( int epoll_fd )
{
	sigset_t sset;

	// Hook up signals
	sigemptyset( &sset );
	sigaddset( &sset, SIGTERM );
	sigaddset( &sset, SIGINT );
	sigaddset( &sset, SIGQUIT );
	sigaddset( &sset, SIGUSR1 );

	sigprocmask( SIG_SETMASK, &sset, NULL );
	sig_fd = signalfd( -1, &sset, 0 );
	if ( sig_fd < 0 )
	{
		log_errno( "Error create signal." );
		return -1;
	}
	else
	{
		// Add signal to epoll
		struct epoll_event events_setup;
		bzero( &events_setup, sizeof( events_setup ) );
		events_setup.data.fd = sig_fd;
		events_setup.events = EPOLLIN;
		if ( epoll_ctl( epoll_fd, EPOLL_CTL_ADD, sig_fd, &events_setup ) < 0 )
		{
			log_errno( "Error adding signal to poll." );
			close( sig_fd );
			return -1;
		}
	}
	return 0;
}

void signal_close( void )
{
	if ( sig_fd >= 0 )
		close( sig_fd );
	sig_fd = -1;
}
#pragma endregion

#pragma region Parse options
int parse_options( int argc, char** argv )
{
	int rc;
	while ( ( rc = getopt( argc, argv, "psv:d:" ) ) != -1 )
	{
		switch ( rc )
		{
		case 'v':
			settings.verbose = atoi( optarg );
			break;
		case 'd':
			rc = atoi( optarg );
			if ( rc >= 10 && rc <= 90 )
				settings.oneshot_timer_msec = rc;
			else
			{
				printf( "Incorrect value for FINE_GUIDANCE timeout (10-90)." );
				return -1;
			}
			break;
		case 's':
			daemonize = 1;
			break;
		case 'p':
			settings.disable_periodic_timer = 1;
			break;
		case 'h':
		default:
			printf( "Usage: %s\n", argv[0] );
			printf(
				"  -p    disable sending REGULAR TELEMETRY\n"
				"  -d nn set FINE_GUIDANCE timeout (10-90) in milliseconds\n"
				"  -s    run as demon\n"
			);
			return -1;
		}
	}
	return 0;
}

#pragma endregion

#pragma region main_process_stop

void main_process_stop( void )
{
	isRunning = 0;
}
#pragma endregion

// #pragma region Main entry point
/*
 * @brief Main entry point
 *
 **/

int main( int argc, char** argv )
{
	int epoll_fd;

	if ( parse_options( argc, argv ) < 0 )
	{
		exit( EXIT_FAILURE );
	}

#pragma region Daemonize(not used now)
	if ( daemonize )
	{
		FILE* fpid;
		int rc;
		pid_t p_pid;

		p_pid = fork( );
		if ( p_pid == ( -1 ) )
		{
			printf( "Error: Forking failure (%d) %s\n", errno, strerror( errno ) );
			exit( EXIT_FAILURE );
		}
		if ( p_pid != 0 )
		{
			printf( "PID:%d\n", p_pid );
			exit( EXIT_SUCCESS );
		}

		fpid = fopen( pidname, "w+" );
		if ( fpid )
		{
			fprintf( fpid, "%u", getpid( ) );
			fclose( fpid );
		}
		else
		{
			printf( "\nUnable to create pid file %s Error: (%d) %s\n", pidname, errno, strerror( errno ) );
		}

		umask( 0 );
		setsid( );

		rc = chdir( "/" );
		close( STDIN_FILENO );
		close( STDOUT_FILENO );
		close( STDERR_FILENO );
	}
#pragma endregion

	log_open( );
	log_info( "Starting the main process" );

	// Initialize epoll (+1 for SIGNAL SOCKET)
	epoll_fd = epoll_create( uds_get_epool_size( ) + can_get_epool_size( ) + 1 );
	if ( epoll_fd < 0 )
	{
		log_errno( "Can't create epool." );
	}
	else
	{
		// and sockets
		if ( signals_create( epoll_fd ) == 0
			&& can_initialize( epoll_fd ) == 0
			&& uds_initialize( epoll_fd ) == 0 )
		{
			isRunning = 1;
		}
	}

	int fd, n_events;

	while ( isRunning )
	{
		n_events = epoll_wait( epoll_fd, epoll_events, EPOLL_MAX_EVENTS, 500 );
		if ( n_events < 0 )
		{
			if ( errno == EINTR )
				continue;
			log_errno( "Error epoll_wait" );
			break;
		}
		if ( n_events == 0 )
			continue;

#pragma region Process high priority events
		for ( int i = 0; i < n_events; i++ )
		{
			fd = epoll_events[i].data.fd;
			if ( fd < 0 )
				continue;
			if ( uds_process_high( fd ) == 0 )
			{
				epoll_events[i].data.fd = -1;
				continue;
			}
			if ( can_process_high( fd ) == 0 )
			{
				epoll_events[i].data.fd = -1;
				continue;
			}
		}
#pragma endregion

#pragma region Process middle priority events
		for ( int i = 0; i < n_events; i++ )
		{
			fd = epoll_events[i].data.fd;
			if ( fd < 0 )
				continue;
			if ( uds_process_middle( fd ) == 0 )
			{
				epoll_events[i].data.fd = -1;
				continue;
			}
			if ( can_process_middle( fd ) == 0 )
			{
				epoll_events[i].data.fd = -1;
				continue;
			}
		}
#pragma endregion

#pragma region Process low priority events
		for ( int i = 0; i < n_events; i++ )
		{
			fd = epoll_events[i].data.fd;
			if ( fd < 0 )
				continue;
			if ( can_process_low( fd ) == 0 )
			{
				epoll_events[i].data.fd = -1;
				continue;
			}
			if ( uds_process_low( fd ) == 0 )
			{
				epoll_events[i].data.fd = -1;
				continue;
			}
			if ( fd == sig_fd )
			{
				epoll_events[i].data.fd = -1;
				struct signalfd_siginfo siginfo;
				read( sig_fd, &siginfo, sizeof( siginfo ) );
				if ( siginfo.ssi_signo == SIGUSR1 )
					log_info( "Toggling diagnostic flag" );
				else
					main_process_stop( );
				break;
			}
		}
#pragma endregion
	}

	log_info( "Shutdown the main process" );
	if ( epoll_fd >= 0 )
		close( epoll_fd );

	signal_close( );
	uds_close( );
	can_close( );
	log_close( );

	if ( daemonize )
		unlink( pidname );

	return EXIT_SUCCESS;
}
#pragma endregion
