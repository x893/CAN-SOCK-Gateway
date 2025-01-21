#pragma region Includes

#define _POSIX_SOURCE
#define __USE_POSIX2
#define __USE_MISC

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <strings.h>

#include <sys/epoll.h>
#include <sys/socket.h>
#include <sys/stat.h>

#include "common.h"
#include "endpoints.h"
#include "messages.h"

#pragma endregion

#pragma region Type definitions

typedef struct
{
	int uds_fd;
	int epoll_fd;
	struct sockaddr_un sockaddr;
} uds_context_t;

#pragma endregion

#pragma region Variables

void can_send_log_telemetry( uint8_t* uds_data, int nbytes );
void can_send_target_payload( uint8_t* uds_data, int nbytes );
void can_send_main_telemetry( uint8_t* data, int nbytes );
void can_send_extra_telemetry( uint8_t* data, int nbytes );
void can_send_fine_guidance( uint8_t* uds_data, int nbytes );
void can_oneshot_timer_stop( void );

typedef enum
{
	UDS_ORCHESTRATOR_OUT,
	UDS_FINE_GUIDANCE_OUT,
	UDS_ORCHESTRATOR_IN,
	UDS_FINE_GUIDANCE_IN,
	UDS_MAIN_TELEMETRY_IN,
	UDS_REGULAR_TELEMETRY_IN,
	UDS_EXTRA_TELEMETRY_IN,

	UDS_LOG_TELEMETRY_IN,
	UDS_TARGET_PAYLOAD_IN,

	UDS_ENDPOINT_MAX
} UDS_ENDPOINRTS_t;

#define ENDPOINT_END	0
#define ENDPOINT_INPUT	1
#define ENDPOINT_OUTPUT	2

typedef struct
{
	int uds_input;
	int uds_type;
	int uds_fd;
	int epoll_fd;
	struct sockaddr_un sockaddr;
} UDS_ENDPOINT_t;

static UDS_ENDPOINT_t uds_endpoints[] =
{
	[UDS_ORCHESTRATOR_OUT] =
		{ ENDPOINT_OUTPUT, SOCK_STREAM, -1, -1, { AF_UNIX, UDS_ORCHESTRATOR_ENDPOINT } },
	[UDS_FINE_GUIDANCE_OUT] =
		{ ENDPOINT_OUTPUT, SOCK_DGRAM,  -1, -1, { AF_UNIX, UDS_FINE_ANGLES_ENDPOINT } },
	[UDS_ORCHESTRATOR_IN] =
		{ ENDPOINT_INPUT,  SOCK_DGRAM,  -1, -1, { AF_UNIX, UDS_ORCHESTRATOR_CLIENT_ENDPOINT } },
	[UDS_FINE_GUIDANCE_IN] =
		{ ENDPOINT_INPUT,  SOCK_DGRAM,  -1, -1, { AF_UNIX, UDS_FINE_ANGLES_CLIENT_ENDPOINT } },
	[UDS_MAIN_TELEMETRY_IN] =
		{ ENDPOINT_INPUT, SOCK_DGRAM,   -1, -1, { AF_UNIX, UDS_MAIN_TELEMETRY_ENDPOINT } },
	[UDS_REGULAR_TELEMETRY_IN] =
		{ ENDPOINT_INPUT, SOCK_DGRAM,   -1, -1, { AF_UNIX, UDS_REGULAR_TELEMETRY_ENDPOINT } },
	[UDS_EXTRA_TELEMETRY_IN] =
		{ ENDPOINT_INPUT, SOCK_DGRAM,   -1, -1, { AF_UNIX, UDS_EXTRA_TELEMETRY_ENDPOINT } },
	[UDS_LOG_TELEMETRY_IN] =
		{ ENDPOINT_INPUT, SOCK_DGRAM,   -1, -1, { AF_UNIX, UDS_LOG_TELEMETRY_ENDPOINT  } },
	[UDS_TARGET_PAYLOAD_IN] =
		{ ENDPOINT_INPUT, SOCK_DGRAM,   -1, -1, { AF_UNIX, UDS_TARGET_PAYLOAD_ENDPOINT  } },

	[UDS_ENDPOINT_MAX] = { ENDPOINT_END }
};

static pthread_mutex_t regular_telemetry_mutex = PTHREAD_MUTEX_INITIALIZER;
static uint8_t regular_telemetry_data[sizeof( RegularTelemetry )];
static uint8_t uds_data[UDS_BUFFER_SIZE];

#pragma endregion

/*
 * @briesf Return nnumber of binding (input) sockets
 */
int uds_get_epool_size( void )
{
	return UDS_ENDPOINT_MAX;
}

#pragma region Reconnect UDS

int uds_orchestration_reconnect( int fd )
{
	UDS_ENDPOINT_t* uds_data = &uds_endpoints[UDS_ORCHESTRATOR_OUT];
	struct stat uds_stat = { 0 };
	char* path = &uds_data->sockaddr.sun_path[0];

	if ( stat( path, &uds_stat ) == 0 )
	{
		if ( fd < 0 )
		{
			if ( uds_data->uds_fd >= 0 )
			{
				close( uds_data->uds_fd );
				uds_data->uds_fd = -1;
			}
			fd = socket( AF_UNIX, SOCK_STREAM, 0 );
			if ( fd < 0 )
			{
				log_err( "Failed to create socket for '%s', Error (%d) %s", path, errno, strerror( errno ) );
				return -1;
			}
			uds_data->uds_fd = fd;
		}
		if ( connect( fd, (struct sockaddr*) &uds_data->sockaddr, sizeof( uds_data->sockaddr ) ) < 0 )
		{
			log_err( "Can't connect to %s" ERROR_MESSAGE, path, errno, strerror( errno ) );
			return -1;
		}
	}
	else
	{
		log_err( "Socket path not exist %s" ERROR_MESSAGE, path, errno, strerror( errno ) );
		return -1;
	}
	return 0;
}

#pragma endregion

#pragma region Close UDS
void uds_close_one( UDS_ENDPOINT_t* uds_endpoint )
{
	if ( uds_endpoint->uds_fd >= 0 )
	{
		if ( uds_endpoint->epoll_fd >= 0 )
		{
			epoll_ctl( uds_endpoint->epoll_fd, EPOLL_CTL_ADD, uds_endpoint->uds_fd, NULL );
			remove( (const char*) &uds_endpoint->sockaddr.sun_path );
		}
		close( uds_endpoint->uds_fd );
		uds_endpoint->uds_fd = -1;
	}
	uds_endpoint->epoll_fd = -1;
}

void uds_close( void )
{
	for ( UDS_ENDPOINT_t* p = &uds_endpoints[0]; p->uds_input != ENDPOINT_END; p++ )
	{
		uds_close_one( p );
	}
}
#pragma endregion

#pragma region Create UDS

int uds_create( int epoll_fd, UDS_ENDPOINT_t* uds_endpoint )
{
	uds_close_one( uds_endpoint );

	int uds_fd = socket( AF_UNIX, uds_endpoint->uds_type, 0 );
	if ( uds_fd < 0 )
	{
		log_err( "Failed to create socket for '%s', Error (%d) %s",
			uds_endpoint->sockaddr.sun_path,
			errno, strerror( errno ) );
		return -1;
	}

	if ( epoll_fd < 0 )
	{
		if ( uds_endpoint->uds_type == SOCK_STREAM )
		{
			uds_orchestration_reconnect( uds_endpoint->uds_fd );
		}
	}
	else
	{
		struct stat uds_stat = { 0 };
		if ( stat( uds_endpoint->sockaddr.sun_path, &uds_stat ) == 0 )
		{
			if ( remove( uds_endpoint->sockaddr.sun_path ) && errno != ENOENT )
				log_err( "Failed to remove socket path '%s'. Error (%d) %s",
					uds_endpoint->sockaddr.sun_path,
					errno, strerror( errno ) );
		}

		if ( bind( uds_fd, (struct sockaddr*) &uds_endpoint->sockaddr, sizeof( uds_endpoint->sockaddr ) ) < 0 )
		{
			log_err( "Failed bind socket to '%s'. Error (%d) %s",
				uds_endpoint->sockaddr.sun_path,
				errno, strerror( errno ) );
			close( uds_fd );
			return -1;
		}

		struct epoll_event event_setup = { .events = EPOLLIN };
		bzero( &event_setup, sizeof( event_setup ) );
		event_setup.events = EPOLLIN;
		event_setup.data.fd = uds_fd;
		if ( epoll_ctl( epoll_fd, EPOLL_CTL_ADD, uds_fd, &event_setup ) < 0 )
		{
			log_err( "Unable to add socket '%s' to epoll queue. Error (%d) %s",
				uds_endpoint->sockaddr.sun_path,
				errno, strerror( errno ) );
			close( uds_fd );
			return -1;
		}
	}
	log_info( "Open socket:%3d (%s) '%s'",
		uds_fd,
		( epoll_fd < 0 ) ? "out" : "in ",
		uds_endpoint->sockaddr.sun_path );

	uds_endpoint->uds_fd = uds_fd;
	uds_endpoint->epoll_fd = epoll_fd;
	return 0;
}
#pragma endregion

#pragma region Initialize UDS
int uds_initialize( int epoll_fd )
{
	int i, j, ret;
	int nbytes;
	struct stat uds_path = { 0 };

	if ( stat( CSG_UDS_PATH, &uds_path ) == -1 )
	{
		if ( mkdir( CSG_UDS_PATH, 0700 ) != 0 )
		{
			log_err( "Error create folder: '%s'." ERROR_MESSAGE, CSG_UDS_PATH, errno, strerror( errno ) );
			return -1;
		}
	}

	for ( UDS_ENDPOINT_t* p = &uds_endpoints[0]; p->uds_input != ENDPOINT_END; p++ )
	{
		if ( uds_create( ( p->uds_input == ENDPOINT_OUTPUT ? -1 : epoll_fd ), p ) < 0 )
			return -1;
	}
	return 0;
}
#pragma endregion

#pragma region Process high priority events
int uds_process_high( int fd )
{
	if ( fd >= 0 )
	{
		if ( fd == uds_endpoints[UDS_FINE_GUIDANCE_IN].uds_fd )
		{
			can_oneshot_timer_stop( );
			int nbytes = read( fd, uds_data, sizeof( uds_data ) );
			can_send_fine_guidance( uds_data, nbytes );
			return 0;
		}
	}
	return -1;
}
#pragma endregion

#pragma region Process middle priority events
int uds_process_middle( int fd )
{
	return -1;
}
#pragma endregion

#pragma region Process low priority events
int uds_process_low( int fd )
{
	int nbytes;

	if ( fd >= 0 )
	{
		if ( fd == uds_endpoints[UDS_ORCHESTRATOR_IN].uds_fd )
		{
			nbytes = read( fd, uds_data, sizeof( uds_data ) );
			log_info( "Recieve %d bytes from UDS_ORCHESTRATOR", nbytes );
			return 0;
		}

		if ( fd == uds_endpoints[UDS_REGULAR_TELEMETRY_IN].uds_fd )
		{
			nbytes = (int) read( fd, uds_data, sizeof( uds_data ) );
			if ( nbytes == sizeof( regular_telemetry_data ) )
			{
				pthread_mutex_lock( &regular_telemetry_mutex );
				memcpy( regular_telemetry_data, uds_data, nbytes );
				pthread_mutex_unlock( &regular_telemetry_mutex );
			}
			else
			{
				log_err( "REGULAR_TELEMETRY package incorrect size %d/%d", nbytes, sizeof( regular_telemetry_data ) );
			}
			return 0;
		}

		if ( fd == uds_endpoints[UDS_EXTRA_TELEMETRY_IN].uds_fd )
		{
			nbytes = (int) read( fd, uds_data, sizeof( uds_data ) );
			if ( nbytes <= sizeof( ExtraTelemetry ) )
				can_send_extra_telemetry( uds_data, nbytes );
			else
				log_err( "EXTRA_TELEMETRY package too long %d/%d", nbytes, sizeof( ExtraTelemetry ) );
			return 0;
		}

		if ( fd == uds_endpoints[UDS_MAIN_TELEMETRY_IN].uds_fd )
		{
			nbytes = (int) read( fd, uds_data, sizeof( uds_data ) );
			if ( nbytes <= TELEMETRY_MAIN_SIZE )
				can_send_main_telemetry( uds_data, nbytes );
			else
				log_err( "MAIN_TELEMETRY package too long %d/%d", nbytes, TELEMETRY_MAIN_SIZE );
			return 0;
		}

		if ( fd == uds_endpoints[UDS_LOG_TELEMETRY_IN].uds_fd )
		{
			nbytes = (int) read( fd, uds_data, sizeof( uds_data ) );
			if ( nbytes == sizeof( LogTelemetry ) )
				can_send_log_telemetry( uds_data, nbytes );
			else
				log_err( "LOG_TELEMETRY package incorrect size %d/%d", nbytes, sizeof( LogTelemetry ) );
			return 0;
		}

		if ( fd == uds_endpoints[UDS_TARGET_PAYLOAD_IN].uds_fd )
		{
			nbytes = (int) read( fd, uds_data, sizeof( uds_data ) );
			if ( nbytes == sizeof( TargetPayload ) )
				can_send_target_payload( uds_data, nbytes );
			else
				log_err( "TARGET_PAYLOAD package incorrect size %d/%d", nbytes, sizeof( TargetPayload ) );
			return 0;
		}
	}
	return -1;
}
#pragma endregion

#pragma region Get REGULAR telemetry data
/*
 * @brief Get regular telemetry data
 */
uint8_t* uds_regular_telemetry_lock( void )
{
	pthread_mutex_lock( &regular_telemetry_mutex );
	return regular_telemetry_data;
}

void uds_regular_telemetry_unlock( void )
{
	pthread_mutex_unlock( &regular_telemetry_mutex );
}
#pragma endregion

#pragma region Send FINE GUIDANCE command
int uds_send_fine_guidance( uint8_t cmd )
{
	uint8_t buffer[1] = { cmd };
	int rc;
	if ( uds_endpoints[UDS_FINE_GUIDANCE_OUT].uds_fd < 0 )
	{
		log_err( "UDS_FINE_GUIDANCE socket not open." );
		return -1;
	}
	if ( sizeof( buffer ) != sendto(
		uds_endpoints[UDS_FINE_GUIDANCE_OUT].uds_fd,
		buffer,
		sizeof( buffer ),
		0,
		(struct sockaddr*) &uds_endpoints[UDS_FINE_GUIDANCE_OUT].sockaddr,
		sizeof( struct sockaddr_un ) ) )
	{
		log_errno( "Error sending UDS_FINE_GUIDANCE request." );
		return -1;
	}
	return 0;
}
#pragma endregion

#pragma region Send ORCHESTRATOR command

int uds_send_orchestrator( uint16_t id, uint8_t* buffer, uint16_t idx )
{
	OrchestratorRequestHeader header;
	header.id = id;
	header.payload_length = idx;

	if ( uds_endpoints[UDS_ORCHESTRATOR_OUT].uds_fd < 0 )
	{
		log_err( "UDS_ORCHESTRATOR socket not open." );
		return -1;
	}

	if ( sizeof( OrchestratorRequestHeader )
		!= send( uds_endpoints[UDS_ORCHESTRATOR_OUT].uds_fd, (uint8_t*) &header, sizeof( OrchestratorRequestHeader ), MSG_NOSIGNAL ) )
	{
		if ( uds_orchestration_reconnect( -1 ) == 0 )
		{
			// Try after reconnect
			if ( sizeof( OrchestratorRequestHeader )
				!= send( uds_endpoints[UDS_ORCHESTRATOR_OUT].uds_fd, (uint8_t*) &header, sizeof( OrchestratorRequestHeader ), MSG_NOSIGNAL ) )
			{
				log_errno( "Error sending UDS_ORCHESTRATOR header." );
				return -1;
			}
		}
		else
			return -1;
	}

	if ( idx > 0 )
	{
		if ( idx != send( uds_endpoints[UDS_ORCHESTRATOR_OUT].uds_fd, buffer, idx, MSG_NOSIGNAL ) )
		{
			if ( uds_orchestration_reconnect( -1 ) == 0 )
			{
				// Try after reconnect
				if ( idx != send( uds_endpoints[UDS_ORCHESTRATOR_OUT].uds_fd, buffer, idx, MSG_NOSIGNAL ) )
				{
					log_errno( "Error sending UDS_ORCHESTRATOR data." );
					return -1;
				}
			}
			else
				return -1;
		}
	}
	/*
	if ( sizeof(uds_header_t) != sendto(uds_orchestrator_out.uds_fd,
		(uint8_t*) header,
		sizeof(uds_header_t),
		MSG_NOSIGNAL,
		(struct sockaddr*) &uds_orchestrator_out.sockaddr,
		sizeof(struct sockaddr_un)) )
	{
		log_errno("Error sending UDS_ORCHESTRATOR header.");
		return -1;
	}
	if ( idx > 0 )
	{
		if ( idx != sendto(uds_orchestrator_out.uds_fd,
			buffer,
			idx,
			MSG_NOSIGNAL,
			(struct sockaddr*) &uds_orchestrator_out.sockaddr,
			sizeof(struct sockaddr_un)) )
		{
			log_errno("Error sending UDS_ORCHESTRATOR data.");
			return -1;
		}
	}
	*/
	return 0;
}
#pragma endregion
