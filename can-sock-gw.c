#pragma region Includes

#define _POSIX_SOURCE
#define __USE_POSIX2
#define __USE_MISC
#define _POSIX_C_SOURCE 199309L

#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <time.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <sys/signalfd.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/timerfd.h>
#include <sys/types.h>
#include <sys/uio.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>
#include <linux/if.h>
#include <linux/net_tstamp.h>

#include "common.h"
#include "endpoints.h"
#include "messages.h"

#pragma endregion

#pragma region Definitions

#define PT_KEEPALIVE_INTERVAL_SEC 5

#define CAN_ID_DATA_MASK 0x1FFF00FFU

#define CAN_ID_CODE_MASK		0x00FFU
#define CAN_ID_ATTRIBUTE_MASK	0xFF00U
#define CAN_ID_PACK_INC			0x0100U
#define CAN_ID_PACK_INC_MASK	0x0F00U
#define CAN_ID_PACK_FIRST		0x0000U
#define CAN_ID_PACK_MIDDLE		0x4000U
#define CAN_ID_PACK_LAST		0x8000U
#define CAN_ID_PACK_LAST_ACK	0xC000U
#define CAN_ID_PACK_MASK		0xF000U
#define CAN_ID_PACK_TYPE_MASK	0xC000U

#define CAN_ID_RECIEVER_POS		16
#define CAN_ID_RECIEVER_MASK	(0x3FU << CAN_ID_RECIEVER_POS)
#define CAN_ID_SENDER_POS		22
#define CAN_ID_SENDER_MASK		(0x3FU << CAN_ID_SENDER_POS)

#define CAN_ID_IS_DATA			0x10000000U
#define CAN_ID_IS_CMD			0x00000000U

#define CMD_APISOS				0x11U

#define CMD_TARGET_PAYLOAD_DATA	0xF2U
#define CMD_LOG_TELEMETRY_DATA	0xF3U

#define CMD_TIMESTAMP			0xF5U
#define CMD_POWER_OFF			0xF9U
#define CMD_REGULAR_DATA		0xFAU
#define CMD_MAIN_DATA			0xFBU
#define CMD_EXTRA_DATA			0xFCU
#define CMD_FINE_GUIDANCE		0xFEU

#define CAN_CMD_WITH_DATA		0x2000U
#define CAN_CMD_NEED_ACK		0x4000U
#define CAN_CMD_BAD_ACK			0xC000U
#define CAN_CMD_ACK				0x8000U
#define CAN_COUNT_ACK			0x3F00U

#define CENTER_ADDRESS			0x01U
#define NODE_ADDRESS			0x29U
#define APISOS_ADDRESS			0x0CU
#define TIME_0A_ADDRESS			0x0AU
#define TIME_0B_ADDRESS			0x0BU

#define BCAST0_ADDRESS			0x00U

#define CENTER_RECIEVER_ID		((CENTER_ADDRESS << CAN_ID_RECIEVER_POS) & CAN_ID_RECIEVER_MASK)
#define CENTER_SENDER_ID		((CENTER_ADDRESS << CAN_ID_SENDER_POS) & CAN_ID_SENDER_MASK)
#define APISOS_SENDER_ID		((APISOS_ADDRESS << CAN_ID_SENDER_POS) & CAN_ID_SENDER_MASK)
#define APISOS_RECIEVER_ID		((APISOS_ADDRESS << CAN_ID_RECIEVER_POS) & CAN_ID_RECIEVER_MASK)
#define BCAST0_RECIEVER_ID		((BCAST0_ADDRESS << CAN_ID_RECIEVER_POS) & CAN_ID_RECIEVER_MASK)
#define NODE_RECIEVER_ID		((NODE_ADDRESS << CAN_ID_RECIEVER_POS) & CAN_ID_RECIEVER_MASK)
#define NODE_SENDER_ID			((NODE_ADDRESS << CAN_ID_SENDER_POS) & CAN_ID_SENDER_MASK)
#define TIME_SENDER_0A_ID		((TIME_0A_ADDRESS << CAN_ID_SENDER_POS) & CAN_ID_SENDER_MASK)
#define TIME_SENDER_0B_ID		((TIME_0B_ADDRESS << CAN_ID_SENDER_POS) & CAN_ID_SENDER_MASK)

#define CAN_ID_EXTRA_TELEMETRY		(CAN_EFF_FLAG | CAN_ID_IS_DATA | NODE_SENDER_ID | CENTER_RECIEVER_ID | CAN_ID_PACK_FIRST | CMD_EXTRA_DATA)
#define CAN_ID_MAIN_TELEMETRY		(CAN_EFF_FLAG | CAN_ID_IS_DATA | NODE_SENDER_ID | CENTER_RECIEVER_ID | CAN_ID_PACK_FIRST | CMD_MAIN_DATA)
#define CAN_ID_REGULAR_TELEMETRY	(CAN_EFF_FLAG | CAN_ID_IS_DATA | NODE_SENDER_ID | CENTER_RECIEVER_ID | CAN_ID_PACK_FIRST | CMD_REGULAR_DATA)
#define CAN_ID_LOG_TELEMETRY		(CAN_EFF_FLAG | CAN_ID_IS_DATA | NODE_SENDER_ID | CENTER_RECIEVER_ID | CAN_ID_PACK_FIRST | CMD_LOG_TELEMETRY_DATA)
#define CAN_ID_TARGET_PAYLOAD		(CAN_EFF_FLAG | CAN_ID_IS_DATA | NODE_SENDER_ID | CENTER_RECIEVER_ID | CAN_ID_PACK_FIRST | CMD_TARGET_PAYLOAD_DATA)



#define CAN_ID_TIMESTAMP_MASK	(CAN_ID_IS_DATA | CAN_ID_SENDER_MASK | CAN_ID_RECIEVER_MASK | )
#define CAN_ID_TIMESTAMP_0A		(CAN_EFF_FLAG | CAN_ID_IS_DATA | TIME_SENDER_0A_ID | CAN_CMD_WITH_DATA | CMD_TIMESTAMP)
#define CAN_ID_TIMESTAMP_0B		(CAN_EFF_FLAG | CAN_ID_IS_DATA | TIME_SENDER_0B_ID | CAN_CMD_WITH_DATA | CMD_TIMESTAMP)

#define TIMESTAMP_0A_ID			( CAN_ID_IS_CMD | TIME_SENDER_0A_ID | CAN_CMD_WITH_DATA | CMD_TIMESTAMP )
#define TIMESTAMP_0B_ID			( CAN_ID_IS_CMD | TIME_SENDER_0B_ID | CAN_CMD_WITH_DATA | CMD_TIMESTAMP )
#define TIMESTAMP_01_ID			( CAN_ID_IS_CMD | CENTER_SENDER_ID | CAN_CMD_WITH_DATA | CMD_TIMESTAMP )

#define POWER_OFF_SENDER_04_ID	((0x04U << CAN_ID_SENDER_POS) & CAN_ID_SENDER_MASK)
#define POWER_OFF_SENDER_05_ID	((0x05U << CAN_ID_SENDER_POS) & CAN_ID_SENDER_MASK)
#define POWER_OFF_SENDER_06_ID	((0x06U << CAN_ID_SENDER_POS) & CAN_ID_SENDER_MASK)
#define POWER_OFF_SENDER_07_ID	((0x07U << CAN_ID_SENDER_POS) & CAN_ID_SENDER_MASK)

#define POWER_OFF_04_ID			(POWER_OFF_SENDER_04_ID | CAN_CMD_WITH_DATA | CMD_POWER_OFF )
#define POWER_OFF_05_ID			(POWER_OFF_SENDER_05_ID | CAN_CMD_WITH_DATA | CMD_POWER_OFF )
#define POWER_OFF_06_ID			(POWER_OFF_SENDER_06_ID | CAN_CMD_WITH_DATA | CMD_POWER_OFF )
#define POWER_OFF_07_ID			(POWER_OFF_SENDER_07_ID | CAN_CMD_WITH_DATA | CMD_POWER_OFF )

#define UDS_ORCHESTRATOR_CMD	0x000
#define UDS_ORCHESTRATOR_DATA	0x100

#pragma endregion

#pragma region Type definitions

typedef struct
{
	int can_fd;
	struct can_frame frame;
} can_job_t;

typedef struct
{
	pthread_mutex_t lock;
	pthread_cond_t wait_slot_free;
	pthread_cond_t wait_slot_ready;
	int size;
	volatile int head;
	volatile int tail;
	can_job_t* jobs;
} can_queue_t;

typedef struct
{
	int fail_count;
	int can_fd;
} can_data_t;

#define USE_CAN_AUTO	0
#define USE_CAN_0		1
#define USE_CAN_1		2

#define USE_NEXT_CAN_0	0
#define USE_NEXT_CAN_1	1

typedef struct
{
	can_data_t can0;
	can_data_t can1;
	int mode_select;
	int next_use_can;
	int success_count;
	int periodic_timer_fd;
	int oneshot_timer_fd;
	sem_t sem_terminate;
	pthread_t thread_can;
	pthread_mutex_t queue_mutex;
	pthread_cond_t queue_ready;
	struct itimerspec oneshot_timer_start;
	struct itimerspec oneshot_timer_stop;
} can_context_t;

static can_context_t cans = {
	.can0 = {.fail_count = 0, .can_fd = -1},
	.can1 = {.fail_count = 0, .can_fd = -1},
	.mode_select = USE_CAN_AUTO,
	.next_use_can = USE_NEXT_CAN_0,
	.success_count = 0,
	.periodic_timer_fd = -1,
	.oneshot_timer_fd = -1,
};

typedef struct
{
	int can_fd;
	canid_t can_id;
} fine_guidance_state_t;

typedef struct
{
	int can_fd;
	canid_t can_id;
	canid_t last_can_id;
	uint16_t number; // package sequence number
	uint16_t index;  // index in buffer array
	uint8_t data[CAN_PACKAGE_MAX];
} data_receive_state_t;

#pragma endregion

#pragma region External functions

uint8_t* uds_regular_telemetry_lock( void );
void uds_regular_telemetry_unlock( void );
int uds_send_fine_guidance( uint8_t cmd );
int uds_send_orchestrator( uint16_t id, uint8_t* buffer, uint16_t idx );
void main_process_stop( void );

#pragma endregion

#pragma region Variables

static data_receive_state_t data_recv_state;
fine_guidance_state_t fine_guidance_state = { 0 };

static uint8_t regular_data[TELEMETRY_REGULAR_SIZE];

static can_queue_t can_queue_low;
static can_queue_t can_queue_high;
static can_job_t can_jobs_low[CAN_QUEUE_LOW_SIZE];
static can_job_t can_jobs_high[CAN_QUEUE_HIGH_SIZE];

static struct can_filter can_filter[3] = {
	{.can_id = NODE_RECIEVER_ID, .can_mask = CAN_ID_RECIEVER_MASK},
	{.can_id = BCAST0_RECIEVER_ID, .can_mask = CAN_ID_RECIEVER_MASK}
};

#pragma endregion

/*
 * @briesf Return nnumber of binding (input) sockets
 */
int can_get_epool_size( void )
{
	// CAN0
	// CAN1
	// PERIODIC TIMER
	// ONE-SHOT TIMER
	return 4;
}

#pragma region Queue Helpers
int can_queue_initialize( can_queue_t* queue, int slots, can_job_t* jobs )
{
	pthread_mutex_init( &queue->lock, NULL );
	pthread_cond_init( &queue->wait_slot_free, NULL );
	pthread_cond_init( &queue->wait_slot_ready, NULL );

	pthread_mutex_lock( &queue->lock );
	queue->size = slots;
	queue->head = 0;
	queue->tail = 0;
	queue->jobs = jobs;
	pthread_mutex_unlock( &queue->lock );

	return 0;
}

can_job_t* can_queue_get( can_queue_t* queue )
{
	can_job_t* job = NULL;

	pthread_mutex_lock( &queue->lock );
	if ( queue->head != queue->tail )
	{
		//	while (queue->head == queue->tail)
		//		pthread_cond_wait(&queue->wait_slot_ready, &queue->lock);

		job = &queue->jobs[queue->tail];
		queue->tail = ( queue->tail + 1 ) % queue->size;
		pthread_cond_signal( &queue->wait_slot_free );
	}
	pthread_mutex_unlock( &queue->lock );
	return job;
}

can_job_t* can_queue_get_any( void )
{
	can_job_t* job;
	for ( ;;)
	{
		job = can_queue_get( &can_queue_high );
		if ( job != NULL )
			return job;
		job = can_queue_get( &can_queue_low );
		if ( job != NULL )
			return job;

		pthread_mutex_lock( &cans.queue_mutex );
		pthread_cond_wait( &cans.queue_ready, &cans.queue_mutex );
		pthread_mutex_unlock( &cans.queue_mutex );
	}
}

int can_queue_put( can_queue_t* queue, can_job_t* job )
{
	pthread_mutex_lock( &queue->lock );
	while ( ( queue->head + 1 ) % queue->size == queue->tail )
		pthread_cond_wait( &queue->wait_slot_free, &queue->lock );

	if ( job != NULL )
		memcpy( &queue->jobs[queue->head], job, sizeof( can_job_t ) );
	else
		queue->jobs[queue->head].can_fd = -1;
	queue->head = ( queue->head + 1 ) % queue->size;

	pthread_cond_signal( &queue->wait_slot_ready );
	pthread_cond_signal( &cans.queue_ready );

	pthread_mutex_unlock( &queue->lock );
	return 0;
}

void can_queue_clear( can_queue_t* queue )
{
	pthread_mutex_lock( &queue->lock );
	queue->head = 0;
	queue->tail = 0;
	pthread_mutex_unlock( &queue->lock );
}
#pragma endregion

#pragma region MessageCAN

static const char* messageCanError = "Error Message CAN";

char* MessageCAN( char* dst, const char* prefix, int fd, struct can_frame* frame )
{
	if ( settings.verbose != 0 )
		return NULL;

	char* m = dst;
	int len = sprintf( m, "%s FD:%d Len:%u %08X#", prefix, fd, frame->can_dlc, frame->can_id );
	if ( len > 0 )
	{
		m += len;
		if ( frame->can_dlc != 0 )
		{
			for ( len = 0; len < frame->can_dlc; len++ )
			{
				if ( sprintf( m, "%02X", frame->data[len] ) != 2 )
				{
					dst = NULL;
					break;
				}
				m += 2;
			}
		}
	}
	else
		dst = NULL;

	if ( dst == NULL )
		dst = (char*) messageCanError;

	return dst;
}
#pragma endregion

#pragma region Send CAN data
/*
 * @brief Send CAN data
 */
static int can_out( int fd, canid_t can_id, uint8_t* src, size_t nbytes,
	bool use_last, bool log, bool high_priority )
{
	struct can_frame frame;
	int len, rc;

	if ( fd == -1 )
	{
		// Use auto CAN select
		if ( cans.mode_select == USE_CAN_AUTO )
		{
			if ( cans.next_use_can == USE_NEXT_CAN_0 )
			{
				// Use CAN channel 0, next time use CAN 1
				fd = cans.can0.can_fd;
				cans.next_use_can = USE_NEXT_CAN_1;
			}
			else
			{
				// Use CAN channel 1, next time use CAN 0
				fd = cans.can1.can_fd;
				cans.next_use_can = USE_NEXT_CAN_0;
			}
		}
		else if ( cans.mode_select == USE_CAN_0 )
		{
			fd = cans.can0.can_fd;
		}
		else if ( cans.mode_select == USE_CAN_1 )
		{
			fd = cans.can1.can_fd;
		}
	}

	if ( fd < 0 )
		return -1;
	if ( nbytes > CAN_PACKAGE_MAX )
	{
		log_err( "CAN package too long %d max:%d", nbytes, CAN_PACKAGE_MAX );
		return -1;
	}

	can_id |= CAN_EFF_FLAG;

	for ( ;;)
	{
		len = ( nbytes > 8 ) ? 8 : nbytes;
		nbytes -= len;

		if ( use_last && nbytes == 0 && ( ( can_id & CAN_ID_IS_DATA ) != 0 ) )
		{
			can_id = ( can_id & ~CAN_ID_PACK_MASK ) | CAN_ID_PACK_LAST;
		}

		can_job_t job;
		job.can_fd = fd;
		job.frame.can_id = can_id;
		job.frame.can_dlc = len;
		if ( len > 0 && src != NULL )
		{
			memcpy( job.frame.data, src, len );
			src += len;
		}
		can_queue_put( ( high_priority ) ? &can_queue_high : &can_queue_low, &job );
		can_id = 0 | ( can_id & ~( CAN_ID_PACK_INC_MASK | CAN_ID_PACK_MASK ) ) |
			( ( can_id + CAN_ID_PACK_INC ) & CAN_ID_PACK_INC_MASK ) |
			CAN_ID_PACK_MIDDLE;

		if ( nbytes == 0 )
			break;
	}
	return 0;
}
#pragma endregion

#pragma region Send FINE GUIDANCE data
/*
 * @brief Send FINE GUIDANCE data
 */
static void can_out_apisos( int fd, canid_t can_id, uint8_t* data )
{
	const uint8_t apisos_bad_data[FINE_GUIDANCE_SIZE] = { 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F };

	if ( data == NULL )
		data = (uint8_t*) &apisos_bad_data[0];
	can_out( fd,
		( ( can_id & CAN_ID_SENDER_MASK ) >> ( CAN_ID_SENDER_POS - CAN_ID_RECIEVER_POS ) ) | ( CAN_ID_IS_DATA | NODE_SENDER_ID | CMD_FINE_GUIDANCE ),
		data,
		FINE_GUIDANCE_SIZE, false, false, true
	);
}
#pragma endregion

#pragma region Start / Stop oneshot timer
int can_oneshot_timer_start( void )
{
	if ( cans.oneshot_timer_fd >= 0 )
	{
		if ( timerfd_settime( cans.oneshot_timer_fd, 0, &cans.oneshot_timer_start,
			NULL ) >= 0 )
			return 0;

		log_errno( "Can't set one-shot timer." );
		close( cans.oneshot_timer_fd );
	}
	return -1;
}
void can_oneshot_timer_stop( void )
{
	if ( cans.oneshot_timer_fd >= 0 )
		timerfd_settime( cans.oneshot_timer_fd, 0, &cans.oneshot_timer_stop, NULL );
}
#pragma endregion

#pragma region Processing received CAN data
static bool IsEqualLast( struct can_frame* pframe )
{
	canid_t can_id = pframe->can_id & ( CAN_ID_DATA_MASK | CAN_ID_ATTRIBUTE_MASK );
	static struct can_frame last_frame = { 0 };

	if ( last_frame.can_id == can_id )
	{
		if ( last_frame.can_dlc == pframe->can_dlc )
		{
			if ( pframe->can_dlc != 0 )
			{
				if ( memcmp( &last_frame.data, pframe->data, pframe->can_dlc ) == 0 )
					return true;
			}
			else
				return true;
		}
	}
	last_frame.can_id = can_id;
	last_frame.can_dlc = pframe->can_dlc;
	if ( pframe->can_dlc != 0 )
		memcpy( &last_frame.data, pframe->data, pframe->can_dlc );
	return false;
}

static void can_in( int fd, struct can_frame* pframe )
{
	canid_t can_id = pframe->can_id & ( CAN_ID_DATA_MASK | CAN_ID_ATTRIBUTE_MASK );
	int rc, len = pframe->can_dlc;
	uint8_t buffer[8];

	if ( ( can_id & CAN_ID_RECIEVER_MASK ) == BCAST0_RECIEVER_ID )
	{
		if ( can_id == TIMESTAMP_0A_ID || can_id == TIMESTAMP_0B_ID || can_id == TIMESTAMP_01_ID )
		{
			if ( len == 8 )
			{
				uds_send_orchestrator(
					UDS_ORCHESTRATOR_CMD | ( can_id & CAN_ID_CODE_MASK ),
					(uint8_t*) &pframe->data,
					len );
			}
		}
		else if ( can_id == POWER_OFF_04_ID || can_id == POWER_OFF_05_ID || can_id == POWER_OFF_06_ID || can_id == POWER_OFF_07_ID )
		{
			if ( len == 8 )
			{
				uds_send_orchestrator(
					UDS_ORCHESTRATOR_CMD | ( can_id & CAN_ID_CODE_MASK ),
					(uint8_t*) &pframe->data,
					len );
			}
		}
		else
		{
			if ( settings.verbose <= 1 )
			{
				char msg[128];
				char* m = MessageCAN( msg, "Unknown Broadcast", fd, pframe );
				if ( m != NULL )
					log_warn( m );
			}
		}
		return;
	}

	if ( ( can_id & CAN_ID_RECIEVER_MASK ) != NODE_RECIEVER_ID )
		return;

	canid_t sender = can_id & CAN_ID_SENDER_MASK;

	if ( ( can_id & CAN_ID_IS_DATA ) != 0 )
	{
		canid_t can_id_recv = data_recv_state.can_id;

		if ( sender != CENTER_SENDER_ID )
			return;

		// Recieve the same as previous CAN package
		if ( IsEqualLast( pframe ) )
			return;

		data_recv_state.can_id = 0;

		if ( ( can_id & CAN_ID_PACK_TYPE_MASK ) == CAN_ID_PACK_FIRST )
		{
			if ( ( can_id & CAN_ID_PACK_INC_MASK ) == 0 )
			{
				data_recv_state.can_id = can_id;
				data_recv_state.index = 0;
				data_recv_state.can_fd = fd;
				data_recv_state.number = CAN_ID_PACK_INC;
			}
			else
			{
				log_err( "First data package with non-zero counter" );
				return;
			}
		}
		else if ( ( can_id & CAN_ID_PACK_LAST ) == CAN_ID_PACK_LAST )
		{
			if ( can_id_recv == 0 )
			{
				if ( ( can_id & CAN_ID_PACK_INC_MASK ) == 0 )
				{
					data_recv_state.can_id = can_id;
					data_recv_state.index = 0;
					data_recv_state.can_fd = fd;
					data_recv_state.number = 0;
				}
				else
				{
					log_err( "The only data packet with a non-zero counter" );
					return;
				}
			}
			else if ( ( can_id_recv & CAN_ID_DATA_MASK ) !=
				( can_id & CAN_ID_DATA_MASK ) )
			{
				log_err( "Can_ID is different from the previous data packet" );
				return;
			}
			else if ( data_recv_state.number !=
				(uint16_t) ( can_id & CAN_ID_PACK_INC_MASK ) )
			{
				log_err( "Data packet counter is not persistent" );
				return;
			}
			else
			{
				data_recv_state.can_id = can_id;
				data_recv_state.number =
					( data_recv_state.number + CAN_ID_PACK_INC ) & CAN_ID_PACK_INC_MASK;
			}
		}
		else if ( ( can_id & CAN_ID_PACK_TYPE_MASK ) == CAN_ID_PACK_MIDDLE )
		{
			if ( ( can_id_recv & CAN_ID_DATA_MASK ) != ( can_id & CAN_ID_DATA_MASK ) )
			{
				log_err( "Can_ID is different from the previous data packet" );
				return;
			}
			else if ( data_recv_state.number != (uint16_t) ( can_id & CAN_ID_PACK_INC_MASK ) )
			{
				log_err( "Data packet counter is not persistent" );
				return;
			}
			else
			{
				data_recv_state.can_id = can_id;
				data_recv_state.number = ( data_recv_state.number + CAN_ID_PACK_INC ) & CAN_ID_PACK_INC_MASK;
			}
		}
		else
		{
			log_err( "Unknown data package attribute" );
			return;
		}

		if ( data_recv_state.can_id != 0 && len > 0 )
		{
			if ( data_recv_state.index + len > CAN_PACKAGE_MAX )
			{
				log_err( "Data package too long %d", data_recv_state.index + len );
				data_recv_state.can_id = 0;
			}
			else
			{
				memcpy( &data_recv_state.data[data_recv_state.index], pframe->data, len );
				data_recv_state.index += len;
			}
		}

		if ( ( can_id & CAN_ID_PACK_LAST ) == CAN_ID_PACK_LAST )
		{
			rc = CAN_CMD_BAD_ACK;
			if ( data_recv_state.can_id != 0 )
			{
				if ( settings.verbose == 0 )
				{
					log_info( "Data package receive %d bytes, send to UDS_ORCHESTRATOR", data_recv_state.index );
				}
				if ( uds_send_orchestrator(
					UDS_ORCHESTRATOR_DATA | ( data_recv_state.can_id & CAN_ID_CODE_MASK ),
					data_recv_state.data,
					data_recv_state.index ) == 0 )
					rc = CAN_CMD_ACK;
			}
			else
			{
				log_err( "Incorrect final data package" );
				return;
			}

			data_recv_state.can_id = 0;

			if ( ( can_id & CAN_ID_PACK_LAST_ACK ) == CAN_ID_PACK_LAST_ACK )
			{
				can_id = rc
					| ( ( can_id & CAN_ID_RECIEVER_MASK ) << ( CAN_ID_SENDER_POS - CAN_ID_RECIEVER_POS ) )
					| ( ( can_id & CAN_ID_SENDER_MASK ) >> ( CAN_ID_SENDER_POS - CAN_ID_RECIEVER_POS ) )
					| ( can_id & ( CAN_COUNT_ACK | CAN_ID_CODE_MASK ) );
				can_out( fd, can_id, NULL, 0, false, true, true );
			}
		}
		return;
	}

	if ( sender == APISOS_SENDER_ID )
	{
		if ( ( can_id & ( CAN_ID_IS_DATA | CAN_ID_CODE_MASK |
			CAN_ID_ATTRIBUTE_MASK ) ) == CMD_APISOS )
		{
			if ( uds_send_fine_guidance( CMD_APISOS ) < 0 )
				can_out_apisos( fd, can_id, NULL );
			else
			{
				fine_guidance_state.can_fd = fd;
				fine_guidance_state.can_id = can_id;
				can_oneshot_timer_start( );
			}
		}
		else
			log_err( "Unknown command from APISOS %04X",
				(uint16_t) ( can_id & ( CAN_ID_CODE_MASK | CAN_ID_ATTRIBUTE_MASK ) ) );

		return;
	}

	if ( sender != CENTER_SENDER_ID )
		return;

	// Send command to UDS_ORCHESTRATOR
	if ( ( can_id & CAN_CMD_WITH_DATA ) != 0 )
	{
		if ( len <= 0 )
		{
			log_err( "Cammand 0x0%08X with parameters but without data", can_id );
			len = 0;
		}
		else
			memcpy( buffer, pframe->data, len );
	}

	rc = uds_send_orchestrator( UDS_ORCHESTRATOR_CMD | ( can_id & CAN_ID_CODE_MASK ), buffer, len );

	if ( ( can_id & CAN_CMD_NEED_ACK ) == CAN_CMD_NEED_ACK )
	{
		can_id =
			( ( rc == 0 ) ? CAN_CMD_ACK : CAN_CMD_BAD_ACK )
			| ( ( can_id & CAN_ID_RECIEVER_MASK ) << ( CAN_ID_SENDER_POS - CAN_ID_RECIEVER_POS ) )
			| ( ( can_id & CAN_ID_SENDER_MASK ) >> ( CAN_ID_SENDER_POS - CAN_ID_RECIEVER_POS ) )
			| ( can_id & CAN_ID_CODE_MASK );
		can_out( fd, can_id, NULL, 0, false, true, true );
	}
}
#pragma endregion

#pragma region Close CAN and periodic timer sockets
/*
 * @brief Close CAN and periodic timer sockets
 */
void can_close( void )
{
	if ( cans.oneshot_timer_fd >= 0 )
	{
		can_oneshot_timer_stop( );
		close( cans.oneshot_timer_fd );
		cans.oneshot_timer_fd = -1;
	}

	if ( cans.periodic_timer_fd >= 0 )
	{
		close( cans.periodic_timer_fd );
		cans.periodic_timer_fd = -1;
	}

	can_queue_clear( &can_queue_high );
	can_queue_clear( &can_queue_low );

	if ( cans.thread_can != (pthread_t) ( -1 ) )
	{
		int sem_value;
		if ( sem_getvalue( &cans.sem_terminate, &sem_value ) == 0 )
		{
			if ( sem_value == 0 )
			{
				sem_post( &cans.sem_terminate );
				can_queue_put( &can_queue_high, NULL );
				pthread_join( cans.thread_can, NULL );
			}
		}
	}

	if ( cans.can0.can_fd >= 0 )
	{
		close( cans.can0.can_fd );
		cans.can0.can_fd = -1;
	}

	if ( cans.can1.can_fd >= 0 )
	{
		close( cans.can1.can_fd );
		cans.can1.can_fd = -1;
	}
}
#pragma endregion

#pragma region Create oneshot timer
static int oneshot_timer_create( int epoll_fd )
{
	cans.oneshot_timer_fd = -1;

	int timer_fd = timerfd_create( CLOCK_MONOTONIC, TFD_NONBLOCK );
	if ( timer_fd < 0 )
	{
		log_errno( "Can't create one-shot timer." );
		return -1;
	}

	bzero( &cans.oneshot_timer_stop, sizeof( cans.oneshot_timer_stop ) );
	bzero( &cans.oneshot_timer_start, sizeof( cans.oneshot_timer_start ) );
	cans.oneshot_timer_start.it_value.tv_nsec =
		(__syscall_slong_t) settings.oneshot_timer_msec * 1000000ULL;

	struct epoll_event event_setup;
	event_setup.data.fd = timer_fd;
	event_setup.events = EPOLLIN | EPOLLET;
	if ( epoll_ctl( epoll_fd, EPOLL_CTL_ADD, timer_fd, &event_setup ) < 0 )
	{
		log_errno( "Can't add one-short timer to poll." );
		close( timer_fd );
		return -1;
	}
	log_info( "One-short timer initialization complete." );
	cans.oneshot_timer_fd = timer_fd;

	return 0;
}
#pragma endregion

#pragma region Create periodic timer
/*
 * @brief Initialize periodic timer for regular telemetry
 */
static int periodic_timer_create( int epoll_fd )
{
	cans.periodic_timer_fd = -1;

	int timer_fd = timerfd_create( CLOCK_MONOTONIC, TFD_NONBLOCK );
	if ( timer_fd < 0 )
	{
		log_errno( "Can't create periodic timer." );
		return -1;
	}

	struct itimerspec newTime;
	bzero( &newTime, sizeof( newTime ) );

	struct timespec ts = { .tv_sec = PT_KEEPALIVE_INTERVAL_SEC, .tv_nsec = 0 };
	newTime.it_value = ts;
	newTime.it_interval = ts;
	if ( timerfd_settime( timer_fd, 0, &newTime, NULL ) < 0 )
	{
		log_errno( "Can't set periodic timer." );
		close( timer_fd );
		return -1;
	}

	struct epoll_event event_setup;
	event_setup.data.fd = timer_fd;
	event_setup.events = EPOLLIN | EPOLLET;
	if ( epoll_ctl( epoll_fd, EPOLL_CTL_ADD, timer_fd, &event_setup ) < 0 )
	{
		log_errno( "Can't add periodic timer to poll." );
		close( timer_fd );
		return -1;
	}
	log_info( "Periodic timer initialization complete." );
	cans.periodic_timer_fd = timer_fd;
	return 0;
}
#pragma endregion

#pragma region Create CAN socket
/*
 * @brief Create CAN socket for dev0 first and if index not found then for dev1
 * and add it to epool.
 * @return 0 on success, -1 on error.
 */
int can_create( int epoll_fd, can_data_t* can, const char* dev0,
	const char* dev1 )
{
	can->can_fd = -1;

	int can_fd = socket( PF_CAN, SOCK_RAW, CAN_RAW );
	if ( can_fd < 0 )
	{
		log_err( "Can't create socket for %s/%s." ERROR_MESSAGE, dev0, dev1, errno, strerror( errno ) );
		return -1;
	}

	if ( setsockopt( can_fd, SOL_CAN_RAW, CAN_RAW_FILTER, &can_filter, sizeof( can_filter ) ) < 0 )
	{
		log_errno( "Can't set CAN socket filter." );
		close( can_fd );
		return -1;
	}

	can_err_mask_t err_mask = (
		CAN_ERR_TX_TIMEOUT |
		CAN_ERR_LOSTARB |
		CAN_ERR_CRTL |
		CAN_ERR_PROT |
		CAN_ERR_TRX |
		CAN_ERR_ACK |
		CAN_ERR_BUSOFF |
		CAN_ERR_BUSERROR |
		CAN_ERR_RESTARTED
		);

	if ( setsockopt( can_fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof( err_mask ) ) < 0 )
	{
		log_errno( "Can't set CAN socket error mask." );
		close( can_fd );
		return -1;
	}

	struct ifreq ifr;
	bzero( &ifr.ifr_name, sizeof( ifr.ifr_name ) );
	strncpy( ifr.ifr_name, dev0, sizeof( ifr.ifr_name ) );
	if ( ioctl( can_fd, SIOCGIFINDEX, &ifr ) < 0 )
	{
		// Device index not exists, try second name
		bzero( &ifr.ifr_name, sizeof( ifr.ifr_name ) );
		strncpy( ifr.ifr_name, dev1, sizeof( ifr.ifr_name ) );
		if ( ioctl( can_fd, SIOCGIFINDEX, &ifr ) < 0 )
		{
			log_err( "Invalid CAN indexes for %s/%s." ERROR_MESSAGE, dev0, dev1, errno,
				strerror( errno ) );
			close( can_fd );
			return -1;
		}
	}

	struct sockaddr_can addr;
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	if ( bind( can_fd, (struct sockaddr*) &addr, sizeof( addr ) ) < 0 )
	{
		log_err( "Failed bind to CAN '%s'." ERROR_MESSAGE, ifr.ifr_name, errno,
			strerror( errno ) );
		close( can_fd );
		return -1;
	}

	struct epoll_event event_setup;
	event_setup.events = EPOLLIN;
	event_setup.data.fd = can_fd;
	if ( epoll_ctl( epoll_fd, EPOLL_CTL_ADD, can_fd, &event_setup ) < 0 )
	{
		log_err( "Failed to add CAN %s to epool." ERROR_MESSAGE, ifr.ifr_name, errno,
			strerror( errno ) );
		close( can_fd );
		return -1;
	}
	log_err( "CAN '%s' (%d) initialization complete.", ifr.ifr_name, can_fd );
	can->can_fd = can_fd;
	return 0;
}
#pragma endregion

#pragma region Process CAN error
void can_process_error( int fd )
{
	if ( fd == cans.can0.can_fd )
	{
		// Error on CAN 0, use CAN 1 only
		cans.mode_select = USE_CAN_1;
		cans.can0.fail_count++;
		cans.can1.fail_count = 0;
		cans.success_count = 0;
		log_err( "Switch to CAN 1 only" );
	}
	else if ( fd == cans.can1.can_fd )
	{
		// Error on CAN 1, use CAN 0 only
		cans.mode_select = USE_CAN_0;
		cans.can1.fail_count++;
		cans.can0.fail_count = 0;
		cans.success_count = 0;
		log_err( "Switch to CAN 0 only" );
	}
}
#pragma endregion

#pragma region Process CAN success
void can_process_success( int fd )
{
	if ( cans.mode_select != USE_CAN_AUTO )
	{
		cans.success_count++;
		if ( cans.success_count >= 6 )
		{
			cans.mode_select = USE_CAN_AUTO;
			cans.success_count = 0;
			log_err( "Switch to CAN AUTO" );
		}
	}
}
#pragma endregion

#pragma region CAN thread
void* can_thread( void* p )
{
	int semval = 0;
	can_job_t* job;
	const struct timespec sleepValue = { .tv_sec = 0, .tv_nsec = 5000000 };
	int rc;

	while ( semval == 0 )
	{
		job = can_queue_get_any( );
		if ( job != NULL && job->can_fd >= 0 )
		{
			if ( ( ( job->frame.can_id & CAN_ID_CODE_MASK ) == CMD_REGULAR_DATA ) ||
				( ( job->frame.can_id & CAN_ID_CODE_MASK ) == CMD_FINE_GUIDANCE ) )
			{
				// No need output
			}
			else
			{
				if ( settings.verbose == 0 )
				{
					char msg[128];
					char* m = MessageCAN( msg, "COut ", job->can_fd, &job->frame );
					if ( m != NULL )
						log_info( m );
				}
			}

			nanosleep( &sleepValue, NULL );
			int rc = write( job->can_fd, &job->frame, sizeof( struct can_frame ) );
			if ( rc != sizeof( struct can_frame ) )
			{
				can_process_error( job->can_fd );
				log_err( "CAN send error. (%d)" ERROR_MESSAGE, job->can_fd, errno, strerror( errno ) );
			}
			else
			{
				can_process_success( job->can_fd );
			}
		}
		sem_getvalue( &cans.sem_terminate, &semval );
	}
	return NULL;
}
#pragma endregion

#pragma region Initialize CAN
/*
 * @brief Initialize CAN
 * @return 0 on success, -1 on error
 */
int can_initialize( int epoll_fd )
{
	//	sem_init(&cans.semTerm, 0, 0);
	cans.can0.can_fd = -1;
	cans.can1.can_fd = -1;
	cans.periodic_timer_fd = -1;
	cans.thread_can = (pthread_t) ( -1 );

	can_create( epoll_fd, &cans.can0, "can0", "slcan0" );
	can_create( epoll_fd, &cans.can1, "can1", "slcan1" );
	if ( cans.can0.can_fd < 0 && cans.can1.can_fd < 0 )
	{
		log_err( "CAN interfaces not found." );
		can_close( );
		return -1;
	}

	if ( settings.disable_periodic_timer == 0 )
	{
		if ( periodic_timer_create( epoll_fd ) < 0 )
		{
			can_close( );
			return -1;
		}
	}
	if ( oneshot_timer_create( epoll_fd ) < 0 )
	{
		can_close( );
		return -1;
	}

	can_queue_initialize( &can_queue_high, CAN_QUEUE_HIGH_SIZE, can_jobs_high );
	can_queue_initialize( &can_queue_low, CAN_QUEUE_LOW_SIZE, can_jobs_low );

	pthread_mutex_init( &cans.queue_mutex, NULL );
	pthread_cond_init( &cans.queue_ready, NULL );

	if ( sem_init( &cans.sem_terminate, 0, 0 ) < 0 )
	{
		log_errno( "Can't create CAN semaphore." );
		can_close( );
		return -1;
	}
	if ( pthread_create( &cans.thread_can, NULL, can_thread, NULL ) < 0 )
	{
		log_errno( "Can't create CAN thread." );
		can_close( );
		return -1;
	}

	return 0;
}
#pragma endregion

#pragma region Process high priority events
int can_process_high( int fd )
{
	if ( fd == cans.oneshot_timer_fd )
	{
		uint64_t res;
		read( fd, &res, sizeof( res ) );

		if ( fine_guidance_state.can_id != 0 )
		{
			can_out_apisos( fine_guidance_state.can_fd, fine_guidance_state.can_id, NULL );
			fine_guidance_state.can_id = 0;
		}
		return 0;
	}
	return -1;
}
#pragma endregion

#pragma region Process middle priority events

int can_process_middle( int fd )
{
	if ( fd == cans.can0.can_fd || fd == cans.can1.can_fd )
	{
		struct can_frame frame;

		int nbytes = (int) read( fd, &frame, sizeof( frame ) );
		if ( nbytes < 0 )
		{
			can_process_error( fd );
			return 0;
		}
		if ( nbytes != (int) CAN_MTU )
		{
			log_err( "Incomplete CAN frame %d on (%d)", nbytes, fd );
			return 0;
		}

		canid_t can_id = frame.can_id;
		if ( ( can_id & CAN_ID_CODE_MASK ) == CMD_APISOS )
		{
			// No need output info for API-SOS requests
		}
		else
		{
			if ( settings.verbose == 0 )
			{
				char msg[128];
				char* m = MessageCAN( msg, "CIn ", fd, &frame );
				if ( m != NULL )
					log_info( m );
			}
		}

		if ( ( can_id & CAN_ERR_FLAG ) != 0 )
		{
			can_process_error( fd );
			log_err( "Frame with CAN_ERR_FLAG flag ID:0x%04X data:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
				can_id,
				frame.data[0], frame.data[1], frame.data[2], frame.data[3],
				frame.data[4], frame.data[5], frame.data[6], frame.data[7] );
		}
		else if ( ( can_id & CAN_EFF_FLAG ) == 0 )
		{
			log_err( "Frame without CAN_EFF_FLAG flag ID:0x%04X", can_id );
		}
		else if ( ( can_id & CAN_RTR_FLAG ) != 0 )
		{
			log_err( "Frame with CAN_RTR_FLAG flag  ID:0x%04X", can_id );
		}
		else
		{
			can_in( fd, &frame );
			can_process_success( fd );
		}
		return 0;
	}
	return -1;
}
#pragma endregion

#pragma region Process low priority events
int can_process_low( int fd )
{
	if ( fd == cans.periodic_timer_fd )
	{
		uint64_t res;
		read( fd, &res, sizeof( res ) );

		uint8_t* data = uds_regular_telemetry_lock( );
		can_out( -1, CAN_ID_REGULAR_TELEMETRY, data, TELEMETRY_REGULAR_SIZE, true, false, false );
		uds_regular_telemetry_unlock( );

		return 0;
	}
	return -1;
}
#pragma endregion

#pragma region Send TELEMETRY
void can_send_log_telemetry( uint8_t* data, int nbytes )
{
	can_out( -1, CAN_ID_LOG_TELEMETRY, data, nbytes, true, true, false );
}
void can_send_target_payload( uint8_t* data, int nbytes )
{
	can_out( -1, CAN_ID_TARGET_PAYLOAD, data, nbytes, true, true, false );
}

void can_send_main_telemetry( uint8_t* data, int nbytes )
{
	can_out( -1, CAN_ID_MAIN_TELEMETRY, data, nbytes, true, true, false );
}

void can_send_extra_telemetry( uint8_t* data, int nbytes )
{
	can_out( -1, CAN_ID_EXTRA_TELEMETRY, data, nbytes, true, true, false );
}

void can_send_timestamp_0A( canid_t can_id, uint8_t* data, int nbytes )
{
	can_out( -1, CAN_ID_TIMESTAMP_0A, data, nbytes, false, false, true );
}

void can_send_timestamp_0B( canid_t can_id, uint8_t* data, int nbytes )
{
	can_out( -1, CAN_ID_TIMESTAMP_0B, data, nbytes, false, false, true );
}

void can_send_fine_guidance( uint8_t* uds_data, int nbytes )
{
	if ( fine_guidance_state.can_id != 0 )
	{
		can_out_apisos( fine_guidance_state.can_fd, fine_guidance_state.can_id,
			( nbytes == FINE_GUIDANCE_SIZE ) ? uds_data : NULL );
		fine_guidance_state.can_id = 0;
	}
}
#pragma endregion