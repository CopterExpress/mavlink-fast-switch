#ifndef CSC_ENDPOINT
#define CSC_ENDPOINT

#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <arpa/inet.h>
#include <signal.h>
#include <stdlib.h>

#include "mavlink_dialect.h"

// MAVLink filter type
typedef enum filter_type
{
  FT_DROP = 0,  // Drop all messages from the list
  FT_ACCEPT  // Accept only messages from the list
} filter_type_t, *p_filter_type_t;

// Maximal number of messages in the filter (excluding CSC_FILTER_TERMINATION)
#define CSC_FILTER_LEN_MAX  20
// Maximal endpoint name length
#define CSC_ENDPOINT_NAME_MAX 25

// Application read buffer size
// HINT: Mast be equal to the maximum UDP packet size
#define READ_BUF_SIZE MAVLINK_MAX_PACKET_LEN
// Terminating MAVLink message ID in filter
#define CSC_FILTER_TERMINATION  UINT32_MAX

// MAVLink filter type
typedef enum broadcast_type
{
  BT_DISABLED = 0,  // No broadcast
  BT_NORMAL,  // Normal broadcast
  BT_DISCOVERY  // GCS discovery broadcast (disables on the first incoming message)
} broadcast_type_t, *p_broadcast_type_t;

/*
Get a filter length.

Arguments:
  filter - the filter.

Return:
  The filter length.
*/
int filter_len(const uint32_t * const filter);

/*
Find an ID in a filter.

Arguments:
  filter - th filter.

Return:
  The ID index in the filter (-1 if not found).
*/
int filter_id(const uint32_t * const filter, const uint32_t id);

// MAVLink endpoint
typedef struct
{
  char *name;
  // Endpoint sleep interval (-1 - no sleep mode)
  float sleep_interval;
  // Heartbeat minimal interval (-1 - no minimal interval)
  float sleep_heartbeat_interval;
  // Firt heartbeat flag
  bool first_heartbeat;

  // UDP socket file descriptor
  int fd;
  // Sleep mode flag
  bool sleep_mode;
  // The last activity timestamp
  struct timespec last_activity;
  // The last heartbeat timestamp
  struct timespec last_heartbeat;
  // Fix remote address (for broadcast and output endpoints)
  bool fix_remote;
  // The remote target address
  struct sockaddr_in remote_address;
  // Filter type
  filter_type_t filter_type;
  // Filter
  uint32_t *filter;
} endpoint_t, *p_endpoint_t;

/*
Open a new UDP endpoint.

Arguments:
  endpoint - an endpoint structure;
  name - an endpoint name;
  local_ip - a local IP address;
  local_port - a local port (0 to pick any unused port);
  remote_ip - a remote IP address (NULL to get it from the incoming packet);
  remote_port - a remote port (0 to pick it from the incoming packet);
  sleep_interval - an endpoint sleep interval (-1 to disable sleep mode);
  sleep_heartbeat_interval - an heartbeat messages interval in sleep mode (-1 to disable heartbeat rate limitation);
  filter_type - a filter type (FT_DROP if no filter is set);
  filter - a message filter (NULL if no filter is presented).

Return:
    >= 0 - success,
    < 0 - error (doesn't set errno).
*/
int ep_open_udp(const p_endpoint_t endpoint, const char * const name, const char * const local_ip, const uint16_t local_port,
  const char * const remote_ip, const uint16_t remote_port, const float sleep_interval, const float sleep_heartbeat_interval,
  filter_type_t filter_type, const uint32_t * const filter, broadcast_type_t broadcast_type);

/*
Free an UDP endpoint.

Arguments:
  endpoint - an endpoint to close.
*/
static inline void ep_free_udp(const p_endpoint_t endpoint)
{
  close(endpoint->fd);

  free(endpoint->name);
  endpoint->name = NULL;

  free(endpoint->filter);
  endpoint->filter = NULL;
}

/*
Update endpoint last incoming message stamp.

Arguments:
  endpoint - an endpoint to stamp.

Return:
    >= 0 - success,
    < 0 - error (doesn't set errno).
*/
static inline int ep_stamp(const p_endpoint_t endpoint)
{
  if (clock_gettime(CLOCK_MONOTONIC, &endpoint->last_activity) < 0)
  {
    syslog(LOG_ERR, "Failed to get clock value: \"%s\"", strerror(errno));
    
    return -1;
  }

  return 0;
}

/*
Convert struct timespec to a time in seconds.

Arguments:
  time - a timespec to convert.

Return:
    Time in seconds.
*/
static inline float timespec2float(const struct timespec * const time)
{
  return time->tv_sec + time->tv_nsec / 1000000000.0;
}

/*
Get the time passed between the two timestamps.

Arguments:
  a - first timestamp;
  b - second timestamp.

Return:
    Time passed between the two timestamps.
*/
static float inline timespec_passed(const struct timespec * const a, const struct timespec * const b)
{
  return timespec2float(a) - timespec2float(b);
}

// Endpoint collection
typedef struct
{
  // Endpoints array
  endpoint_t *endpoints;
  // Current collection size
  unsigned int size;
} endpoints_collection_t, *p_endpoints_collection_t;

/*
Init endpoints collection.

Arguments:
  collection - a collection to init.
*/
static inline void ec_init(const p_endpoints_collection_t collection)
{
  // Reset endpoints collection buffer
  memset(collection, 0, sizeof(endpoints_collection_t));
}

// Increase endpoints collection size result codes
typedef enum
{
  EIS_OK = 0,
  EIS_LIMIT = -1,  // No more MAVLink channels in C MAVLink
  EIS_HEAP_ERROR = -2  // Heap allocation/reallocation failed
} ec_increase_size_result_t;

/* Increase the collection size by 1 memory block.

Arguments:
  collection - a collection to increase.
*/
ec_increase_size_result_t ec_increase_size(const p_endpoints_collection_t collection);

// Open endpoints error codes
typedef enum
{
  ECCE_FULL = -1,  // No free MAVLink channels
  ECCE_OPEN_FAILED = -2,  // Failed to open a socket
  ECCE_RESIZE_ERROR = -3  // Failed to resize endpoints collection
} ec_open_endpoint_error_t;

/*
Open a new UDP endpoint in the collection.

Arguments:
  collection - an endpoint collection;
  name - an endpoint name;
  local_ip - a local IP address;
  local_port - a local port (0 to pick any unused port);
  remote_ip - a remote IP address (NULL to get it from the incoming packet);
  remote_port - a remote port (0 to pick it from the incoming packet);
  sleep_interval - an endpoint sleep interval (-1 to disable sleep mode);
  sleep_heartbeat_interval - an heartbeat messages interval in sleep mode (-1 to disable heartbeat rate limitation);
  filter_type - a filter type (FT_DROP if no filter is set);
  filter - a message filter (NULL if no filter is presented).

Return:
    >= 0 - opened endpoint index,
    < 0 - error (doesn't set errno) (ec_open_endpoint_error_t).
*/
int ec_open_endpoint(const p_endpoints_collection_t collection, const char * const name, const char * const local_ip,
  const uint16_t local_port, const char * const remote_ip, const uint16_t remote_port, const float sleep_interval,
  const float sleep_heartbeat_interval, const filter_type_t filter_type, const uint32_t * const filter,
  broadcast_type_t broadcast_type);

/*
Stamp all endpoints in the collection.

Arguments:
  collection - the collection.

Return:
  >= 0 - success,
  < 0 - error (doesn't set errno).
*/
int ec_stamp_all(const p_endpoints_collection_t collection);

/*
Free all endpoints in the collection.

Arguments:
  collection - the collection.
*/
void ec_free_all(const p_endpoints_collection_t collection);

/*
Find maximal fd for select().

Arguments:
  collection - the collection to find maximal fd in.

Return:
  >= 0 - success,
  < 0 - error (doesn't set errno).
*/
int ec_fd_max(const p_endpoints_collection_t collection);

/*
Get fds for select.

Arguments:
  collection - the collection to generate fds for.

Return:
  >= 0 - success,
  < 0 - error (doesn't set errno).
*/
void ec_get_fds(const p_endpoints_collection_t collection, fd_set * const read_fds);

typedef enum
{
  ECSR_OK = 0,
  ECSR_INV_LEN = -1,  // Invalid MAVLink message binary size
  ECSR_TIME_ERROR = -2,  // Failed to get current time
  ECSR_SEND_FAILED = -3  // Socket send error
} ec_sendto_result_t;

/*
Send MAVLink message to all endpoints.

Arguments:
  collection - the collection to send massage to;
  sender_index - a sender endpoint index (-1 if no sender in the collection);
  message -  a MAVLink message to send.

Return:
  ECSR_OK - success,
  < 0 - error (doesn't set errno) (ec_sendto_result_t).
*/
int ec_sendto(const p_endpoints_collection_t collection, const int sender_index, const mavlink_message_t * const message);

typedef enum
{
  ECEC_OK = 0,
  ECEC_SELECT_FAILED = -1,  // Failed to start select
  ECEC_SEND_FAILED = -2,  // Failed to get current time
  ECEC_STAMP_FAILED = -3  // Socket send error
} ec_process_result_t;

/*
Process MAVLink messages from all endpoints.

Arguments:
  collection - the collection to process messages in;
  fd_max - a maximum fd for select;
  orig_mask - an original signals mask to allow callbacks from select.

Return:
  ECEC_OK - success,
  < 0 - error (doesn't set errno) (ec_process_result_t).
*/
ec_process_result_t ec_select(const p_endpoints_collection_t collection, int const fd_max, const sigset_t * const orig_mask);

/*
Free the endpoint collection and all the endpoints.

Arguments:
  collection - the collection to free.
*/
void ec_free(const p_endpoints_collection_t collection);

#endif
