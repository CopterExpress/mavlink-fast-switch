#ifndef CSC_ENDPOINT
#define CSC_ENDPOINT

#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <arpa/inet.h>

#include "mavlink_dialect.h"

typedef enum filter_type
{
  FT_DROP = 0,
  FT_ACCEPT
} filter_type_t, *p_filter_type_t;

#define FILTER_MAX_LEN  (20 + 1)
#define FILTER_TERMINATION  UINT32_MAX
#define ENDPOINT_NAME_MAX 25

int filter_len(uint32_t *filter);

int filter_id(uint32_t *filter, uint32_t id);

typedef struct
{
  char name[ENDPOINT_NAME_MAX + 1];
  // UDP socket file descriptor
  int fd;
  // Endpoint sleep interval (-1 - no sleep mode)
  float sleep_interval;
  // Sleep mode flag
  bool sleep_mode;
  // The last activity timestamp
  struct timespec last_activity;
  // The last heartbeat timestamp
  struct timespec last_heartbeat;
  // Heartbeat minimal interval (-1 - no minimal interval)
  float sleep_heartbeat_interval;
  // Firt heartbeat flag
  bool first_heartbeat;
  // Broadcast enabled flag
  bool broadcast;
  // The remote target address
  struct sockaddr_in remote_address;
  filter_type_t filter_type;
  uint32_t filter[FILTER_MAX_LEN];
} endpoint_t, *p_endpoint_t;

int ep_open_udp(p_endpoint_t endpoint, const char *name, const char *local_ip, const uint16_t local_port,
                                   const char *remote_ip, const uint16_t remote_port, const float sleep_interval,
                                   const float sleep_heartbeat_interval, filter_type_t filter_type, 
                                   uint32_t *filter);

static inline void ep_close_udp(p_endpoint_t endpoint)
{
  close(endpoint->fd);
}

int ep_stamp(p_endpoint_t endpoint);

static inline float timespec2float(struct timespec *time)
{
  return time->tv_sec + time->tv_nsec / 1000000000.0;
}

static float inline timespec_passed(struct timespec *a, struct timespec *b)
{
  return timespec2float(a) - timespec2float(b);
}

typedef struct
{
  // Used endpoints array
  bool used_set[MAVLINK_COMM_NUM_BUFFERS];
  // Endpoints array
  endpoint_t endpoints[MAVLINK_COMM_NUM_BUFFERS];
} endpoints_collection_t, *p_endpoints_collection_t;

static inline void ec_init(p_endpoints_collection_t collection)
{
  // Reset endpoints collection buffer
  memset(collection, 0, sizeof(endpoints_collection_t));
}

typedef enum
{
  ECCE_FULL = -1,
  ECCE_OPEN_FAILED = -2
} ec_create_endpoint_value_t;

int ec_open_endpoint(p_endpoints_collection_t collection, const char *name, const char *local_ip, const uint16_t local_port,
  const char *remote_ip, const uint16_t remote_port, float sleep_interval, const float sleep_heartbeat_interval,
  filter_type_t filter_type, uint32_t *filter);

static inline void ec_close_endpoint(p_endpoints_collection_t collection, int endpoint_handle)
{
  ep_close_udp(&collection->endpoints[endpoint_handle]);

  // Mark the memory block as unused
  collection->used_set[endpoint_handle] = false;
}

int ec_len(p_endpoints_collection_t collection);

int ec_stamp_all(p_endpoints_collection_t collection);

void ec_close_all(p_endpoints_collection_t collection);

int ec_fd_max(p_endpoints_collection_t collection);

void ec_get_fds(p_endpoints_collection_t collection, fd_set *read_fds);

typedef enum
{
  ECSR_INV_LEN = -1,
  ECSR_SEND_FAILED = -2
} ec_sendto_result_t;

int ec_sendto(p_endpoints_collection_t collection, const int sender_handle, const mavlink_message_t *message);

#endif
