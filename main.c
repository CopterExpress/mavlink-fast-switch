#include <stdio.h>
#include <syslog.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <stdbool.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <time.h>
#include <sys/select.h>
#include <assert.h>

// WARNING: Make sure the correct dialect is included
#include "libs/mavlink/v2.0/ardupilotmega/mavlink.h"

typedef enum
{
  SUCCESS = 0,
  ARGS_INV,             // Invalid command line arguments
  ENDPOINT_FAILED,      // Failed to setup ENDPOINT
  SIG_SETUP_FAILED,     // Failed to configure signals
  FC_SERIAL_DISC = 10,  // FC serial port was disconnected (USB UART?)
  MAV_INV_LEN           // MAVLink message is too big
} cscExitCode;

// unistd optarg externals for arguments parsing
extern char *optarg;
extern int optind, opterr, optopt;
//

// Application read buffer size
// HINT: Mast be equal to the maximum UDP packet size
#define READ_BUF_SIZE MAVLINK_MAX_PACKET_LEN

// Volatile flag to stop application (from signal)
static volatile sig_atomic_t stop_application = false;

// Linux signal handler (for SIGINT and SIGTERM)
void signal_handler(int signum)
{
  // Set application stop flag
  stop_application = true;
}

typedef enum
{
  FT_DROP = 0,
  FT_ACCEPT
} filter_type_t, *p_filter_type_t;

#define FILTER_MAX_LEN  (20 + 1)
#define FILTER_TERMINATION  UINT32_MAX

static int filter_len(uint32_t *filter)
{
  int i = 0;
  while(filter[i] != FILTER_TERMINATION)
    i++;

  return i;
}

static int filter_id(uint32_t *filter, uint32_t id)
{
  int i = 0;
  
  while (filter[i] != FILTER_TERMINATION)
  {
    if (filter[i] == id)
      return i;

    i++;
  }

  return -1;
}

typedef struct
{
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

// Config file path command line argument option value buffer size
#define CONFIG_FILE_ARGUMENT_BUF_SIZE 50

// Log verbosity level command line argument option value buffer size
#define LOG_LEVEL_ARGUMENT_BUF_SIZE 16

// Config file path
static char config_path[CONFIG_FILE_ARGUMENT_BUF_SIZE] = {
    '\0',
};

// Log level verbosity (string)
static char log_level[LOG_LEVEL_ARGUMENT_BUF_SIZE] = {
    'i', 'n', 'f', 'o', '\0'
};

static int ep_open_udp(p_endpoint_t endpoint, const char *local_ip, const uint16_t local_port,
                                   const char *remote_ip, const uint16_t remote_port, const float sleep_interval,
                                   const float sleep_heartbeat_interval, filter_type_t filter_type, 
                                   uint32_t *filter)
{
  syslog(LOG_INFO,
    "Opening MAVLink UDP endpoint: local %s:%u, remote %s:%u, sleep %f, sleep heartbeat %f, filter %s, filter size %i",
    (local_ip) ? local_ip : "ANY", local_port, (remote_ip) ? remote_ip : "UNKNOWN", remote_port, sleep_interval,
    sleep_heartbeat_interval, (filter_type == FT_DROP) ? "DROP" : "ACCEPT", (filter) ? filter_len(filter) : 0);

  // Create UDP socket
  endpoint->fd = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);

  if (endpoint->fd < 0)
  {
    syslog(LOG_ERR, "Failed to create a new socket: \"%s\"", strerror(errno));

    return -1;
  }

  struct sockaddr_in addr;

  // Setting the local address
  memset(&addr, 0, sizeof(addr));

  // IP
  addr.sin_family = AF_INET;

  // Local IP is povided
  if (local_ip)
  {
    // Convert string to the IP address
    addr.sin_addr.s_addr = inet_addr(local_ip);

    // Check if IP address conversion was successfull
    if (addr.sin_addr.s_addr == INADDR_NONE)
    {
      syslog(LOG_ERR, "Invalid local IP address: \"%s\"", local_ip);

      close(endpoint->fd);

      return -1;
    }
  }
  else
    addr.sin_addr.s_addr = INADDR_ANY; // Bind on all interfaces

  // Convert port to the network byte order
  addr.sin_port = htons(local_port);

  // Bind UDP socket to the local address
  if (bind(endpoint->fd, (struct sockaddr *)&addr, sizeof(addr)) == -1)
  {
    syslog(LOG_ERR, "Failed to bind the network address: \"%s\"", strerror(errno));

    close(endpoint->fd);

    return -1;
  }

  // Set UDP socket to the non-blocking mode (to work with select)
  if (fcntl(endpoint->fd, F_SETFL, O_NONBLOCK | O_ASYNC) < 0)
  {
    syslog(LOG_ERR, "Failed to set non-blocking mode on the socket: \"%s\"", strerror(errno));

    close(endpoint->fd);

    return -1;
  }

  endpoint->sleep_interval = sleep_interval;
  endpoint->sleep_mode = sleep_interval > 0;

  endpoint->sleep_heartbeat_interval = sleep_heartbeat_interval;
  endpoint->first_heartbeat = true;

  endpoint->filter_type = filter_type;

  if (filter)
  {
    int i;

    while(filter[i] != FILTER_TERMINATION)
    {
      if (i >= FILTER_MAX_LEN)
        return -1;

      endpoint->filter[i] = filter[i];

      i++;
    };

    endpoint->filter[i] = FILTER_TERMINATION;
  }
  else
    endpoint->filter[0] = FILTER_TERMINATION;
  

  // Reset a remote address for the endpoint
  memset(&endpoint->remote_address, 0, sizeof(endpoint->remote_address));

  // A remote address is presented
  if (remote_ip)
  {
    // IP
    endpoint->remote_address.sin_family = AF_INET;
    // Convert IP from the string to the IP address
    endpoint->remote_address.sin_addr.s_addr = inet_addr(remote_ip);

    // Check if IP address conversion was successfull
    if (endpoint->remote_address.sin_addr.s_addr == INADDR_NONE)
    {
      syslog(LOG_ERR, "Invalid remote IP address: \"%s\"", remote_ip);

      close(endpoint->fd);

      return -1;
    }

    // Convert port to the network byte order
    endpoint->remote_address.sin_port = htons(remote_port);
  }
  else
    endpoint->remote_address.sin_addr.s_addr = INADDR_NONE;  // No remote IP address presented

  // Check if the remote address is a multicast address
  if (IN_MULTICAST(ntohl(endpoint->remote_address.sin_addr.s_addr)))
  {
    syslog(LOG_INFO, "Remote address %s is a multicast address!", remote_ip);

    endpoint->broadcast = true;

    // Const true value for a SO_BROADCAST option
    int so_broadcast = true;

    // Enable broadcasting for the socket
    if (setsockopt(endpoint->fd, SOL_SOCKET, SO_BROADCAST, &so_broadcast, sizeof(so_broadcast)) < 0)
    {
      syslog(LOG_ERR, "Failed to set broadcast mode on the socket: \"%s\"", strerror(errno));

      close(endpoint->fd);

      return -1;
    }
  }

  return 0;
}

static inline void ep_close_udp(p_endpoint_t endpoint)
{
  close(endpoint->fd);
}

static int ep_stamp(p_endpoint_t endpoint)
{
  if (clock_gettime(CLOCK_MONOTONIC, &endpoint->last_activity) < 0)
  {
    syslog(LOG_ERR, "Failed to get clock value: \"%s\"", strerror(errno));
    
    return -1;
  }

  return 0;
}

static inline float timespec2float(struct timespec *time)
{
  return time->tv_sec + time->tv_nsec / 1000000000.0;
}

float timespec_passed(struct timespec *a, struct timespec *b)
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

static int ec_open_endpoint(p_endpoints_collection_t collection, const char *local_ip, const uint16_t local_port,
  const char *remote_ip, const uint16_t remote_port, float sleep_interval, const float sleep_heartbeat_interval,
  filter_type_t filter_type, uint32_t *filter)
{
  int i;

  // Look for the first empty memory block
  for (i = 0; i < MAVLINK_COMM_NUM_BUFFERS; i++)
    if (!collection->used_set[i])
      break;

  if (i == MAVLINK_COMM_NUM_BUFFERS)
  {
    syslog(LOG_ERR, "No free memory blocks in the endpoints collection!");

    return ECCE_FULL;
  }

  syslog(LOG_DEBUG, "Found free memory block: %i", i);

  int handle = ep_open_udp(&collection->endpoints[i], local_ip, local_port, remote_ip, remote_port, sleep_interval,
    sleep_heartbeat_interval, filter_type, filter);

  // Error opening a new endpoint
  if (handle < 0)
    return ECCE_OPEN_FAILED;

  // Mark the memory block as used
  collection->used_set[i] = true;

  return handle;
}

static inline void ec_close_endpoint(p_endpoints_collection_t collection, int endpoint_handle)
{
  ep_close_udp(&collection->endpoints[endpoint_handle]);

  // Mark the memory block as unused
  collection->used_set[endpoint_handle] = false;
}

static int ec_len(p_endpoints_collection_t collection)
{
  int len = 0;

  // Count all used memory blocks
  for (int i = 0; i < MAVLINK_COMM_NUM_BUFFERS; i++)
    if (collection->used_set[i])
      len++;

  return len;
}

static int ec_stamp_all(p_endpoints_collection_t collection)
{
  int i;

  // Stamp monotonic clock value for endpoints in the used memory blocks
  for (i = 0; i < MAVLINK_COMM_NUM_BUFFERS; i++)
    if (collection->used_set[i])
    {
      if (ep_stamp(&collection->endpoints[i]) < 0)
        return -1;
    }

  return 0;
}

static void ec_close_all(p_endpoints_collection_t collection)
{
  int i;

  // Close endpoints in the used memory blocks
  for (i = 0; i < MAVLINK_COMM_NUM_BUFFERS; i++)
    if (collection->used_set[i])
      ec_close_endpoint(collection, i);
}

static int ec_fd_max(p_endpoints_collection_t collection)
{
  int fd = -1;

  // Find file descriptor with the max number in the used memory blocks
  for (int i = 0; i < MAVLINK_COMM_NUM_BUFFERS; i++)
    if (collection->used_set[i] && (fd < collection->endpoints[i].fd))
      fd = collection->endpoints[i].fd;

  if (fd < 0)
    syslog(LOG_DEBUG, "No used memory blocks in the endpoints collection!");

  return fd;
}

static void ec_get_fds(p_endpoints_collection_t collection, fd_set *read_fds)
{
  FD_ZERO(read_fds);

  // Build select file descriptors set from the file descriptors in the used memory blocks
  for (int i = 0; i < MAVLINK_COMM_NUM_BUFFERS; i++)
    if (collection->used_set[i])
      FD_SET(collection->endpoints[i].fd, read_fds);
}

typedef enum
{
  ECSR_INV_LEN = -1,
  ECSR_SEND_FAILED = -2
} ec_sendto_result_t;

static int ec_sendto(p_endpoints_collection_t collection, const int sender_handle, const mavlink_message_t *message)
{
  // Send buffer (to store parsed MAVLink message)
  static uint8_t message_buf[MAVLINK_MAX_PACKET_LEN];
  // MAVLink message length
  static unsigned int message_length;

  // Convert message to the binary format
  message_length = mavlink_msg_to_send_buffer(message_buf, message);

  // Check if the binary message size is correct
  if (message_length > MAVLINK_MAX_PACKET_LEN)
  {
    syslog(LOG_CRIT, "MAVLink message is bigger than buffer size!");

    return ECSR_INV_LEN;
  }

  for (int i = 0; i < MAVLINK_COMM_NUM_BUFFERS; i++)
  {
    // For every endpoint in the used memory block that is not a sender and has the remote address
    if (collection->used_set[i] && (i != sender_handle)
      && (collection->endpoints[i].remote_address.sin_addr.s_addr != INADDR_NONE))
    {
      static struct timespec current_time;

      if (clock_gettime(CLOCK_MONOTONIC, &current_time) < 0)
      {
        syslog(LOG_ERR, "Failed to get current clock value: \"%s\"", strerror(errno));
        
        return -3;
      }

      if (collection->endpoints[i].sleep_interval > 0)
      {
        if ((!collection->endpoints[i].sleep_mode) &&
          (timespec_passed(&current_time, &collection->endpoints[i].last_activity) > collection->endpoints[i].sleep_interval))
        {
          syslog(LOG_INFO, "Endpoint %i is now sleeping", i);

          collection->endpoints[i].sleep_mode = true;
        }

        if (collection->endpoints[i].sleep_mode)
        {
          if (message->msgid == MAVLINK_MSG_ID_HEARTBEAT)
          {
            if ((collection->endpoints[i].sleep_heartbeat_interval > 0) && (!collection->endpoints[i].first_heartbeat)
              && (timespec_passed(&current_time, &collection->endpoints[i].last_heartbeat) <=
              collection->endpoints[i].sleep_heartbeat_interval))
            {
              syslog(LOG_DEBUG, "Endpoint %i HEARTBEAT drop due to sleep heartbeat interval", i);

              continue;
            }
          }
          else
            continue;
        }
      }

      if (message->msgid == MAVLINK_MSG_ID_HEARTBEAT)
      {
        collection->endpoints[i].last_heartbeat = current_time;

        collection->endpoints[i].first_heartbeat = false;
      }

      static int msgid_index;

      msgid_index = filter_id(collection->endpoints[i].filter, message->msgid);

      if (((collection->endpoints[i].filter_type == FT_ACCEPT) && (msgid_index < 0)) ||
          ((collection->endpoints[i].filter_type == FT_DROP) && (msgid_index >= 0)))
        continue;

      // Send the data to the remote address
      if (sendto(collection->endpoints[i].fd, message_buf, message_length, 0,
        (struct sockaddr *)&collection->endpoints[i].remote_address, sizeof(struct sockaddr_in)) < 0)
      {
        syslog(LOG_ERR, "Failed to send data to endpoint %i: \"%s\"\n", i, strerror(errno));

        return ECSR_SEND_FAILED;
      }
    }
  }

  return 0;
}

int main(int argc, char **argv)
{
  printf("COEX charging station comminucation module v0.1\n");

  // Signal action structure
  struct sigaction act;

  // Reset it and set a handler
  memset(&act, 0, sizeof(act));
  act.sa_handler = signal_handler;

  // Bind SIGINT and SIGTERM to the application signal handler
  if ((sigaction(SIGTERM, &act, 0) < 0) ||
      (sigaction(SIGINT, &act, 0) < 0))
  {
    printf("\nError setting signal handler: %s\n", strerror(errno));

    return SIG_SETUP_FAILED;
  }

  int option;

  // For every command line argument
  while ((option = getopt(argc, argv, "c:l:")) != -1)
    switch (option)
    {
    // Configuration file path
    case 'c':
      // Copy path to the variable
      strncpy(config_path, optarg, sizeof(config_path));

      // The command line argument value is too long if there is no \0 at the end
      if (config_path[sizeof(config_path) - 1])
      {
        printf("\nConfig path is too long!\n");
        return ARGS_INV;
      }
      break;
    // Log verbosity level
    case 'l':
      // Copy IP adress to the variable
      strncpy(log_level, optarg, sizeof(log_level));

      // The command line argument value is too long if there is no \0 at the end
      if (log_level[sizeof(log_level) - 1])
      {
        printf("\nLog level is too long!\n");
        return ARGS_INV;
      }
      break;
    // Help request
    case '?':
      puts("\nOptions:\n\t-c - configuration file;\n\t-l - log verbosity level (debug, info, warn)");
      return ARGS_INV;
      break;
    default:
      return ARGS_INV;
    }

  // Check if serial port path was enetered
  if (!config_path[0])
  {
    printf("\nConfiguration file path is not set!\n");
    return ARGS_INV;
  }

  int log_level_up;

  // Check if GCS IP address was enetered
  if (!strcmp(log_level, "debug"))
    log_level_up = LOG_DEBUG;
  else if (!strcmp(log_level, "info"))
    log_level_up = LOG_INFO;
  else if (!strcmp(log_level, "warn"))
    log_level_up = LOG_WARNING;
  else
  {
    printf("\nUnknown log verbosity level: %s!\n", log_level);

    return ARGS_INV;
  }

  puts("");

  setlogmask(LOG_UPTO(log_level_up));

  openlog("charging-station-comm", LOG_CONS | LOG_PID | LOG_NDELAY | LOG_PERROR, LOG_USER);
  
  syslog(LOG_DEBUG, "Debug mode enabled");

  endpoints_collection_t collection;

  // Init the collection
  ec_init(&collection);

  if (ec_open_endpoint(&collection, "127.0.0.1", 14588, NULL, 0, 3.0, 1.0, FT_DROP, NULL) < 0)
  {
    syslog(LOG_ERR, "Failed to create endpoint 1!");

    closelog();

    return ENDPOINT_FAILED;
  }

  uint32_t endpoint2_filters[] = { MAVLINK_MSG_ID_SYS_STATUS, FILTER_TERMINATION };

  if (ec_open_endpoint(&collection, "127.0.0.1", 14589, NULL, 0, 3.0, 1.0, FT_DROP, endpoint2_filters) < 0)
  {
    syslog(LOG_ERR, "Failed to create endpoint 2!");

    ec_close_all(&collection);
    closelog ();

    return ENDPOINT_FAILED;
  }

  // Signals to block
  sigset_t mask;
  // Clear the mask
  sigemptyset(&mask);
  // Set signals to ignore
  sigaddset(&mask, SIGTERM);
  sigaddset(&mask, SIGINT);

  // Original signal parameters
  sigset_t orig_mask;
  // Block signals according to mask and save previous mask
  if (sigprocmask(SIG_BLOCK, &mask, &orig_mask) < 0)
  {
    syslog(LOG_ERR, "Error setting new signal mask: \"%s\"", strerror(errno));

    ec_close_all(&collection);
    closelog ();

    return SIG_SETUP_FAILED;
  }

  // WARNING: No SIGINT and SIGTERM from this point

  // Data read buffer
  uint8_t read_buf[READ_BUF_SIZE];
  // Read data counter
  ssize_t data_read;

  // MAVLink message buffer
  mavlink_message_t message;
  // MAVLink message parsing status
  mavlink_status_t status;

  // Read fd set for select
  fd_set read_fds;

  // Get the maximal file descriptor number from all endpoints
  int fd_max = ec_fd_max(&collection);
  assert(fd_max >= 0);

  // select fds number
  int select_fds_num;

  int endpoint_index;
  int i;

  socklen_t addr_len;

  int send_all_result;

  while (!stop_application)
  {
    // Get FD set mask for the collection
    ec_get_fds(&collection, &read_fds);

    // Wait for data at any fd and process SIGINT and SIGTERM
    select_fds_num = pselect(fd_max + 1, &read_fds, NULL, NULL, NULL, &orig_mask);

    // select returned an error
    if (select_fds_num < 0)
    {
      // Ignore signal interrupt
      if (errno != EINTR)
        syslog(LOG_ERR, "EPOLL wait failed: %s", strerror(errno));
      continue;
    }

    // For every memory block in the collection
    for (endpoint_index = 0; endpoint_index < MAVLINK_COMM_NUM_BUFFERS; endpoint_index++)
    {
      // If the memory block is used check if it is the source of the data
      if (collection.used_set[endpoint_index] && FD_ISSET(collection.endpoints[endpoint_index].fd, &read_fds))
      {
        if (collection.endpoints[endpoint_index].broadcast)
          // Read the data from the broadcast endpoint (no sender address is needed)
          data_read = read(collection.endpoints[endpoint_index].fd, &read_buf, sizeof(read_buf));
        else  // We need to retrieve the sender name for the non-brodcast endpoint
        {
          // Set the address length
          addr_len = sizeof(struct sockaddr_in);

          // Retrieve the data and the sender information, save the sender information in the endpoint
          data_read = recvfrom(collection.endpoints[endpoint_index].fd, &read_buf, sizeof(read_buf), 0,
                               (struct sockaddr *)&(collection.endpoints[endpoint_index].remote_address), &addr_len);

          // It's a strange situation
          if (addr_len != sizeof(struct sockaddr_in))
          {
            collection.endpoints[endpoint_index].remote_address.sin_addr.s_addr = INADDR_NONE;

            syslog(LOG_CRIT, "Invalid address size: %u!", addr_len);
          }
        }

        // For every byte in the new data
        for (i = 0; i < data_read; i++)
        {
          // Parse using MAVLink
          if (mavlink_parse_char(endpoint_index, read_buf[i], &message, &status))
          {
            if (collection.endpoints[endpoint_index].sleep_mode)
            {
              syslog(LOG_INFO, "Endpoint %i is now awake", endpoint_index);
              
              collection.endpoints[endpoint_index].sleep_mode = false;
            }

            // Send the data to every endpoint except the sender
            send_all_result = ec_sendto(&collection, endpoint_index, &message);

            if ((send_all_result < 0) && (send_all_result != ECSR_SEND_FAILED))
            {
              ec_close_all(&collection);
              closelog ();

              return 10;
            }

            if (ep_stamp(&collection.endpoints[endpoint_index]) < 0)
            {
              syslog(LOG_ERR, "Failed to stamp monotonic clock value to the endpoint!");

              ec_close_all(&collection);
              closelog ();

              return 10;
            }
          }
        }
      }
    }
  }

  syslog(LOG_DEBUG, "Stopping application...");

  ec_close_all(&collection);
  closelog();

  return SUCCESS;
}
