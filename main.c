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

#include "mavlink_dialect.h"

#include "endpoint.h"
#include "config.h"

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

  if (config_load(&collection, config_path) < 0)
  {
    closelog ();

    return SIG_SETUP_FAILED;
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
              syslog(LOG_INFO, "Endpoint %s is now awake", collection.endpoints[endpoint_index].name);
              
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
