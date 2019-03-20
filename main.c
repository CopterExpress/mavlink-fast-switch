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
#include <assert.h>
#include <sysexits.h>

#include "mavlink_dialect.h"

#include "endpoint.h"
#include "config.h"

// unistd optarg externals for arguments parsing
extern char *optarg;
extern int optind, opterr, optopt;
//

// Volatile flag to stop application (from signal)
static volatile sig_atomic_t stop_application = false;

// Linux signal handler (for SIGINT and SIGTERM)
void signal_handler(int signum)
{
  // Set application stop flag
  stop_application = true;
}

int main(int argc, char **argv)
{
  printf("COEX MAVLink fast switch v0.1\n");

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

    return EX_OSERR;
  }

  // Current command line option
  int option;

  // Log verbosity level command line argument value
  char *log_level = NULL;

  // For every command line argument
  while ((option = getopt(argc, argv, "l:h")) != -1)
    switch (option)
    {
    // Log verbosity level
    case 'l':
      log_level = optarg;
      break;
    case 'h':
    // Help request
    case '?':
      puts("\nUsage:\n\tmavlink-fast-switch [-l <log level>] <configuration file>\n\nOptions:\n\t"
      "-l - log verbosity level (debug, info, warn)");
      return EX_USAGE;
      break;
    default:
      return EX_USAGE;
    }

  // Config file path command line argument value
  char *config_path = NULL;

  // Configuration file path (last position argument)
  if (optind < argc)
    config_path = argv[optind];
  else
  {
    printf("\nConfiguration file path is not set!\n");
    return EX_USAGE;
  }

  int log_level_up;

  // Default log level
  if (!log_level)
    log_level_up = LOG_INFO;
  // Process the user requested log level
  else if (!strcmp(log_level, "debug"))
    log_level_up = LOG_DEBUG;
  else if (!strcmp(log_level, "info"))
    log_level_up = LOG_INFO;
  else if (!strcmp(log_level, "warn"))
    log_level_up = LOG_WARNING;
  else
  {
    printf("\nUnknown log verbosity level: %s!\n", log_level);

    return EX_USAGE;
  }

  puts("");

  setlogmask(LOG_UPTO(log_level_up));

  openlog("charging-station-comm", LOG_CONS | LOG_PID | LOG_NDELAY, LOG_USER);
  
  syslog(LOG_DEBUG, "Debug mode enabled");

  endpoints_collection_t collection;

  // Init the collection
  ec_init(&collection);

  if (config_load(&collection, config_path) < 0)
  {
    closelog();

    return EX_CONFIG;
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

    return EX_OSERR;
  }

  // WARNING: No SIGINT and SIGTERM from this point

  // Get the maximal file descriptor number from all endpoints
  int fd_max = ec_fd_max(&collection);
  assert(fd_max >= 0);

  while (!stop_application)
  {
    if (ec_select(&collection, fd_max, &orig_mask) < 0)
      break;
  }

  syslog(LOG_DEBUG, "Stopping application...");

  ec_close_all(&collection);
  closelog();

  return EX_OK;
}
