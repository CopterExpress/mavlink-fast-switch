#include <syslog.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <stdint.h>
#include <sys/select.h>
#include <assert.h>

#include "endpoint.h"

int filter_len(const uint32_t * const filter)
{
  int i = 0;
  while(filter[i] != CSC_FILTER_TERMINATION)
    i++;

  return i;
}

int filter_id(const uint32_t * const filter, const uint32_t id)
{
  int i = 0;
  while (filter[i] != CSC_FILTER_TERMINATION)
  {
    if (filter[i] == id)
      return i;

    i++;
  }

  return -1;
}

int ep_open_udp(const p_endpoint_t endpoint, const char * const name, const char * const local_ip, const uint16_t local_port,
  const char * const remote_ip, const uint16_t remote_port, const float sleep_interval, const float sleep_heartbeat_interval,
  filter_type_t filter_type, const uint32_t * const filter)
{
  syslog(LOG_INFO,
    "Opening MAVLink UDP endpoint: name \"%s\", local %s:%u, remote %s:%u, sleep interval %f, heartbeat interval %f, "
    "filter %s, filter size %i",
    name, (local_ip) ? local_ip : "ANY", local_port, (remote_ip) ? remote_ip : "UNKNOWN", remote_port, sleep_interval,
    sleep_heartbeat_interval, (filter_type == FT_DROP) ? "DROP" : "ACCEPT", (filter) ? filter_len(filter) : 0);

  // Copy the endpoint name
  strncpy(endpoint->name, name, CSC_ENDPOINT_NAME_MAX + 1);
  if (endpoint->name[CSC_ENDPOINT_NAME_MAX] != '\0')
  {
    syslog(LOG_ERR, "Endpoint name is too long");

    return -1;
  }

  // Create an UDP socket
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
    if (!inet_aton(local_ip, &addr.sin_addr))
    {
      syslog(LOG_ERR, "Invalid remote IP address: \"%s\"", local_ip);

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
  // If endpoint sleep interval is set, the endpoint starts sleeping
  endpoint->sleep_mode = sleep_interval > 0;

  endpoint->sleep_heartbeat_interval = sleep_heartbeat_interval;
  // 
  endpoint->first_heartbeat = true;

  endpoint->filter_type = filter_type;

  // Filter is set
  if (filter)
  {
    int i = 0;

    // Copy it message by message
    while(filter[i] != CSC_FILTER_TERMINATION)
    {
      if (i >= CSC_FILTER_LEN_MAX)
        return -1;

      endpoint->filter[i] = filter[i];

      i++;
    };

    // Terminate a filter
    endpoint->filter[i] = CSC_FILTER_TERMINATION;
  }
  else
    // Empty filter
    endpoint->filter[0] = CSC_FILTER_TERMINATION;
  

  // Reset a remote address for the endpoint
  memset(&endpoint->remote_address, 0, sizeof(endpoint->remote_address));

  // A remote address is presented
  if (remote_ip)
  {
    // IP
    endpoint->remote_address.sin_family = AF_INET;
    // Convert IP from the string to the IP address
    if (!inet_aton(remote_ip, &addr.sin_addr))
    {
      syslog(LOG_ERR, "Invalid remote IP address: \"%s\"", remote_ip);

      close(endpoint->fd);

      return -1;
    }

    // Convert port to the network byte order
    endpoint->remote_address.sin_port = htons(remote_port);

    endpoint->fix_remote = true;
  }
  else
    endpoint->remote_address.sin_addr.s_addr = INADDR_NONE;  // No remote IP address presented

  // Check if the remote address is a multicast address
  if (IN_MULTICAST(ntohl(endpoint->remote_address.sin_addr.s_addr)))
  {
    syslog(LOG_INFO, "Remote address %s is a multicast address", remote_ip);

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

int ec_open_endpoint(const p_endpoints_collection_t collection, const char * const name, const char * const local_ip,
  const uint16_t local_port, const char * const remote_ip, const uint16_t remote_port, const float sleep_interval,
  const float sleep_heartbeat_interval, const filter_type_t filter_type, const uint32_t * const filter)
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

  int index = ep_open_udp(&collection->endpoints[i], name, local_ip, local_port, remote_ip, remote_port, sleep_interval,
    sleep_heartbeat_interval, filter_type, filter);

  // Error opening a new endpoint
  if (index < 0)
    return ECCE_OPEN_FAILED;

  // Mark the memory block as used
  collection->used_set[i] = true;

  return index;
}

int ec_len(const p_endpoints_collection_t collection)
{
  int len = 0;

  // Count all used memory blocks
  for (int i = 0; i < MAVLINK_COMM_NUM_BUFFERS; i++)
    if (collection->used_set[i])
      len++;

  return len;
}

int ec_stamp_all(const p_endpoints_collection_t collection)
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

void ec_close_all(const p_endpoints_collection_t collection)
{
  int i;

  // Close endpoints in the used memory blocks
  for (i = 0; i < MAVLINK_COMM_NUM_BUFFERS; i++)
    if (collection->used_set[i])
      ec_close_endpoint(collection, i);
}

int ec_fd_max(const p_endpoints_collection_t collection)
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

void ec_get_fds(const p_endpoints_collection_t collection, fd_set * const read_fds)
{
  FD_ZERO(read_fds);

  // Build select file descriptors set from the file descriptors in the used memory blocks
  for (int i = 0; i < MAVLINK_COMM_NUM_BUFFERS; i++)
    if (collection->used_set[i])
      FD_SET(collection->endpoints[i].fd, read_fds);
}

int ec_sendto(const p_endpoints_collection_t collection, const int sender_index, const mavlink_message_t * const message)
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
    if (collection->used_set[i] && (i != sender_index)
      && (collection->endpoints[i].remote_address.sin_addr.s_addr != INADDR_NONE))
    {
      static struct timespec current_time;

      if (clock_gettime(CLOCK_MONOTONIC, &current_time) < 0)
      {
        syslog(LOG_ERR, "Failed to get current clock value: \"%s\"", strerror(errno));
        
        return ECSR_TIME_ERROR;
      }

      // Sleep mode is enabled
      if (collection->endpoints[i].sleep_interval > 0)
      {
        // Endpoint is not sleeping and there is no activity on the endpoint for enought time
        if ((!collection->endpoints[i].sleep_mode) &&
          (timespec_passed(&current_time, &collection->endpoints[i].last_activity) > collection->endpoints[i].sleep_interval))
        {
          syslog(LOG_INFO, "Endpoint \"%s\" (#%i) is now sleeping", collection->endpoints[i].name, i);

          // Sleep mode is now sleeping
          collection->endpoints[i].sleep_mode = true;
        }

        // Endpoint is sleeping
        if (collection->endpoints[i].sleep_mode)
        {
          // Incoming message is HEARTBEAT
          if (message->msgid == MAVLINK_MSG_ID_HEARTBEAT)
          {
            if ((collection->endpoints[i].sleep_heartbeat_interval > 0) && (!collection->endpoints[i].first_heartbeat)
              && (timespec_passed(&current_time, &collection->endpoints[i].last_heartbeat) <=
              collection->endpoints[i].sleep_heartbeat_interval))
            {
              syslog(LOG_DEBUG, "Endpoint %s HEARTBEAT drop due to sleep heartbeat interval", collection->endpoints[i].name);

              // Do not send an incoming message to this endpoint
              continue;
            }
          }
          // Incoming message is not HEARTBEAT
          else
            // Do not send an incoming message to this endpoint
            continue;
        }
      }

      // Incoming message is HEARTBEAT
      if (message->msgid == MAVLINK_MSG_ID_HEARTBEAT)
      {
        // Update timestamp
        collection->endpoints[i].last_heartbeat = current_time;

        collection->endpoints[i].first_heartbeat = false;
      }

      static int msgid_index;

      // Find a message ID in the filter
      msgid_index = filter_id(collection->endpoints[i].filter, message->msgid);

      // If it is an ACCEPT filter and the message is not found, if it is a DROP filter and the message is found
      if (((collection->endpoints[i].filter_type == FT_ACCEPT) && (msgid_index < 0)) ||
          ((collection->endpoints[i].filter_type == FT_DROP) && (msgid_index >= 0)))
        // Do not send an incoming message to this endpoint
        continue;

      // Send the data to the remote address
      if (sendto(collection->endpoints[i].fd, message_buf, message_length, 0,
        (struct sockaddr *)&collection->endpoints[i].remote_address, sizeof(struct sockaddr_in)) < 0)
      {
        syslog(LOG_ERR, "Failed to send data to endpoint %s: \"%s\"\n", collection->endpoints[i].name, strerror(errno));

        return ECSR_SEND_FAILED;
      }
    }
  }

  return ECSR_OK;
}

int ec_select(const p_endpoints_collection_t collection, const int fd_max, const sigset_t * const orig_mask)
{
  // select fds number
  static int select_fds_num;

  static int endpoint_index;
  static int i;

  static socklen_t addr_len;

  static int send_all_result;

  // Read fd set for select
  static fd_set read_fds;

  // Data read buffer
  static uint8_t read_buf[READ_BUF_SIZE];
  // Read data counter
  static ssize_t data_read;

  // MAVLink message buffer
  static mavlink_message_t message;
  // MAVLink message parsing status
  static mavlink_status_t status;

  // Get FD set mask for the collection
  ec_get_fds(collection, &read_fds);

  // Wait for data at any fd and process SIGINT and SIGTERM
  select_fds_num = pselect(fd_max + 1, &read_fds, NULL, NULL, NULL, orig_mask);

  // select returned an error
  if (select_fds_num < 0)
  {
    // Ignore signal interrupt
    if (errno != EINTR)
      syslog(LOG_ERR, "select call failed: %s", strerror(errno));
    return ECEC_SELECT_FAILED;
  }

  // For every memory block in the collection
  for (endpoint_index = 0; endpoint_index < MAVLINK_COMM_NUM_BUFFERS; endpoint_index++)
  {
    // If the memory block is used check if it is the source of the data
    if (collection->used_set[endpoint_index] && FD_ISSET(collection->endpoints[endpoint_index].fd, &read_fds))
    {
      if (collection->endpoints[endpoint_index].fix_remote)
        // Read the data from the broadcast endpoint (no sender address is needed)
        data_read = read(collection->endpoints[endpoint_index].fd, &read_buf, sizeof(read_buf));
      else  // We need to retrieve the sender name for the non-brodcast endpoint
      {
        // Set the address length
        addr_len = sizeof(struct sockaddr_in);

        // Retrieve the data and the sender information, save the sender information in the endpoint
        data_read = recvfrom(collection->endpoints[endpoint_index].fd, &read_buf, sizeof(read_buf), 0,
                              (struct sockaddr *)&(collection->endpoints[endpoint_index].remote_address), &addr_len);

        // It's a strange situation
        assert(addr_len == sizeof(struct sockaddr_in));
      }

      // For every byte in the new data
      for (i = 0; i < data_read; i++)
      {
        // Parse using MAVLink
        if (mavlink_parse_char(endpoint_index, read_buf[i], &message, &status))
        {
          if (collection->endpoints[endpoint_index].sleep_mode)
          {
            syslog(LOG_INFO, "Endpoint \"%s\" is now awake", collection->endpoints[endpoint_index].name);
            
            collection->endpoints[endpoint_index].sleep_mode = false;
          }

          // Send the data to every endpoint except the sender
          send_all_result = ec_sendto(collection, endpoint_index, &message);

          if ((send_all_result < 0) && (send_all_result != ECSR_SEND_FAILED))
            return ECEC_SEND_FAILED;

          if (ep_stamp(&collection->endpoints[endpoint_index]) < 0)
          {
            syslog(LOG_ERR, "Failed to stamp monotonic clock value to the endpoint!");

            return ECEC_STAMP_FAILED;
          }
        }
      }
    }
  }

  return ECEC_OK;
}
