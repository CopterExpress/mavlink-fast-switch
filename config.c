#include <errno.h>
#include <stdint.h>
#include <stdarg.h>
#include <assert.h>
#include <syslog.h>
#include <stddef.h>

#include <cyaml/cyaml.h>

#include "config.h"
#include "mavlink_dialect.h"

// Filter type dictionary
static const cyaml_strval_t config_filter_type_strings[] = 
{
	{ "drop", FT_DROP },
	{ "accept", FT_ACCEPT }
};

// Filter configuration
struct config_filter
{
    // Filter type
    enum filter_type type;
    // MAVLink message name pointers array
    const char **messages;
    // MAVLink message name pointers size
    unsigned int messages_count;
};

// MAVLink message name pointers array schema (array of pointers to srings)
static const cyaml_schema_value_t string_ptr_schema =
{
    // MAVLink message name size >= 1, <= CSC_FILTER_MESSAGE_NAME_MAX
	CYAML_VALUE_STRING(CYAML_FLAG_POINTER, char, 1, CSC_FILTER_MESSAGE_NAME_MAX),
};

// Filter fields schema
static const cyaml_schema_field_t config_filter_fields_schema[] =
{
    // Filter type
    CYAML_FIELD_ENUM("type", CYAML_FLAG_CASE_INSENSITIVE, struct config_filter, type, config_filter_type_strings,
        CYAML_ARRAY_LEN(config_filter_type_strings)),
    // MAVLink message name pointers array >= 1, <= CSC_FILTER_MESSAGES_MAX
    CYAML_FIELD_SEQUENCE("messages", CYAML_FLAG_POINTER, struct config_filter, messages, &string_ptr_schema,
        1, CSC_FILTER_LEN_MAX),
    CYAML_FIELD_END
};

// Endpoint local ports interval
struct config_local_ports_interval
{
    // Start port
    uint16_t start;
    // End port
    uint16_t end;
};

// Endpoint local ports interval fields schema
static const cyaml_schema_field_t config_local_ports_interval_fields_schema[] =
{
    // Start port
    CYAML_FIELD_UINT("start", CYAML_FLAG_DEFAULT, struct config_local_ports_interval, start),
    // End port
    CYAML_FIELD_UINT("end", CYAML_FLAG_DEFAULT, struct config_local_ports_interval, end),
    CYAML_FIELD_END
};

// Endpoint sleep settings
struct config_endpoint_sleep
{
    // Sleep interval (s)
    float interval;
    // Minimal hertbeat interval in sleep mode (optional, no limit by default) (s)
    float *heartbeat_interval;
};

// Endpoint sleep settings fields schema
static const cyaml_schema_field_t config_endpoint_sleep_fields_schema[] =
{
    // Start port
    CYAML_FIELD_FLOAT("interval", CYAML_FLAG_DEFAULT, struct config_endpoint_sleep, interval),
    // End port
    CYAML_FIELD_FLOAT_PTR("heartbeat-interval", CYAML_FLAG_POINTER | CYAML_FLAG_OPTIONAL, struct config_endpoint_sleep,
        heartbeat_interval),
    CYAML_FIELD_END
};

// Endpoint local address
struct config_local_address
{
    // Endpoint local IP (optional, 0.0.0.0 by default)
    char *ip;
    // Endpoint local port (optional, 0 by default)
    uint16_t *port;
    // Endpoint local ports interval (optional, mutually exclusive with local port)
    struct config_local_ports_interval *ports;
};

// Endpoint local address fields schema
static const cyaml_schema_field_t config_local_address_fields_schema[] =
{
    // Endpoint local IP >= 1, <= CSC_ENDPOINT_IP_MAX_LEN
    CYAML_FIELD_STRING_PTR("ip", CYAML_FLAG_POINTER | CYAML_FLAG_OPTIONAL, struct config_local_address, ip, 1,
        CSC_ENDPOINT_IP_MAX_LEN),
    // Endpoint local port (uint16_t)
    CYAML_FIELD_UINT_PTR("port", CYAML_FLAG_POINTER | CYAML_FLAG_OPTIONAL, struct config_local_address, port),
    // Endpoint local ports interval
    CYAML_FIELD_MAPPING_PTR("ports", CYAML_FLAG_POINTER | CYAML_FLAG_OPTIONAL, struct config_local_address, ports,
                        config_local_ports_interval_fields_schema),
    CYAML_FIELD_END
};

// Endpoint remote address
struct config_remote_address
{
    // Endpoint local IP
    char ip[CSC_ENDPOINT_IP_MAX_LEN + 1];
    // Endpoint local port
    uint16_t port;
};

// Endpoint remote address fields schema
static const cyaml_schema_field_t config_remote_address_fields_schema[] =
{
    // Endpoint local IP >= 1, <= CSC_ENDPOINT_IP_MAX_LEN
    CYAML_FIELD_STRING("ip", CYAML_FLAG_DEFAULT, struct config_remote_address, ip, 1),
    // Endpoint local port (uint16_t)
    CYAML_FIELD_UINT("port", CYAML_FLAG_DEFAULT, struct config_remote_address, port),
    CYAML_FIELD_END
};

// Broadcast type dictionary
static const cyaml_strval_t broadcast_type_strings[] = 
{
	{ "disabled", BT_DISABLED },
	{ "normal", BT_NORMAL },
    { "discovery", BT_DISCOVERY }
};

// Endpoint
struct config_endpoint
{
    // Endpoint name
    const char name[CSC_ENDPOINT_NAME_USER_MAX + 1];
    // Endpoint local address (optional, 0.0.0.0:0 by default)
    const struct config_local_address *local;
    // Endpoint remote address (optional, will be discovered by the incoming packed by default)
    const struct config_remote_address *remote; 
    // Endpoint config filter (optional, no filter by default)
    struct config_filter *filter;
    // Endpoint sleep settings (optional, sleep disabled by default)
    struct config_endpoint_sleep *sleep;
    // Endpoint broadcast type (optional, BT_DISABLED by default)
    enum broadcast_type *broadcast_type;
};

// Endpoint fields schema
static const cyaml_schema_field_t config_endpoint_fields_schema[] =
{
    // Endpoint name >= 1, <= CSC_ENDPOINT_NAME_MAX
    CYAML_FIELD_STRING("name", CYAML_FLAG_DEFAULT, struct config_endpoint, name, 1),
    // Endpoint filter
    CYAML_FIELD_MAPPING_PTR("local", CYAML_FLAG_POINTER | CYAML_FLAG_OPTIONAL, struct config_endpoint, local,
                        config_local_address_fields_schema),
    // Endpoint local adddress
    CYAML_FIELD_MAPPING_PTR("remote", CYAML_FLAG_POINTER | CYAML_FLAG_OPTIONAL, struct config_endpoint, remote,
                        config_remote_address_fields_schema),
    // Endpoint remote address
    CYAML_FIELD_MAPPING_PTR("filter", CYAML_FLAG_POINTER | CYAML_FLAG_OPTIONAL, struct config_endpoint, filter,
                        config_filter_fields_schema),
    // Endpoint sleep settings
    CYAML_FIELD_MAPPING_PTR("sleep", CYAML_FLAG_POINTER | CYAML_FLAG_OPTIONAL, struct config_endpoint, sleep,
                        config_endpoint_sleep_fields_schema),
    // Endpoint broadcast type
    CYAML_FIELD_ENUM_PTR("broadcast", CYAML_FLAG_POINTER | CYAML_FLAG_OPTIONAL | CYAML_FLAG_CASE_INSENSITIVE,
        struct config_endpoint, broadcast_type, broadcast_type_strings, CYAML_ARRAY_LEN(broadcast_type_strings)),
    CYAML_FIELD_END
};  

// Endpoint schema to use it as a sequence item
static const cyaml_schema_value_t config_endpoint_schema =
{
    CYAML_VALUE_MAPPING(CYAML_FLAG_DEFAULT, struct config_endpoint, config_endpoint_fields_schema),
};

// Configuration
struct config
{
    // Endpoints array
    struct config_endpoint *endpoints;
    // Endpoints array size
    unsigned endpoints_count;
};

// Configuration fields schema
static const cyaml_schema_field_t config_fields_schema[] =
{
    // Endpoints >= 1, <= MAVLINK_COMM_NUM_BUFFERS
    CYAML_FIELD_SEQUENCE("endpoints", CYAML_FLAG_POINTER, struct config, endpoints, &config_endpoint_schema, 1,
        MAVLINK_COMM_NUM_BUFFERS),
    CYAML_FIELD_END
};

// Configuration schema to use it as a root entry
static const cyaml_schema_value_t config_schema =
{
    CYAML_VALUE_MAPPING(CYAML_FLAG_POINTER, struct config, config_fields_schema),
};

// syslog-based log function
static void cyaml_log_syslog(cyaml_log_t level, const char *fmt, va_list args)
{
    int priority;

    // Convert libcyaml level to syslog priority
    switch(level)
    {
        case CYAML_LOG_DEBUG:
            priority = LOG_DEBUG;
            break;
        case CYAML_LOG_INFO:
            priority = LOG_INFO;
            break;
        case CYAML_LOG_NOTICE:
            priority = LOG_NOTICE;
            break;
        case CYAML_LOG_WARNING:
            priority = LOG_WARNING;
            break;
        case CYAML_LOG_ERROR:
            priority = LOG_ERR;
            break;
        // Set INFO to unknown levels
        default:
            priority = LOG_INFO;
            break;
    }

    // Call va_list-based syslog function
    vsyslog(priority, fmt, args);
}

/*
Open new endpoit from the endpoint configuration and add to the collection.

HINT: All integrity checks must be completed before the call.

Arguments:
    collection - an endpoint collection;
    endpoint - an endpoint collection;
    name - an endpoint name to override (NULL - do not override the endpoint name);
    local_port - a local port to override (-1 - do not override the endpoint name).

Return:
    >= 0 - success,
    < 0 - error (doesn't set errno).
*/
static int open_endpoint_from_config(const p_endpoints_collection_t collection, const struct config_endpoint *endpoint,
    const char *name, int local_port)
{
    // No endpoint name to override
    if (!name)
        name = endpoint->name;

    const char *local_ip;

    // Local endpoint configuration is set
    if (endpoint->local)
    {
        // The actual string or NULL (all interfaces)
        local_ip = endpoint->local->ip;

        // The local port override is not set
        if (local_port < 0)
            // The actual local port or 0 (any free port) if not set
            local_port = (endpoint->local->port) ? *endpoint->local->port : 0;
    }
    // Local endpoint configuration is not set
    else
    {
        // All interfaces
        local_ip = NULL;

        // The local port override is not set
        if (local_port < 0)
            // Use any free port
            local_port = 0;
    }

    const char *remote_ip;
    uint16_t remote_port;

    // Remote endpoint configuration is set
    if (endpoint->remote)
    {
        // Both are defined according to the YAML schema
        remote_ip = endpoint->remote->ip;
        remote_port = endpoint->remote->port;
    }
    // Remote endpoint configuration is not set
    else
    {
        // Determine by the incoming packets
        remote_ip = NULL;
        remote_port = 0;
    }

    float sleep_interval;
    float heartbeat_interval;

    // Endpoint sleep configuration is set
    if (endpoint->sleep)
    {
        sleep_interval = endpoint->sleep->interval;
        // Get the endpoint heartbeat interval if set, disable if doesn't
        heartbeat_interval = (endpoint->sleep->heartbeat_interval) ? *endpoint->sleep->heartbeat_interval : -1;
    }  
    // Endpoint sleep configuration is not set 
    else
    {
        // Sleep mode disabled
        sleep_interval = -1;
        heartbeat_interval = -1;
    }
    
    filter_type_t filter_type;
    uint32_t filter[CSC_FILTER_LEN_MAX + 1];

    // Endpoint filter configuration is set
    if (endpoint->filter)
    {
        filter_type = endpoint->filter->type;

        const mavlink_message_info_t *message_info;

        int message_index;
        // For every string MAVLink message ID
        for (message_index = 0; message_index < endpoint->filter->messages_count; message_index++)
        {
            // Find the MAVLink message information
            message_info = mavlink_get_message_info_by_name(endpoint->filter->messages[message_index]);

            if (!message_info)
            {
                syslog(LOG_ERR, "Unknown MAVLink message: \"%s\"!", endpoint->filter->messages[message_index]);

                return -1;
            }

            // Save the message ID
            filter[message_index] = message_info->msgid;

            syslog(LOG_DEBUG, "Message ID: \"%s\" -> %u", endpoint->filter->messages[message_index], filter[message_index]);
        }
        
        // Terminate the filter
        filter[message_index] = CSC_FILTER_TERMINATION;
    }
    // Endpoint filter configuration is not set
    else
    {
        filter_type = FT_DROP;
        // Empty filter
        filter[0] = CSC_FILTER_TERMINATION;
    }

    // Extract broadcast type
    broadcast_type_t broadcast_type = (endpoint->broadcast_type) ? *endpoint->broadcast_type : BT_DISABLED;

    // Open a new endpoint
    return ec_open_endpoint(collection, name, local_ip, local_port, remote_ip, remote_port, sleep_interval,
        heartbeat_interval, filter_type, filter, broadcast_type);
}

// libcyaml configuration
static const cyaml_config_t lib_config = 
{
    .log_level = CYAML_LOG_INFO,  // libcyaml verbosity level (syslog handles the real output)
    .log_fn = cyaml_log_syslog,  // Log using syslog
    .mem_fn = cyaml_mem, // Use the default memory allocator
};

int config_load(const p_endpoints_collection_t collection, const char *config_file_path)
{
    // Root configuration struct
    struct config *app_config;

    // YAML configuration parsing result
    cyaml_err_t result;

    // Parse a YAML configuration file
    result = cyaml_load_file(config_file_path, &lib_config, &config_schema, (cyaml_data_t **)&app_config, NULL);
    if (result != CYAML_OK)
    {
        syslog(LOG_ERR, "Configuration error: \"%s\"!", cyaml_strerror(result));

        return -1;
    }

    syslog(LOG_DEBUG, "Endpoints in configuration: %u", app_config->endpoints_count);

    if (app_config->endpoints_count <= 1)
    {
        syslog(LOG_ERR, "Configuration mast have more than 1 endpoint!");

        // Free libcyaml resources
        cyaml_free(&lib_config, &config_schema, app_config, 0);

        return -1;
    }

    // For every endpoint configuration
    for (int endpoint_index = 0; endpoint_index < app_config->endpoints_count; endpoint_index++)
    {
        syslog(LOG_DEBUG, "Entering endpoint: \"%s\" (#%u)", app_config->endpoints[endpoint_index].name, endpoint_index);

        // User can't set a local port for the multiport endpoint
        if (app_config->endpoints[endpoint_index].local && app_config->endpoints[endpoint_index].local->port &&
            app_config->endpoints[endpoint_index].local->ports)
        {
            syslog(LOG_ERR, "Multiport endpoint can't have a preset local port!");

            // Free libcyaml resources
            cyaml_free(&lib_config, &config_schema, app_config, 0);

            return -1;
        }

        // User can't set a negative sleep intervals
        if (app_config->endpoints[endpoint_index].sleep && ((app_config->endpoints[endpoint_index].sleep->interval <= 0) ||
            ((app_config->endpoints[endpoint_index].sleep->heartbeat_interval) &&
            (*app_config->endpoints[endpoint_index].sleep->heartbeat_interval <= 0))))
        {
            syslog(LOG_ERR, "Endpoint sleep intervals must be positive numbers!");

            // Free libcyaml resources
            cyaml_free(&lib_config, &config_schema, app_config, 0);

            return -1;
        }

        // It is a multiport endpoint
        if (app_config->endpoints[endpoint_index].local && app_config->endpoints[endpoint_index].local->ports)
        {
            syslog(LOG_INFO, "Multiport endpoint: \"%s\" (#%u)", app_config->endpoints[endpoint_index].name, endpoint_index);

            // Start port > end port
            if (app_config->endpoints[endpoint_index].local->ports->start >
                app_config->endpoints[endpoint_index].local->ports->end)
            {
                syslog(LOG_ERR, "Multiport endpoint start port is greater tahn end port!");

                // Free libcyaml resources
                cyaml_free(&lib_config, &config_schema, app_config, 0);

                return -1;
            }

            // Start port == end port
            if (app_config->endpoints[endpoint_index].local->ports->start ==
                app_config->endpoints[endpoint_index].local->ports->end)
            {
                syslog(LOG_ERR, "Multiport endpoint with one port! Use single port endpoint declaration instead.");

                // Free libcyaml resources
                cyaml_free(&lib_config, &config_schema, app_config, 0);

                return -1;
            }

            // Buffer has extra space for a multiport endpoint postfix (@<local_port>)
            char multiport_name[CSC_ENDPOINT_NAME_MAX + 1];

            // For every port in the multiport endpoint port interval
            for (uint16_t local_port = app_config->endpoints[endpoint_index].local->ports->start;
                local_port <= app_config->endpoints[endpoint_index].local->ports->end;
                local_port++)
            {
                syslog(LOG_INFO, "Processing port: %u", local_port);

                // Generate a name for the multiport endpoint instance
                result = snprintf(multiport_name, sizeof(multiport_name), "%s@%u", app_config->endpoints[endpoint_index].name,
                    local_port);

                assert(result > 0);

                // Open a new endpoint and override the name and the local port
                if (open_endpoint_from_config(collection, app_config->endpoints + endpoint_index, multiport_name, local_port) < 0)
                {
                    syslog(LOG_ERR, "Failed to open endpoint!");

                    // Free libcyaml resources
                    cyaml_free(&lib_config, &config_schema, app_config, 0);

                    return -1;
                }
            }
        }
        // It is a single port endpoint
        else
        {
            if (open_endpoint_from_config(collection, app_config->endpoints + endpoint_index, NULL, -1) < 0)
            {
                syslog(LOG_ERR, "Failed to open endpoint!");

                // Free libcyaml resources
                cyaml_free(&lib_config, &config_schema, app_config, 0);

                return -1;
            }
        }
    }

    // Free libcyaml resources
    cyaml_free(&lib_config, &config_schema, app_config, 0);

    return 0;
}