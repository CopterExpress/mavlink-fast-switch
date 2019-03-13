#include <errno.h>
#include <stdint.h>
#include <stdarg.h>
#include <assert.h>
#include <syslog.h>
#include <stddef.h>

#include <cyaml/cyaml.h>

// WARNING: Make sure the correct dialect is included
#include "mavlink_dialect.h"

#include "config.h"

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
    CYAML_FIELD_ENUM("type", CYAML_FLAG_DEFAULT, struct config_filter, type, config_filter_type_strings,
        CYAML_ARRAY_LEN(config_filter_type_strings)),
    // MAVLink message name pointers array >= 1, <= CSC_FILTER_MESSAGES_MAX
    CYAML_FIELD_SEQUENCE("messages", CYAML_FLAG_POINTER, struct config_filter, messages, &string_ptr_schema,
        1, CSC_FILTER_MESSAGES_MAX),
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

static int open_endpoint_from_config(p_endpoints_collection_t collection, struct config_endpoint *endpoint,
    const char *name, uint16_t local_port)
{
    const char *local_ip;

    if (endpoint->local)
    {
        local_ip = endpoint->local->ip;

        if (!local_port)
            local_port = (endpoint->local->port) ? *endpoint->local->port : 0;
    }
    else
        local_ip = NULL;

    const char *remote_ip;
    uint16_t remote_port;

    if (endpoint->remote)
    {
        remote_ip = endpoint->remote->ip;
        remote_port = endpoint->remote->port;
    }
    else
    {
        remote_ip = NULL;
        remote_port = 0;
    }

    float sleep_interval;
    float heartbeat_interval;

    if (endpoint->sleep)
    {
        sleep_interval = endpoint->sleep->interval;
        heartbeat_interval = (endpoint->sleep->heartbeat_interval) ? *endpoint->sleep->heartbeat_interval : -1;
    }   
    else
    {
        sleep_interval = -1;
        heartbeat_interval = -1;
    }
    
    filter_type_t filter_type;
    uint32_t filter[FILTER_MAX_LEN];

    if (endpoint->filter)
    {
        filter_type = endpoint->filter->type;

        const mavlink_message_info_t *message_info;

        int message_index;
        for (message_index = 0; message_index < endpoint->filter->messages_count; message_index++)
        {
            message_info = mavlink_get_message_info_by_name(endpoint->filter->messages[message_index]);

            if (!message_info)
            {
                syslog(LOG_ERR, "Unknown message: \"%s\"!", endpoint->filter->messages[message_index]);

                return -1;
            }

            filter[message_index] = message_info->msgid;

            syslog(LOG_DEBUG, "Message ID: \"%s\" -> %u", endpoint->filter->messages[message_index], filter[message_index]);
        }
        
        filter[message_index] = FILTER_TERMINATION;
    }
    else
    {
        filter_type = FT_DROP; 
        filter[0] = FILTER_TERMINATION;
    }

    if (!name)
        name = endpoint->name;

    return ec_open_endpoint(collection, name, local_ip, local_port, remote_ip, remote_port, sleep_interval,
        heartbeat_interval, filter_type, filter);
}

// libcyaml configuration
static const cyaml_config_t lib_config = 
{
    .log_level = CYAML_LOG_INFO,  // Maximum verbosity level (syslog handles the real output)
    .log_fn = cyaml_log_syslog,  // Log using syslog
    .mem_fn = cyaml_mem, // Use the default memory allocator
};

int config_load(p_endpoints_collection_t collection, const char *config_file_path)
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

    syslog(LOG_DEBUG, "Endpoints found: %u", app_config->endpoints_count);

    if (app_config->endpoints_count < 1)
    {
        syslog(LOG_ERR, "Endpoints number has to be more than 1!");

        cyaml_free(&lib_config, &config_schema, app_config, 0);

        return -1;
    }

    for (int endpoint_index = 0; endpoint_index < app_config->endpoints_count; endpoint_index++)
    {
        syslog(LOG_DEBUG, "Entering endpoint: %u", endpoint_index);

        if (app_config->endpoints[endpoint_index].local && app_config->endpoints[endpoint_index].local->port &&
            app_config->endpoints[endpoint_index].local->ports)
        {
            syslog(LOG_ERR, "Endpoint can't have both local port and local ports interval!");

            cyaml_free(&lib_config, &config_schema, app_config, 0);

            return -1;
        }

        if (app_config->endpoints[endpoint_index].sleep && ((app_config->endpoints[endpoint_index].sleep->interval <= 0) ||
            ((app_config->endpoints[endpoint_index].sleep->heartbeat_interval) &&
            (*app_config->endpoints[endpoint_index].sleep->heartbeat_interval <= 0))))
        {
            syslog(LOG_ERR, "Endpoint sleep intervals must be positive numbers!");

            cyaml_free(&lib_config, &config_schema, app_config, 0);

            return -1;
        }

        if (app_config->endpoints[endpoint_index].local && app_config->endpoints[endpoint_index].local->ports)
        {
            syslog(LOG_INFO, "Multiport endpoint: %u", endpoint_index);

            char multiport_name[ENDPOINT_NAME_MAX + 1];
            int result;

            for (uint16_t local_port = app_config->endpoints[endpoint_index].local->ports->start;
                local_port <= app_config->endpoints[endpoint_index].local->ports->end;
                local_port++)
            {
                syslog(LOG_INFO, "Processing port: %u", local_port);

                result = snprintf(multiport_name, ENDPOINT_NAME_MAX, "%s@%u", app_config->endpoints[endpoint_index].name,
                    local_port);

                assert(result > 0);

                if (open_endpoint_from_config(collection, app_config->endpoints + endpoint_index, multiport_name, local_port) < 0)
                {
                    syslog(LOG_ERR, "Failed to open endpoint!");

                    cyaml_free(&lib_config, &config_schema, app_config, 0);

                    return -1;
                }
            }
        }
        else
        {
            if (open_endpoint_from_config(collection, app_config->endpoints + endpoint_index, NULL, 0) < 0)
            {
                syslog(LOG_ERR, "Failed to open endpoint!");

                cyaml_free(&lib_config, &config_schema, app_config, 0);

                return -1;
            }
        }
    }

    // Free the data
    cyaml_free(&lib_config, &config_schema, app_config, 0);

    return 0;
}