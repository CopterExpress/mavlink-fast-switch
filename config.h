#ifndef CSC_CONFIG
#define CSC_CONFIG

#include "endpoint.h"

// Maximal endpoint name length (excluding terminating \0, including the prefix)
#define CSC_FILTER_MESSAGE_NAME_MAX 50

// Maximal IP address length (excluding terminating \0)
#define CSC_ENDPOINT_IP_MAX_LEN (3 * 4 + 3 * 1)
// Maximal endpoint name prefix length (excluding terminating \0)
#define CSC_ENDPOINT_NAME_POSTFIX_MAX (1 + 5)
// Maximal user part of the endpoint name (part of the name user inputs in the configuration, excluding terminating \0)
#define CSC_ENDPOINT_NAME_USER_MAX (CSC_ENDPOINT_NAME_MAX - CSC_ENDPOINT_NAME_POSTFIX_MAX)

/*
Load the user configuration

Arguments:
    collection - an endpoint collection,
    config_file_path - a configuration file name.
*/
int config_load(const p_endpoints_collection_t collection, const char *config_file_path);

#endif