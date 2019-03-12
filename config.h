#ifndef CSC_CONFIG
#define CSC_CONFIG

#include "endpoint.h"

#define CSC_FILTER_MESSAGE_NAME_MAX 50
#define CSC_ENDPOINT_IP_MAX_LEN (3 * 4 + 3 * 1)
#define CSC_FILTER_MESSAGES_MAX 20
#define CSC_ENDPOINT_NAME_POSTFIX_MAX (1 + 5)
#define CSC_ENDPOINT_NAME_USER_MAX (ENDPOINT_NAME_MAX - CSC_ENDPOINT_NAME_POSTFIX_MAX)

int config_load(p_endpoints_collection_t collection, const char *config_file_path);

#endif