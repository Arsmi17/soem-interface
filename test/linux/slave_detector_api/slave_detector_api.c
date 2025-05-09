#include "slave_detector_api.h"
#include "ethercat.h"
#include <string.h>

// Maximum number of slaves supported
#define MAX_SLAVES 16
#define MAX_NAME_LEN 128

// Static storage for slave info
static int slave_count = -1;
static char slave_names[MAX_SLAVES][MAX_NAME_LEN];

int init_slave_detector(const char* ifname) {
    char IOmap[4096];

    if (!ec_init((char*)ifname)) {
        fprintf(stderr, "Failed to initialize EtherCAT on %s\n", ifname);
        return -1;
    }

    slave_count = ec_config(FALSE, IOmap);

    if (slave_count <= 0) {
        fprintf(stderr, "No slaves found!\n");
        ec_close();
        return -1;
    }

    // Store slave names
    for (int i = 0; i < slave_count && i < MAX_SLAVES; i++) {
        snprintf(slave_names[i], MAX_NAME_LEN, "%s", ec_slave[i + 1].name);
    }

    return slave_count;
}

int get_slave_count(void) {
    return slave_count >= 0 ? slave_count : -1;
}

int get_slave_name(int index, char* buffer, size_t buflen) {
    if (buffer == NULL || buflen <= 0 || index < 1 || index > slave_count) {
        return -1;
    }

    strncpy(buffer, slave_names[index - 1], buflen - 1);
    buffer[buflen - 1] = '\0';
    return 0;
}