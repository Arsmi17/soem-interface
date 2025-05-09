#ifndef SLAVE_DETECTOR_API_H
#define SLAVE_DETECTOR_API_H

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize EtherCAT communication and detect slaves.
 * 
 * @param ifname Network interface (e.g., "eth0")
 * @return int Number of detected slaves, or -1 on failure
 */
int init_slave_detector(const char* ifname);

/**
 * @brief Get the total number of connected EtherCAT slaves.
 * 
 * @return int Slave count (>=0), or -1 if not initialized
 */
int get_slave_count(void);

/**
 * @brief Get the name of a specific slave by index (1-based).
 * 
 * @param index Slave index (starting from 1)
 * @param buffer Buffer to store the name
 * @param buflen Length of the buffer
 * @return int 0 on success, -1 on error
 */
int get_slave_name(int index, char* buffer, size_t buflen);

#ifdef __cplusplus
}
#endif

#endif // SLAVE_DETECTOR_API_H