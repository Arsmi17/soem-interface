#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "soem/ethercat.h"

char ifname[] = "enp0s31f6"; // Change to your Ethernet interface

int main() {
    if (!ec_init(ifname)) {
        printf("No socket connection on %s\n", ifname);
        return -1;
    }

    printf("EtherCAT socket initialized on %s\n", ifname);

    if (ec_config_init(FALSE) <= 0) {
        printf("No EtherCAT slaves found.\n");
        ec_close();
        return -1;
    }

    printf("Found %d slaves, setting up...\n", ec_slavecount);

    // Map PDOs and transition to Safe-Op
    ec_config_map(NULL);
    ec_configdc();
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

    // Set CSP mode via SDO
    int8 mode = 8;  // CSP
    ec_SDOwrite(1, 0x6060, 0x00, FALSE, sizeof(mode), &mode, EC_TIMEOUTRXM);

    // Request Operational
    ec_slave[0].state = EC_STATE_OPERATIONAL;
    ec_writestate(0);
    ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);

    if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
        printf("Drive is in OPERATIONAL state.\n");

        // Output/Input buffer
        uint16 control_word;
        int32 target_position = 10000;
        uint16 status_word;
        int32 actual_position;

        // Enable the drive: 0x06 → 0x07 → 0x0F
        uint16 enable_sequence[] = {0x06, 0x07, 0x0F};

        for (int i = 0; i < 3; i++) {
            control_word = enable_sequence[i];
            memcpy(ec_slave[1].outputs, &control_word, 2);
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            usleep(100000);
        }

        // Send target position
        control_word = 0x1F;  // Enable + new setpoint
        memcpy(ec_slave[1].outputs, &control_word, 2);
        memcpy((uint8*)ec_slave[1].outputs + 4, &target_position, 4);

        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        usleep(100000);

        // Monitor drive state
        for (int i = 0; i < 50; i++) {
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);

            memcpy(&status_word, ec_slave[1].inputs, 2);
            memcpy(&actual_position, (uint8*)ec_slave[1].inputs + 4, 4);

            printf("[%d] Status: 0x%04X, Position: %d\n", i, status_word, actual_position);
            usleep(100000);
        }
    } else {
        printf("Failed to reach OPERATIONAL state.\n");
    }

    ec_slave[0].state = EC_STATE_INIT;
    ec_writestate(0);
    ec_close();

    return 0;
}