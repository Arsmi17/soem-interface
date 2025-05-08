#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include "ethercat.h"

typedef struct __attribute__((packed))
{
    uint16_t control_word;
    int32_t target_position;
    uint8_t mode_of_operation;
} el7_out_t;

typedef struct __attribute__((packed))
{
    uint16_t status_word;
    int32_t actual_position;
    int32_t actual_velocity;
    int16_t actual_torque;
} el7_in_t;

char IOmap[4096];
el7_out_t *out_data;
el7_in_t *in_data;

// Non-blocking keypress setup
void setup_keyboard()
{
    struct termios t;
    tcgetattr(STDIN_FILENO, &t);
    t.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &t);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
}

void restore_keyboard()
{
    struct termios t;
    tcgetattr(STDIN_FILENO, &t);
    t.c_lflag |= (ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &t);
}

void send_and_receive()
{
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    usleep(1000);
}

void move_to_absolute(el7_out_t *out_data, int position)
{
    out_data->target_position = position;

    // Set control word with bit 4 (new setpoint)
    out_data->control_word = 0x0F | 0x10; // Enable operation + New setpoint
    send_and_receive();
    usleep(10000);

    // Clear bit 4 (edge trigger)
    out_data->control_word = 0x0F;
    send_and_receive();
    usleep(10000);
}

void enable_drive()
{
    printf("Enabling the drive...\n");
    out_data->control_word = 0x06;
    send_and_receive();
    usleep(10000);

    out_data->control_word = 0x07;
    send_and_receive();
    usleep(10000);

    out_data->control_word = 0x0F;
    send_and_receive();
    usleep(10000);
}

void disable_drive()
{
    printf("Disabling drive...\n");
    out_data->control_word = 0x00;
    send_and_receive();
}

void reset_fault()
{
    printf("Resetting fault...\n");
    out_data->control_word = 0x80;
    send_and_receive();
    usleep(10000);
}

void move_to_position(int32_t pos)
{
    printf("Moving to position: %d\n", pos);
    out_data->target_position = pos;
    out_data->control_word = 0x1F;
    send_and_receive();
}

int main()
{
    if (!ec_init("enp0s31f6"))
    {
        printf("Failed to initialize EtherCAT interface.\n");
        return -1;
    }

    if (ec_config_init(FALSE) <= 0)
    {
        printf("No slaves found!\n");
        ec_close();
        return -1;
    }

    ec_config_map(&IOmap);

    // Optional: ec_configdc();

    uint32_t profile_velocity = 50000; // Units: counts/sec (adjust as needed)
    uint32_t max_velocity = 60000;
    uint32_t acceleration = 30000;
    uint32_t deceleration = 25000;

    ec_SDOwrite(1, 0x6081, 0x00, FALSE, sizeof(profile_velocity), &profile_velocity, EC_TIMEOUTSAFE);
    ec_SDOwrite(1, 0x607F, 0x00, FALSE, sizeof(max_velocity), &max_velocity, EC_TIMEOUTSAFE);
    ec_SDOwrite(1, 0x6083, 0x00, FALSE, sizeof(acceleration), &acceleration, EC_TIMEOUTSAFE);
    ec_SDOwrite(1, 0x6084, 0x00, FALSE, sizeof(deceleration), &deceleration, EC_TIMEOUTSAFE);

    // Watchdog and anti-dither fixes
    uint16_t val16 = 0;
    ec_SDOwrite(1, 0x5004, 0x07, FALSE, sizeof(val16), &val16, EC_TIMEOUTSAFE);
    ec_SDOwrite(1, 0x5004, 0x09, FALSE, sizeof(val16), &val16, EC_TIMEOUTSAFE);
    ec_SDOwrite(1, 0x5004, 0x0B, FALSE, sizeof(val16), &val16, EC_TIMEOUTSAFE);

    uint16_t sync1 = 100, sync2 = 500, delay = 5;
    ec_SDOwrite(1, 0x2025, 0x00, FALSE, sizeof(sync1), &sync1, EC_TIMEOUTSAFE);
    ec_SDOwrite(1, 0x2026, 0x00, FALSE, sizeof(sync2), &sync2, EC_TIMEOUTSAFE);
    ec_SDOwrite(1, 0x2027, 0x00, FALSE, sizeof(delay), &delay, EC_TIMEOUTSAFE);

    out_data = (el7_out_t *)ec_slave[1].outputs;
    in_data = (el7_in_t *)ec_slave[1].inputs;

    uint8_t mode = 1;
    ec_SDOwrite(1, 0x6060, 0x00, FALSE, sizeof(mode), &mode, EC_TIMEOUTSAFE);

    ec_slave[1].state = EC_STATE_OPERATIONAL;
    ec_writestate(1);
    usleep(10000);

    if (ec_statecheck(1, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE) != EC_STATE_OPERATIONAL)
    {
        printf("Failed to reach OPERATIONAL state.\n");
        ec_close();
        return -1;
    }

    printf("Slave is OPERATIONAL. Use keys:\n");
    printf("[e] Enable  [s] Set Position  [r] Reset Fault  [x] Disable  [q] Quit\n");

    setup_keyboard();
    int running = 1;
    // int move_pos = 0;

    while (running)
    {
        send_and_receive();

        printf("Status: 0x%04X | Pos: %d | Vel: %d | Torque: %d\r",
               in_data->status_word,
               in_data->actual_position,
               in_data->actual_velocity,
               in_data->actual_torque);
        fflush(stdout);

        int ch = getchar();
        switch (ch)
        {
        case 'e':
            enable_drive();
            break;
        case 'x':
            disable_drive();
            break;
        case 'r':
            reset_fault();
            break;
        case 'a':
            move_to_absolute(out_data, 100000);
            break;
        case 's':
            move_to_absolute(out_data, 200000);
            break;
        case 'd':
            move_to_absolute(out_data, 300000);
            break;
        case 'q':
            running = 0;
            break;
        }

        usleep(20000);
    }
    restore_keyboard();
    ec_slave[1].state = EC_STATE_INIT;
    ec_writestate(1);
    ec_close();
    printf("\nExited.\n");
    return 0;
}
