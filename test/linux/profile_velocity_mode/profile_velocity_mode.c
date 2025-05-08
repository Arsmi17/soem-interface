#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include "ethercat.h"

#define NUM_SLAVES 2

typedef struct __attribute__((packed))
{
    uint16_t control_word;
    int32_t target_velocity;
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
el7_out_t *out_data[NUM_SLAVES];
el7_in_t *in_data[NUM_SLAVES];

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

void move_with_velocity(int slave_idx, int velocity)
{
    out_data[slave_idx]->mode_of_operation = 3; // Profile Velocity
    out_data[slave_idx]->target_velocity = velocity;
    out_data[slave_idx]->control_word = 0x0F;
    send_and_receive();
}

void stop_motion(int slave_idx)2
{
    move_with_velocity(slave_idx, 0);
}

void enable_drive(int slave_idx)
{
    printf("Enabling drive %d...\n", slave_idx + 1);
    out_data[slave_idx]->control_word = 0x06;
    send_and_receive();
    usleep(10000);
    out_data[slave_idx]->control_word = 0x07;
    send_and_receive();
    usleep(10000);
    out_data[slave_idx]->control_word = 0x0F;
    send_and_receive();
    usleep(10000);
}

void disable_drive(int slave_idx)
{
    printf("Disabling drive %d...\n", slave_idx + 1);
    out_data[slave_idx]->control_word = 0x00;
    send_and_receive();
}

void reset_fault(int slave_idx)
{
    printf("Resetting fault on drive %d...\n", slave_idx + 1);
    out_data[slave_idx]->control_word = 0x80;
    send_and_receive();
    usleep(10000);
}

int main()
{
    if (!ec_init("enp0s31f6"))
    {
        printf("Failed to initialize EtherCAT interface.\n");
        return -1;
    }

    if (ec_config_init(FALSE) < NUM_SLAVES)
    {
        printf("Not enough slaves found!\n");
        ec_close();
        return -1;
    }

    ec_config_map(&IOmap);

    for (int i = 0; i < NUM_SLAVES; ++i)
    {
        out_data[i] = (el7_out_t *)ec_slave[i + 1].outputs;
        in_data[i] = (el7_in_t *)ec_slave[i + 1].inputs;

        uint8_t mode = 3; // Profile Velocity
        uint32_t profile_velocity = 50000;
        uint32_t max_velocity = 60000;
        uint32_t acceleration = 30000;
        uint32_t deceleration = 25000;

        ec_SDOwrite(i + 1, 0x6060, 0x00, FALSE, sizeof(mode), &mode, EC_TIMEOUTSAFE);
        ec_SDOwrite(i + 1, 0x6081, 0x00, FALSE, sizeof(profile_velocity), &profile_velocity, EC_TIMEOUTSAFE);
        ec_SDOwrite(i + 1, 0x607F, 0x00, FALSE, sizeof(max_velocity), &max_velocity, EC_TIMEOUTSAFE);
        ec_SDOwrite(i + 1, 0x6083, 0x00, FALSE, sizeof(acceleration), &acceleration, EC_TIMEOUTSAFE);
        ec_SDOwrite(i + 1, 0x6084, 0x00, FALSE, sizeof(deceleration), &deceleration, EC_TIMEOUTSAFE);
    }

    for (int i = 0; i < NUM_SLAVES; ++i)
    {
        ec_slave[i + 1].state = EC_STATE_OPERATIONAL;
        ec_writestate(i + 1);
    }
    usleep(10000);

    for (int i = 0; i < NUM_SLAVES; ++i)
    {
        if (ec_statecheck(i + 1, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE) != EC_STATE_OPERATIONAL)
        {
            printf("Slave %d failed to reach OPERATIONAL state.\n", i + 1);
            ec_close();
            return -1;
        }
    }

    printf("Slaves are OPERATIONAL. Use arrow keys: Left/Right: Slave 0 Up/Down: Slave 1");

    setup_keyboard();
    int running = 1;

    while (running)
    {
        send_and_receive();

        // for (int i = 0; i < NUM_SLAVES; ++i)
        // {
        //     printf("Slave %d -> Status: 0x%04X | Pos: %d | Vel: %d | Torque: %d\n",
        //            i + 1,
        //            in_data[i]->status_word,
        //            in_data[i]->actual_position,
        //            in_data[i]->actual_velocity,
        //            in_data[i]->actual_torque);
        // }

        int ch = getchar();
        switch (ch)
        {
        case '1': for (int i = 0; i < NUM_SLAVES; i++) enable_drive(i); break;
        case '2': for (int i = 0; i < NUM_SLAVES; i++) disable_drive(i); break;
        case '3': for (int i = 0; i < NUM_SLAVES; i++) reset_fault(i); break;

        case 'A': // Up Arrow (ASCII 65)
            printf("moving upwards");
            move_with_velocity(1, 30000);
            break;
        case 'B': // Down Arrow (ASCII 66)
            printf("moving downwards");
            move_with_velocity(1, -30000);
            break;
        case 'C': // Right Arrow (ASCII 67)
            printf("moving right");
            move_with_velocity(0, 30000);
            break;
        case 'D': // Left Arrow (ASCII 68)
            printf("moving left");
            move_with_velocity(0, -30000);
            break;
        case 27:
            if (getchar() == 91) // skip '[' of arrow keys
            {
                int arrow = getchar();
                switch (arrow)
                {
                    case 'A': move_with_velocity(1, 30000); break; // Up
                    case 'B': move_with_velocity(1, -30000); break; // Down
                    case 'C': move_with_velocity(0, 30000); break; // Right
                    case 'D': move_with_velocity(0, -30000); break; // Left
                }
            }
            else
            {
                running = 0; // ESC key
            }
            break;
        default:
            // Stop motors when no key is pressed
            stop_motion(0);
            stop_motion(1);
            break;
        }

        usleep(20000);
    }

    restore_keyboard();
    for (int i = 0; i < NUM_SLAVES; ++i)
    {
        ec_slave[i + 1].state = EC_STATE_INIT;
        ec_writestate(i + 1);
    }
    ec_close();
    printf("\nExited.\n");
    return 0;
}
