#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include "ethercat.h"

#define IFNAME "enp0s31f6" // 👈 Replace with your actual interface name
#define SLAVE_IDX 1        // EtherCAT slave index (starts from 1)

#define FORWARD_VEL 100000
#define REVERSE_VEL -100000
#define STOP_VEL 0

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
el7_out_t *out_data;
el7_in_t *in_data;

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

void enable_drive()
{
    out_data->mode_of_operation = 9;
    out_data->control_word = 0x06;
    send_and_receive();
    usleep(10000);
    out_data->control_word = 0x07;
    send_and_receive();
    usleep(10000);
    out_data->control_word = 0x0F;
    send_and_receive();
    usleep(10000);
    printf("✅ Drive enabled\n");
}

void disable_drive()
{
    out_data->control_word = 0x00;
    send_and_receive();
    printf("🛑 Drive disabled\n");
}

void reset_fault()
{
    out_data->control_word = 0x80;
    send_and_receive();
    usleep(10000);
    printf("🔄 Fault reset\n");
}

void set_velocity(int32_t vel)
{
    out_data->target_velocity = vel;
    out_data->mode_of_operation = 9;
    out_data->control_word = 0x0F;
    send_and_receive();
}

int main()
{
    if (!ec_init(IFNAME))
    {
        printf("❌ Failed to init EtherCAT on %s\n", IFNAME);
        return -1;
    }

    if (ec_config_init(FALSE) < SLAVE_IDX)
    {
        printf("❌ Slave not found!\n");
        ec_close();
        return -1;
    }

    ec_config_map(&IOmap);

    out_data = (el7_out_t *)ec_slave[SLAVE_IDX].outputs;
    in_data = (el7_in_t *)ec_slave[SLAVE_IDX].inputs;

    ec_slave[SLAVE_IDX].state = EC_STATE_OPERATIONAL;
    ec_writestate(SLAVE_IDX);
    usleep(10000);

    if (ec_statecheck(SLAVE_IDX, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE) != EC_STATE_OPERATIONAL)
    {
        printf("❌ Slave failed to reach OP state.\n");
        ec_close();
        return -1;
    }

    printf("✅ Slave is OPERATIONAL. Controls:\n");
    printf("🔼 UP: Forward | 🔽 DOWN: Reverse | 🛑 Release: Stop\n");
    printf("e = Enable | d = Disable | r = Reset Fault | ESC = Exit\n");

    setup_keyboard();
    int running = 1;
    int32_t velocity = STOP_VEL;

    while (running)
    {
        send_and_receive();

        printf("Status: 0x%04X | Pos: %d | Vel: %d | Torque: %d\r",
               in_data->status_word,
               in_data->actual_position,
               in_data->actual_velocity,
               in_data->actual_torque);

        int ch = getchar();
        switch (ch)
        {
        case 'i': // Forward
        case 'I':
            velocity = FORWARD_VEL;
            break;
        case 'k': // Reverse
        case 'K':
            velocity = REVERSE_VEL;
            break;
        case 'e':
            enable_drive();
            break;
        case 'd':
            disable_drive();
            break;
        case 'r':
            reset_fault();
            break;
        case -1:
            velocity = STOP_VEL;
            break; // nothing pressed
        }

        set_velocity(velocity);
        velocity = STOP_VEL; // default to stop unless held
        usleep(20000);
    }

    restore_keyboard();
    ec_slave[SLAVE_IDX].state = EC_STATE_INIT;
    ec_writestate(SLAVE_IDX);
    ec_close();
    printf("\n👋 Exited cleanly.\n");
    return 0;
}
