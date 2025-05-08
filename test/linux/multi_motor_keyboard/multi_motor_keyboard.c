#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include "ethercat.h"
#include <stdlib.h>
#include <signal.h>


#define NUM_SLAVES 4
volatile sig_atomic_t keep_running = 1;


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

#define POSITION_ACTUAL_VALUE_OFFSET 0x00 // Update if your mapping differs

int32_t read_current_position(int slave_id) {
    int32_t position = 0;
    uint8_t *pos_ptr = ec_slave[slave_id + 1].inputs + POSITION_ACTUAL_VALUE_OFFSET;

    // Interpret 4 bytes as int32_t
    position = (int32_t)((pos_ptr[3] << 24) | (pos_ptr[2] << 16) | (pos_ptr[1] << 8) | pos_ptr[0]);

    return position;
}

void move_to_absolute(int slave_idx, int position)
{
    out_data[slave_idx]->target_position = position;
    out_data[slave_idx]->control_word = 0x0F | 0x10; // New setpoint
    send_and_receive();
    usleep(10000);
    out_data[slave_idx]->control_word = 0x0F; // Clear new setpoint
    send_and_receive();
    usleep(10000);
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

void move_to_zero(int slave_idx)
{
    printf("Moving slave %d to position 0...\n", slave_idx + 1);

    // Set target position to 0
    out_data[slave_idx]->target_position = 0;
    out_data[slave_idx]->control_word = 0x0F | 0x10; // Enable + New Setpoint
    send_and_receive();
    usleep(10000);
    out_data[slave_idx]->control_word = 0x0F; // Clear new setpoint bit
    send_and_receive();
    // Wait for motor to reach position 0
    usleep(10000);
}


void wait_until_reached(int slave, int target_position) {
    int retries = 0;
    while (1) {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);

        int current_pos = read_current_position(slave);
        printf("Slave %d current position: %d, Target: %d\n", slave, current_pos, target_position);

        if (abs(current_pos - target_position) < 50) {
            printf("Slave %d reached position %d\n", slave, target_position);
            break;
        }

        usleep(1000);
        retries++;

        if (retries > 10000) { // 10 seconds timeout
            printf("ERROR: Slave %d failed to reach position %d\n", slave, target_position);
            break;
        }
    }
}



void template_1() {
    printf("Running Template 1 Sequence...\n");

    // 1. Move slave 1 to 90000
    move_to_absolute(1, 90000);
    wait_until_reached(1, 90000);

    // 2. Move slave 0 to 90000
    move_to_absolute(0, 90000);
    wait_until_reached(0, 90000);

    // 3. Move slave 0 to 0
    move_to_absolute(0, 0);
    wait_until_reached(0, 0);

    // 4. Move slave 1 to 0
    move_to_absolute(1, 0);
    wait_until_reached(1, 0);

    printf("Template 1 Complete.\n");
}

void template_5() {
    printf("Running Template 2 Sequence...\n");

    // Step 1: Move both motors to 90000
    move_to_absolute(0, 120000);
    move_to_absolute(1, 120000);
    move_to_absolute(2, 90000);
    move_to_absolute(3, 90000);
}



void template_6(){
    move_to_absolute(3, 1600000);
}

void template_7(){
    move_to_absolute(0,100000);
}


void template_4(){
    move_to_absolute(0, 100000);
    move_to_absolute(2, -100000);
    wait_until_reached(3,-100000);

    move_to_absolute(0, 0);
    move_to_absolute(2, 0);
    wait_until_reached(3,0);
}




void template_2(){
    move_to_absolute(1, 455000);
    move_to_absolute(3, -455000);
    // wait_until_reached(3,-320000);

    // move_to_absolute(1, 0);
    // move_to_absolute(3, 0);
    // wait_until_reached(3,0);
}

void template_8(){
    move_to_absolute(3,1600000);
    move_to_absolute(1,1600000);


    // move_to_absolute(0,100000);
    // move_to_absolute(2,100000);  

    // wait_until_reached(2,100000);
    
    // move_to_absolute(0,0);
    // move_to_absolute(2,0);

    // move_to_absolute(1,800000);
    // move_to_absolute(3,800000);
    // move_to_absolute(0,0);
    // move_to_absolute(2,0);
}

void template_9(){
    move_to_absolute(1,800000);
    move_to_absolute(3,800000);
    move_to_absolute(0,0);
    move_to_absolute(2,0);
    // wait_until_reached(2,100000);

    // move_to_absolute(0,0);
    // move_to_absolute(2,0);
}

void template_10(){
    move_to_absolute(1,0);
    move_to_absolute(3,1600000);
    move_to_absolute(0,25000);
    move_to_absolute(2,-25000);
}

void template_3() {
    printf("Running Template 1 Sequence...\n");

    // 1. Move slave 1 to 90000
    move_to_absolute(3, 90000);
    wait_until_reached(3, 90000);

    // 2. Move slave 0 to 90000
    move_to_absolute(2, 90000);
    wait_until_reached(2, 90000);

    // 3. Move slave 0 to 0
    move_to_absolute(2, 0);
    wait_until_reached(2, 0);

    // 4. Move slave 1 to 0
    move_to_absolute(3, 0);
    wait_until_reached(3, 0);

    printf("Template 1 Complete.\n");
}


void int_handler(int sig)
{
    (void)sig;
    printf("\nCaught SIGINT (Ctrl+C), stopping motors...\n");

    for (int i = 0; i < NUM_SLAVES; ++i)
    {
        disable_drive(i); // Make sure this stops the motor or puts it in safe state
    }

    ec_close(); // Close EtherCAT connection if open
    restore_keyboard(); // Optional: if you used raw mode for input

    exit(0); // Exit safely
}



void perform_homing(int slave_idx)
{
    printf("Starting homing procedure for drive %d...\n", slave_idx + 1);

    // Set mode of operation to Homing Mode (6)
    uint8_t mode = 6;
    ec_SDOwrite(slave_idx + 1, 0x6060, 0x00, FALSE, sizeof(mode), &mode, EC_TIMEOUTSAFE);
    usleep(10000);

    // Set homing method (example: 35)
    int8_t homing_method = 35; 
    ec_SDOwrite(slave_idx + 1, 0x6098, 0x00, FALSE, sizeof(homing_method), &homing_method, EC_TIMEOUTSAFE);
    usleep(10000);

    // Start homing
    out_data[slave_idx]->control_word = 0x1F; // Enable operation + Start homing
    send_and_receive();
    usleep(10000);

    // Monitor homing
    printf("Waiting for homing to complete...\n");
    uint16_t status_word;
    int homing_complete = 0;
    int timeout_counter = 0;
    while (!homing_complete && timeout_counter < 500)
    {
        send_and_receive();
        status_word = in_data[slave_idx]->status_word;

        // Print status bits for debugging
        printf("Status Word: 0x%04X\n", status_word);

        // Bit 12 (homing attained), Bit 10 (target reached)
        if ((status_word & 0x3400) == 0x3400)
        {
            homing_complete = 1;
            printf("Homing completed for drive %d.\n", slave_idx + 1);
        }

        usleep(20000);
        timeout_counter++;
    }

    if (!homing_complete)
    {
        printf("Homing timed out or failed for drive %d.\n", slave_idx + 1);
    }

    // Return to Profile Position Mode (1)
    mode = 1;
    ec_SDOwrite(slave_idx + 1, 0x6060, 0x00, FALSE, sizeof(mode), &mode, EC_TIMEOUTSAFE);
    usleep(10000);
}

int main()
{
    signal(SIGINT, int_handler);

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

        uint8_t mode = 1;

        // Default motion parameters
        uint32_t profile_velocity = 50000;
        uint32_t max_velocity = 60000;
        uint32_t acceleration = 30000;
        uint32_t deceleration = 25000;

        // If only 1 slave, reduce all values by half
        if (i==0 || i==2)
        {
            profile_velocity /= 8;
            max_velocity /= 8;
            acceleration /= 8;
            deceleration /= 8;
        }

        // Setting it to Profile Position Mode
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

    printf("Slaves are OPERATIONAL. Use keys:\n");
    printf("[1-2] Enable  [q,w] Position  [a,s] Reset Fault  [z,x] Disable  [h] Homing  [ESC] Quit\n");

    setup_keyboard();
    int running = 1;

    while (running)
    {
        send_and_receive();

        for (int i = 0; i < NUM_SLAVES; ++i)
        {
            printf("Slave %d -> Status: 0x%04X | Pos: %d | Vel: %d | Torque: %d\n",
                   i + 1,
                   in_data[i]->status_word,
                   in_data[i]->actual_position,
                   in_data[i]->actual_velocity,
                   in_data[i]->actual_torque);
        }

        int ch = getchar();
        switch (ch)
        {
        case '1':
            for (int i = 0; i < NUM_SLAVES; i++)
                enable_drive(i);
            break;
        case '2':
            for (int i = 0; i < NUM_SLAVES; i++)
                disable_drive(i);
            break;
        case '3':
            for (int i = 0; i < NUM_SLAVES; i++)
                reset_fault(i);
            break;

        // 0th slave
        // case 'q':
        //     move_to_absolute(0, 30000);
        //     break;
        // case 'w':
        //     move_to_absolute(0, 60000);
        //     break;
        // case 'e':
        //     move_to_absolute(0, 90000);
        //     break;

        // 1st slave
        // case 'p':
        //     move_to_absolute(1, 30000);
        //     break;
        // case 'o':
        //     move_to_absolute(1, 60000);
        //     break;
        // case 'i':
        //     move_to_absolute(1, 90000);
        //     break;

        case 'q':
            template_8();
            break;

        //     break;
        // case 'w':
        //     template_3();
        //     break;
        case 'w':
            template_9();
            break;

        case 'e':
            template_10();
            break;

        // case 'r':
        //     template_4();
        //     break;

        // Homing
        case 'h':
            for (int i = 0; i < NUM_SLAVES; i++)
                move_to_zero(i);
            break;

        case 27:
            running = 0;
            break; // ESC key
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