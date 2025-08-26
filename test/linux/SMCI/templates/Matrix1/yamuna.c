// demo.c - sample template file

#include <stdio.h>

void test_first_pattern() {
    printf("Running test_first_pattern()\n");
}

void delay_with_communication() {
    // Simulate communication delay
    printf("[IGNORED] delay_with_communication()\n");
}

void test_second_pattern() {
    printf("Running test_second_pattern()\n");
}

void test_third_pattern() {
    printf("Running test_third_pattern()\n");
}

void test_fourth_pattern() {
    printf("Running test_third_pattern()\n");
}

// Common entry point
void run_template() 
{
    test_first_pattern();
    delay_with_communication(); // This will be skipped in button generation
    test_second_pattern();
    test_third_pattern();
    test_fourth_pattern();
}
