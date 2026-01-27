#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdio.h>

int angle_to_pulse(int servo_num, int angle) {
    // MG995 servos (0-2): 0°=750, 180°=4600
    // SG90 servos (3-4): 0°=700, 180°=4550
    
    int min_pulse, max_pulse;
    if (servo_num < 3) {
        min_pulse = 750;
        max_pulse = 4600;
    } else {
        min_pulse = 700;
        max_pulse = 4550;
    }
    
    return min_pulse + (angle * (max_pulse - min_pulse) / 180);
}

void move_multiple_servos(int num_servos, uint servos[], int start_angles[], int end_angles[], int duration_ms) {
    int steps = 50;
    int delay = duration_ms / steps;
    
    // Get slice and channel for each servo
    uint slices[num_servos];
    uint channels[num_servos];
    int start_pulses[num_servos];
    int end_pulses[num_servos];
    
    for (int i = 0; i < num_servos; i++) {
        slices[i] = pwm_gpio_to_slice_num(servos[i]);
        channels[i] = pwm_gpio_to_channel(servos[i]);
        start_pulses[i] = angle_to_pulse(i, start_angles[i]);
        end_pulses[i] = angle_to_pulse(i, end_angles[i]);
    }
    
    // Move all servos together in small increments
    for (int step = 0; step <= steps; step++) {
        for (int s = 0; s < num_servos; s++) {
            int current_pulse = start_pulses[s] + 
                              ((end_pulses[s] - start_pulses[s]) * step / steps);
            pwm_set_chan_level(slices[s], channels[s], current_pulse);
        }
        sleep_ms(delay);
    }
}

int main() {
    // Pin definitions
    const uint LED_PIN = 16;
    uint servos[] = {15, 14, 13, 12, 11};  // Base, shoulder, elbow, wrist roll, wrist pitch
    
    stdio_init_all();
    
    // LED setup
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
    // Blink to confirm running
    for (int i = 0; i < 3; i++) {
        gpio_put(LED_PIN, 1);
        sleep_ms(200);
        gpio_put(LED_PIN, 0);
        sleep_ms(200);
    }
    
    // PWM setup for all servos
    for (int i = 0; i < 5; i++) {
        gpio_set_function(servos[i], GPIO_FUNC_PWM);
        uint slice = pwm_gpio_to_slice_num(servos[i]);
        pwm_set_clkdiv(slice, 64.0f);
        pwm_set_wrap(slice, 39062);
        pwm_set_enabled(slice, true);
    }
    
    printf("=== Coordinated Movement Test ===\n");
    printf("Moving 3 servos together\n\n");
    
    uint test_servos[] = {15, 14, 13};  // Base, shoulder, elbow
    
    while (true) {
    printf("Moving to position 1...\n");
    int start[] = {90, 45, 135, 90, 90};  // Added wrist roll and pitch
    int pos1[] = {90, 90, 90, 120, 60};   // Elbow moves WITH shoulder, wrists tilt
    move_multiple_servos(5, servos, start, pos1, 2000);
    sleep_ms(1000);
    
    printf("Moving to position 2...\n");
    int pos2[] = {120, 60, 60, 60, 120};  // Base rotates, arm extends, wrists flip
    move_multiple_servos(5, servos, pos1, pos2, 2000);
    sleep_ms(1000);
    
    printf("Returning to start...\n");
    move_multiple_servos(5, servos, pos2, start, 2000);
    sleep_ms(5000);
    
    printf("Loop complete\n\n");
}
    
    return 0;
}