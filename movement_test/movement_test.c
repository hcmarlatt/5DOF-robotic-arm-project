#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "stdio.h"

void move_servo_slow(uint slice, uint channel, int start_pos, int end_pos, int duration_ms) {
    int steps = 50;
    int delay = duration_ms / steps;
    
    for (int i = 0; i <= steps; i++) {
        int current_pos = start_pos + ((end_pos - start_pos) * i / steps);
        pwm_set_chan_level(slice, channel, current_pos);
        sleep_ms(delay);
    }
}

int angle_to_pulse(int servo_num, int angle) {
    // MG995 servos (0-2): 0°=750, 180°=4600
    // SG90 servos (3-5): 0°=700, 180°=4550
    
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

int main() {
    // Pin definitions
    const uint LED_PIN = 16;
    uint servos[] = {15, 14, 13, 12, 11, 10};  // Base, shoulder, elbow, wrist roll, wrist pitch, gripper
    const char* servo_names[] = {"Base Yaw", "Shoulder", "Elbow", "Wrist Roll", "Wrist Pitch", "Gripper"};
    
    stdio_init_all();
    
    // LED setup
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Blink 3 times to confirm code is running
for (int i = 0; i < 3; i++) {
    gpio_put(LED_PIN, 1);
    sleep_ms(1000);
    gpio_put(LED_PIN, 0);
    sleep_ms(200);
}
    
    // PWM setup
    for (int i = 0; i < 6; i++) {
        gpio_set_function(servos[i], GPIO_FUNC_PWM);
        uint slice = pwm_gpio_to_slice_num(servos[i]);
        pwm_set_clkdiv(slice, 64.0f);
        pwm_set_wrap(slice, 39062);
        pwm_set_enabled(slice, true);
    }
    
    // Track current positions (start at 90°)
    int current_positions[6];
    for (int i = 0; i < 6; i++) {
        current_positions[i] = angle_to_pulse(i, 90);
    }
    
    printf("Servo Manual Control\n");
    printf("Format: servo_num angle (e.g., '2 90')\n");
    printf("Servos: 0=Base, 1=Shoulder, 2=Elbow, 3=Wrist Roll, 4=Wrist Pitch, 5=Gripper\n\n");
    
    while (true) {
    int servo_num, angle;
    
    printf("Enter command: ");
    scanf("%d %d", &servo_num, &angle);
    
    if (servo_num < 0 || servo_num > 5) {
        printf("Error: servo_num must be 0-5\n");
        continue;
    }
    
    if (angle < 0 || angle > 180) {
        printf("Error: angle must be 0-180\n");
        continue;
    }
    
    int target_pulse = angle_to_pulse(servo_num, angle);
    uint slice = pwm_gpio_to_slice_num(servos[servo_num]);
    uint channel = pwm_gpio_to_channel(servos[servo_num]);
    
    printf("Moving %s to %d degrees (pulse: %d)\n", servo_names[servo_num], angle, target_pulse);
    
    gpio_put(LED_PIN, 1);
    move_servo_slow(slice, channel, current_positions[servo_num], target_pulse, 1000);
    current_positions[servo_num] = target_pulse;
    gpio_put(LED_PIN, 0);
    
    printf("Complete\n\n");
}
    
    return 0;
}