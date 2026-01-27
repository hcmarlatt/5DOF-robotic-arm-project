#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include <math.h>
/*
* ARM MEASUREMENTS (mm):
* - Base height: 97mm
* - Shoulder offset from base axis: 14mm
* - Link 1 (shoulder→elbow): 114mm
* - Link 2 (elbow→wrist roll): 87mm (5mm offset)
* - Wrist roll→pitch: 37mm
* - Wrist pitch→pointer tip: 80mm
* - Total Link2 for IK: 167mm (87 + 37 + 80)
* - Max reach: ~318mm
*/

// Link lengths in mm
#define LINK1 114.0  // Shoulder to elbow
#define LINK2 204.0  // Elbow to pointer tip (87 + 37 + 80)

int angle_to_pulse(int servo_num, int angle) {
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

int current_positions[5] = {0, 0, 0, 0, 0};  // Add after angle_to_pulse()

void set_servo_angle(uint servo_pin, int servo_num, int angle) {
    uint slice = pwm_gpio_to_slice_num(servo_pin);
    uint channel = pwm_gpio_to_channel(servo_pin);
    int target_pulse = angle_to_pulse(servo_num, angle);
    
    move_servo_slow(slice, channel, current_positions[servo_num], target_pulse, 1000);
    current_positions[servo_num] = target_pulse;
}

// Slow servo movement
void move_servo_slow(uint slice, uint channel, int start_pos, int end_pos, int duration_ms) {
    int steps = 50;
    int delay = duration_ms / steps;
    
    for (int i = 0; i <= steps; i++) {
        int current_pos = start_pos + ((end_pos - start_pos) * i / steps);
        pwm_set_chan_level(slice, channel, current_pos);
        sleep_ms(delay);
    }
}

// 2D IK function - returns shoulder and elbow angles
void calculate_2d_ik(float x, float z, float *shoulder_angle, float *elbow_angle) {
    float distance = sqrt(x*x + z*z);
    // Check if reachable
    if (distance > (LINK1 + LINK2)) {
        printf("Target unreachable! Distance: %.1f, Max reach: %.1f\n", distance, LINK1 + LINK2);
        return;
    }
    
    // Law of cosines for elbow angle
    float cos_elbow = (LINK1*LINK1 + LINK2*LINK2 - distance*distance) / 
                      (2 * LINK1 * LINK2);
    *elbow_angle = acos(cos_elbow) * 180.0 / M_PI;  // Convert to degrees
    
    // Calculate shoulder angle
    float angle_to_target = atan2(z, x) * 180.0 / M_PI;
    float cos_shoulder = (LINK1*LINK1 + distance*distance - LINK2*LINK2) / 
                         (2 * LINK1 * distance);
    float shoulder_offset = acos(cos_shoulder) * 180.0 / M_PI;
    
    *shoulder_angle = angle_to_target + shoulder_offset;
    
    printf("IK Solution: X=%.1f, Z=%.1f -> Shoulder=%.1f°, Elbow=%.1f°\n",
           x, z, *shoulder_angle, *elbow_angle);
}

int main() {
    // Pin definitions
    const uint LED_PIN = 16;
    const uint BASE = 15;
    const uint SHOULDER = 14;
    const uint ELBOW = 13;
    const uint WRIST_ROLL = 12;
    const uint WRIST_PITCH = 11;
    
    stdio_init_all();
    sleep_ms(2000);  // Wait for serial
    
    // LED setup
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
    // Blink confirmation
    for (int i = 0; i < 3; i++) {
        gpio_put(LED_PIN, 1);
        sleep_ms(200);
        gpio_put(LED_PIN, 0);
        sleep_ms(200);
    }
    
    // PWM setup
    uint servos[] = {BASE, SHOULDER, ELBOW, WRIST_ROLL, WRIST_PITCH};
    for (int i = 0; i < 5; i++) {
        gpio_set_function(servos[i], GPIO_FUNC_PWM);
        uint slice = pwm_gpio_to_slice_num(servos[i]);
        pwm_set_clkdiv(slice, 64.0f);
        pwm_set_wrap(slice, 39062);
        pwm_set_enabled(slice, true);
    }
    // Initialize to 90 degrees
    for (int i = 0; i < 5; i++) {
        current_positions[i] = angle_to_pulse(i, 90);
    }
    printf("=== 2D IK Test ===\n");
    printf("Enter target: X Z (in mm)\n");
    printf("Example: 150 100\n\n");
    
    // Set base and wrists to neutral
    set_servo_angle(BASE, 0, 90);
    set_servo_angle(WRIST_ROLL, 3, 90);
    set_servo_angle(WRIST_PITCH, 4, 145);
    
    while (true) {
        float x, z;
        printf("Enter X Z: ");
        scanf("%f %f", &x, &z);

        float shoulder_angle, elbow_angle;
        calculate_2d_ik(x, z, &shoulder_angle, &elbow_angle);
        
        // Compensate for servo mounting angle
        shoulder_angle = shoulder_angle + 28;

        // Remap IK angles to physical servo angles
        int shoulder_physical = 90 - (int)shoulder_angle;
        int elbow_physical = 90 - (180 - (int)elbow_angle);

        printf("Physical angles: Shoulder=%d°, Elbow=%d°\n", shoulder_physical, elbow_physical);

        // Move servos
        printf("Moving to position...\n");
        set_servo_angle(SHOULDER, 1, shoulder_physical);
        set_servo_angle(ELBOW, 2, elbow_physical);
        
        printf("Complete! Measure and verify.\n\n");
        sleep_ms(100);
    }
    
    return 0;
}