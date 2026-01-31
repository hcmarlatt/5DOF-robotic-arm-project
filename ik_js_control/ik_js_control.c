#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>


// Function declarations
int angle_to_pulse(int servo_num, int angle);
void set_servo_angle(uint servo_pin, int servo_num, int angle);
void move_servo_slow(uint slice, uint channel, int start_pos, int end_pos, int duration_ms);
bool calculate_2d_ik(float x, float z, float *shoulder_angle, float *elbow_angle);
void move_servos_coordinated(uint servo_pins[], int servo_nums[], int target_angles[], int num_servos, int duration_ms);

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

// Current positions
int current_positions[5] = {0, 0, 0, 0, 0};


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

    // ADC setup for joystick
    adc_init();
    adc_gpio_init(26);  // X-axis
    adc_gpio_init(27);  // Y-axis

    // printf("=== 2D IK Joystick Control ===\n");
    // printf("Move joystick to control arm position\n\n");

    // printf("=== 2D IK Test ===\n");
    // printf("Enter target: X Z (in mm)\n");
    // printf("Example: 150 100\n\n");
    
    // Set base and wrists to neutral
    set_servo_angle(BASE, 0, 90);
    set_servo_angle(WRIST_ROLL, 3, 90);
    set_servo_angle(WRIST_PITCH, 4, 145);
    
    // Start at max reach position
float current_x = 318.0;
float current_z = 0.0;

// Move to starting position
float shoulder_angle, elbow_angle;
if (calculate_2d_ik(current_x, current_z, &shoulder_angle, &elbow_angle)) {
    uint moving_pins[] = {SHOULDER, ELBOW};
    int moving_nums[] = {1, 2};
    int target_angles[] = {(int)shoulder_angle, (int)elbow_angle};
    move_servos_coordinated(moving_pins, moving_nums, target_angles, 2, 1500);
}

// printf("Starting at position (%.1f, %.1f)\n", current_x, current_z);
sleep_ms(1000);

uint32_t last_print_time = 0;

while (true) {
    // Read joystick
    adc_select_input(0);
    int joy_x_raw = adc_read();
    adc_select_input(1);
    int joy_y_raw = adc_read();
    
    // Calculate offset from center
    int dead_zone = 300;
    int offset_x = joy_x_raw - 2048;
    int offset_y = joy_y_raw - 2048;
    
    // Apply dead zone
    if (abs(offset_x) < dead_zone) offset_x = 0;
    if (abs(offset_y) < dead_zone) offset_y = 0;
    
    // Convert to movement speed (mm per update)
    float speed = 15.0;  // Max 2mm per update
    float delta_x = (offset_x / 2048.0) * speed;
    float delta_z = (offset_y / 2048.0) * speed;
        
    // Only move if joystick is being pushed
    if (delta_x != 0 || delta_z != 0) {
        float new_x = current_x + delta_x;
        float new_z = current_z + delta_z;
        
        // Try requested movement
        if (calculate_2d_ik(new_x, new_z, &shoulder_angle, &elbow_angle)) {
            // Requested movement works - do it
            current_x = new_x;
            current_z = new_z;
            
            uint moving_pins[] = {SHOULDER, ELBOW};
            int moving_nums[] = {1, 2};
            int target_angles[] = {(int)shoulder_angle, (int)elbow_angle};
            move_servos_coordinated(moving_pins, moving_nums, target_angles, 2, 200);
            
            } 
        else {
        // Movement failed - slide along boundary at full speed
        float distance = sqrt(new_x*new_x + new_z*new_z);
        
        if (distance > (LINK1 + LINK2)) {
            // We're trying to go beyond max reach
            // Move along the arc tangent instead
            
            // Current angle on circle
            float current_angle = atan2(current_z, current_x);
            
            // Determine direction of movement along arc
            float cross = current_x * delta_z - current_z * delta_x;
            float angle_delta = (cross > 0 ? 1 : -1) * (sqrt(delta_x*delta_x + delta_z*delta_z) / (LINK1 + LINK2));
            
            // New position on arc
            float new_angle = current_angle + angle_delta;
            float boundary_x = (LINK1 + LINK2) * cos(new_angle);
            float boundary_z = (LINK1 + LINK2) * sin(new_angle);
            
            if (calculate_2d_ik(boundary_x, boundary_z, &shoulder_angle, &elbow_angle)) {
                current_x = boundary_x;
                current_z = boundary_z;
                
                uint moving_pins[] = {SHOULDER, ELBOW};
                int moving_nums[] = {1, 2};
                int target_angles[] = {(int)shoulder_angle, (int)elbow_angle};
                move_servos_coordinated(moving_pins, moving_nums, target_angles, 2, 200);
            }
        }
    }
}
// Print position once per second
uint32_t current_time = to_ms_since_boot(get_absolute_time());
    if (current_time - last_print_time >= 1000) {
        printf("Current position: Z=%.1f mm, X=%.1f mm\n", current_x, current_z);
        last_print_time = current_time;
}

sleep_ms(50);  // 20Hz update rate

}
return 0;
}

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
bool calculate_2d_ik(float x, float z, float *shoulder_angle, float *elbow_angle) {
    // Calculate distance to target
    float distance = sqrt(x*x + z*z);
    
    // Check reachability
    if (distance > (LINK1 + LINK2)) {
        // printf("Target unreachable! Distance: %.1f, Max: %.1f\n", distance, LINK1 + LINK2);
        return false;
    }
    
    if (distance < fabs(LINK1 - LINK2)) {
        // printf("Target too close! Minimum reach: %.1f\n", fabs(LINK1 - LINK2));
        return false;
    }
    
    // Calculate base angles using law of cosines
    float cos_elbow = (LINK1*LINK1 + LINK2*LINK2 - distance*distance) / (2.0 * LINK1 * LINK2);
    float angle_to_target = atan2(z, x) * 180.0 / M_PI;
    float cos_shoulder_offset = (LINK1*LINK1 + distance*distance - LINK2*LINK2) / (2.0 * LINK1 * distance);
    float shoulder_offset = acos(cos_shoulder_offset) * 180.0 / M_PI;
    
    // Two possible IK solutions
    float shoulder_ik_1 = angle_to_target + shoulder_offset;
    float elbow_ik_1 = 180.0 - (acos(cos_elbow) * 180.0 / M_PI);  // Invert
    
    float shoulder_ik_2 = angle_to_target - shoulder_offset;
    float elbow_ik_2 = -elbow_ik_1;
    
    // Apply mounting offset and convert to physical servo angles for both configs
    int shoulder_physical_1 = 90 - (int)(shoulder_ik_1 + 28);
    int elbow_physical_1 = 90 - (int)elbow_ik_1;
    
    int shoulder_physical_2 = 90 - (int)(shoulder_ik_2 + 28);
    int elbow_physical_2 = 90 - (int)elbow_ik_2;
    
    // Check which configuration has valid servo angles
    bool config1_valid = (shoulder_physical_1 >= 0 && shoulder_physical_1 <= 180 && elbow_physical_1 >= 0 && elbow_physical_1 <= 180);
    bool config2_valid = (shoulder_physical_2 >= 0 && shoulder_physical_2 <= 180 && elbow_physical_2 >= 0 && elbow_physical_2 <= 180);
    
    // Return the valid configuration (already converted to physical angles)
    if (config1_valid) {
        *shoulder_angle = shoulder_physical_1;
        *elbow_angle = elbow_physical_1;
        // printf("IK Config 1: Target(%.1f, %.1f) -> Servos S=%d° E=%d°\n", x, z, shoulder_physical_1, elbow_physical_1);
        return true;
    } else if (config2_valid) {
        *shoulder_angle = shoulder_physical_2;
        *elbow_angle = elbow_physical_2;
        // printf("IK Config 2: Target(%.1f, %.1f) -> Servos S=%d° E=%d°\n", x, z, shoulder_physical_2, elbow_physical_2);
        return true;
    } else {
        // printf("No valid servo angles! Config1: S=%d E=%d, Config2: S=%d E=%d\n", shoulder_physical_1, elbow_physical_1, shoulder_physical_2, elbow_physical_2);
        return false;
    }
}

void move_servos_coordinated(uint servo_pins[], int servo_nums[], int target_angles[], int num_servos, int duration_ms) {
    int steps = 50;
    int delay = duration_ms / steps;
    
    // Get starting pulse values for each servo
    int start_pulses[num_servos];
    int end_pulses[num_servos];
    uint slices[num_servos];
    uint channels[num_servos];
    
    for (int i = 0; i < num_servos; i++) {
        start_pulses[i] = current_positions[servo_nums[i]];
        end_pulses[i] = angle_to_pulse(servo_nums[i], target_angles[i]);
        slices[i] = pwm_gpio_to_slice_num(servo_pins[i]);
        channels[i] = pwm_gpio_to_channel(servo_pins[i]);
    }
    
    // Move all servos together in small increments
    for (int step = 0; step <= steps; step++) {
        for (int i = 0; i < num_servos; i++) {
            int current_pulse = start_pulses[i] + 
                              ((end_pulses[i] - start_pulses[i]) * step / steps);
            pwm_set_chan_level(slices[i], channels[i], current_pulse);
        }
        sleep_ms(delay);
    }
    
    // Update tracked positions
    for (int i = 0; i < num_servos; i++) {
        current_positions[servo_nums[i]] = end_pulses[i];
    }
}