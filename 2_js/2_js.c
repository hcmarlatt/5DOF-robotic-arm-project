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

/*
 * JOYSTICK WIRING:
 * Wires out the bottom orientation
 * Joystick 1 (X/Z control):
 *   VCC/+5V → Pico 3.3V
 *   GND → Pico GND
 *   VRX (X-axis) → GPIO 26 (ADC0)
 *   VRY (Y-axis) → GPIO 27 (ADC1)
 *   SW (button) → Not connected
 */

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
    adc_gpio_init(26);  // Side joystick Y-axis (up/down / Z)
    adc_gpio_init(27);  // Top joystick Y-axis (left/right / Y)
    adc_gpio_init(28);  // Side joystick X-axis (forward/back / X)

    // Set base and wrists to neutral
    set_servo_angle(BASE, 0, 90);
    set_servo_angle(WRIST_ROLL, 3, 90);
    set_servo_angle(WRIST_PITCH, 4, 145);
    
    // Start at max reach position
    float current_x = 318.0;
    float current_y = 0;
    float current_z = 0.0;

    // Move to starting position
    float shoulder_angle, elbow_angle;
    if (calculate_2d_ik(current_x, current_z, &shoulder_angle, &elbow_angle)) {
        uint moving_pins[] = {SHOULDER, ELBOW};
        int moving_nums[] = {1, 2};
        int target_angles[] = {(int)shoulder_angle, (int)elbow_angle};
        move_servos_coordinated(moving_pins, moving_nums, target_angles, 2, 1500);
    }

    sleep_ms(1000);

    uint32_t last_print_time = 0;

    while (true) {
        // Read joystick
        adc_select_input(0);  // GPIO 26 - Side joystick axis 1
        int side_1_raw = adc_read();
        adc_select_input(1);  // GPIO 27 - Top joystick (X forward/back)
        int top_raw = adc_read();
        adc_select_input(2);  // GPIO 28 - Side joystick axis 2
        int side_2_raw = adc_read();
        
        // Calculate offset from center
        int dead_zone = 300;
        int offset_x = side_2_raw - 2048;    // X (forward/back) - side joystick axis 2
        int offset_y = top_raw - 2048;      // Y (base rotation) - top joystick
        int offset_z = side_1_raw - 2048;   // Z (up/down) - side joystick axis 1

        // Apply dead zone
        if (abs(offset_x) < dead_zone) offset_x = 0;
        if (abs(offset_y) < dead_zone) offset_y = 0;
        if (abs(offset_z) < dead_zone) offset_z = 0;

        // Convert to movement speed (mm per update)
        float speed = 15.0;
        float delta_x = (offset_x / 2048.0) * speed;       // Radial
        float delta_y = (offset_y / 2048.0) * speed;       // Base rotation
        float delta_z = -(offset_z / 2048.0) * speed;       // Vertical

        // Only move if any joystick is being pushed
        if (delta_x != 0 || delta_z != 0 || delta_y != 0) {
            // Current radial distance and base angle
            float current_radial = sqrt(current_x * current_x + current_y * current_y);
            float current_base_angle = atan2(current_y, current_x);
            
            // Apply X as radial change, Y as angular change, Z as vertical change
            float new_radial = current_radial + delta_x;
            float new_z = current_z + delta_z;
            
            // Convert delta_y to angular change (scale by distance so speed feels consistent)
            float angle_change = 0;
            if (current_radial > 1.0) {
                angle_change = delta_y / current_radial;  // radians
            }
            float new_base_angle = current_base_angle + angle_change;
            
            // Clamp radial to positive
            if (new_radial < 0) new_radial = 0;
            
            // Convert back to Cartesian
            float new_x = new_radial * cos(new_base_angle);
            float new_y = new_radial * sin(new_base_angle);
            
            float base_angle_deg = new_base_angle * 180.0 / M_PI;
            
            // Use 2D IK in the vertical plane (radial distance vs height)
            float shoulder_angle, elbow_angle;
            
            if (calculate_2d_ik(new_radial, new_z, &shoulder_angle, &elbow_angle)) {
                current_x = new_x;
                current_y = new_y;
                current_z = new_z;
                
                uint moving_pins[] = {BASE, SHOULDER, ELBOW};
                int moving_nums[] = {0, 1, 2};
                int target_angles[] = {(int)(90 + base_angle_deg), (int)shoulder_angle, (int)elbow_angle};
                move_servos_coordinated(moving_pins, moving_nums, target_angles, 3, 200);
                
            } else {
                // Boundary sliding: scale to reachable sphere
                float desired_dist = sqrt(new_radial * new_radial + new_z * new_z);
                
                if (desired_dist > (LINK1 + LINK2)) {
                    float scale = (LINK1 + LINK2) / desired_dist;
                    float clamped_radial = new_radial * scale;
                    float boundary_z = new_z * scale;
                    
                    float boundary_x = clamped_radial * cos(new_base_angle);
                    float boundary_y = clamped_radial * sin(new_base_angle);
                    
                    if (calculate_2d_ik(clamped_radial, boundary_z, &shoulder_angle, &elbow_angle)) {
                        current_x = boundary_x;
                        current_y = boundary_y;
                        current_z = boundary_z;
                        
                        uint moving_pins[] = {BASE, SHOULDER, ELBOW};
                        int moving_nums[] = {0, 1, 2};
                        int target_angles[] = {(int)(90 + base_angle_deg), (int)shoulder_angle, (int)elbow_angle};
                        move_servos_coordinated(moving_pins, moving_nums, target_angles, 3, 200);
                    }
                }
            }
        }
        // Print position once per second
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        if (current_time - last_print_time >= 1000) {
            printf("ADC0=%d ADC1=%d ADC2=%d | X=%.1f Y=%.1f Z=%.1f\n", side_1_raw, top_raw, side_2_raw, current_x, current_y, current_z);
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
        return false;
    }
    
    if (distance < fabs(LINK1 - LINK2)) {
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
        return true;
    } else if (config2_valid) {
        *shoulder_angle = shoulder_physical_2;
        *elbow_angle = elbow_physical_2;
        return true;
    } else {
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