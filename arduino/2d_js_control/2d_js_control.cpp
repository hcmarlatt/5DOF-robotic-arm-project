#include <Servo.h>
#include <math.h>

// Pin definitions
#define BASE_PIN        9
#define SHOULDER_PIN    11
#define ELBOW_PIN       12
#define WRIST_ROLL_PIN  6
#define WRIST_PITCH_PIN 5

#define JOY_X_PIN A0
#define JOY_Y_PIN A1
#define JOY2_Y_PIN A2  // Second joystick - wrist pitch

// Servo indices
#define BASE        0
#define SHOULDER    1
#define ELBOW       2
#define WRIST_ROLL  3
#define WRIST_PITCH 4

// Link lengths in mm
#define LINK1 114.0  // Shoulder to elbow
#define LINK2 204.0  // Elbow to pointer tip (87 + 37 + 80)

Servo servos[5];
const uint8_t servo_pins[5] = {BASE_PIN, SHOULDER_PIN, ELBOW_PIN, WRIST_ROLL_PIN, WRIST_PITCH_PIN};

// Current positions stored as microseconds
int current_positions[5] = {0, 0, 0, 0, 0};

// Pulse ranges per servo - matching your Pico calibration
// Servos 0-2 (base/shoulder/elbow): 750–4600 over 180° on Pico (wrap=39062, clkdiv=64)
// Servos 3-4 (wrist roll/pitch):    700–4550
//
// Pico PWM: period = 39062 * (64/125MHz) = ~20ms
// So pulse counts map directly to microseconds:
//   Pico count 750  → 750  * (20000/39062) ≈ 384 µs
//   Pico count 4600 → 4600 * (20000/39062) ≈ 2355 µs
//
// Conversion factor: µs = count * (20000.0 / 39062.0) = count * 0.512
#define PICO_TO_US(count) ((int)((count) * 20000.0 / 39062.0))

// Pre-computed pulse ranges in microseconds
// Servos 0-2: 750→384µs, 4600→2355µs
// Servos 3-4: 700→358µs, 4550→2330µs
const int min_pulse_us[5] = { PICO_TO_US(750),  PICO_TO_US(750),  PICO_TO_US(750),  PICO_TO_US(700),  PICO_TO_US(700)  };
const int max_pulse_us[5] = { PICO_TO_US(4600), PICO_TO_US(4600), PICO_TO_US(4600), PICO_TO_US(4550), PICO_TO_US(4550) };

// Current arm position in mm
float current_x = 318.0;
float current_z = 0.0;
int wrist_pitch_angle = 145;  // Track wrist pitch angle directly

// Function declarations
int angle_to_us(int servo_num, int angle);
void set_servo_angle(int servo_num, int angle);
void move_servo_slow(int servo_num, int end_us, int duration_ms);
bool calculate_2d_ik(float x, float z, float *shoulder_angle, float *elbow_angle);
void move_servos_coordinated(int servo_nums[], int target_angles[], int num_servos, int duration_ms);

// Convert angle to microseconds for a given servo
int angle_to_us(int servo_num, int angle) {
    return min_pulse_us[servo_num] + ((long)angle * (max_pulse_us[servo_num] - min_pulse_us[servo_num]) / 180);
}

void set_servo_angle(int servo_num, int angle) {
    int target_us = angle_to_us(servo_num, angle);
    move_servo_slow(servo_num, target_us, 1000);
    current_positions[servo_num] = target_us;
}

void move_servo_slow(int servo_num, int end_us, int duration_ms) {
    int steps = 50;
    int step_delay = duration_ms / steps;
    int start_us = current_positions[servo_num];

    for (int i = 0; i <= steps; i++) {
        int current_us = start_us + ((long)(end_us - start_us) * i / steps);
        servos[servo_num].writeMicroseconds(current_us);
        delay(step_delay);
    }
}

bool calculate_2d_ik(float x, float z, float *shoulder_angle, float *elbow_angle) {
    float distance = sqrt(x * x + z * z);

    if (distance > (LINK1 + LINK2)) return false;
    if (distance < fabs(LINK1 - LINK2)) return false;

    float cos_elbow = (LINK1 * LINK1 + LINK2 * LINK2 - distance * distance) / (2.0 * LINK1 * LINK2);
    float angle_to_target = atan2(z, x) * 180.0 / M_PI;
    float cos_shoulder_offset = (LINK1 * LINK1 + distance * distance - LINK2 * LINK2) / (2.0 * LINK1 * distance);
    float shoulder_offset = acos(cos_shoulder_offset) * 180.0 / M_PI;

    // Two IK solutions
    float shoulder_ik_1 = angle_to_target + shoulder_offset;
    float elbow_ik_1 = 180.0 - (acos(cos_elbow) * 180.0 / M_PI);

    float shoulder_ik_2 = angle_to_target - shoulder_offset;
    float elbow_ik_2 = -elbow_ik_1;

    // Apply mounting offset (28°) and convert to physical servo angles
    int shoulder_physical_1 = 90 - (int)(shoulder_ik_1 + 28);
    int elbow_physical_1 = 90 - (int)elbow_ik_1;

    int shoulder_physical_2 = 90 - (int)(shoulder_ik_2 + 28);
    int elbow_physical_2 = 90 - (int)elbow_ik_2;

    bool config1_valid = (shoulder_physical_1 >= 0 && shoulder_physical_1 <= 180 &&
                          elbow_physical_1 >= 0 && elbow_physical_1 <= 180);
    bool config2_valid = (shoulder_physical_2 >= 0 && shoulder_physical_2 <= 180 &&
                          elbow_physical_2 >= 0 && elbow_physical_2 <= 180);

    if (config1_valid) {
        *shoulder_angle = shoulder_physical_1;
        *elbow_angle = elbow_physical_1;
        return true;
    } else if (config2_valid) {
        *shoulder_angle = shoulder_physical_2;
        *elbow_angle = elbow_physical_2;
        return true;
    }
    return false;
}

void move_servos_coordinated(int servo_nums[], int target_angles[], int num_servos, int duration_ms) {
    int steps = 50;
    int step_delay = duration_ms / steps;

    int start_us[5], end_us[5];
    for (int i = 0; i < num_servos; i++) {
        start_us[i] = current_positions[servo_nums[i]];
        end_us[i] = angle_to_us(servo_nums[i], target_angles[i]);
    }

    for (int step = 0; step <= steps; step++) {
        for (int i = 0; i < num_servos; i++) {
            int current_us = start_us[i] + ((long)(end_us[i] - start_us[i]) * step / steps);
            servos[servo_nums[i]].writeMicroseconds(current_us);
        }
        delay(step_delay);
    }

    for (int i = 0; i < num_servos; i++) {
        current_positions[servo_nums[i]] = end_us[i];
    }
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    // LED blink confirmation
    pinMode(LED_BUILTIN, OUTPUT);
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(200);
        digitalWrite(LED_BUILTIN, LOW);
        delay(200);
    }

    // Attach servos with per-servo pulse ranges
    for (int i = 0; i < 5; i++) {
        servos[i].attach(servo_pins[i], min_pulse_us[i], max_pulse_us[i]);
    }

    // Initialize tracked positions to 90°
    for (int i = 0; i < 5; i++) {
        current_positions[i] = angle_to_us(i, 90);
    }

    // Set base and wrists to neutral
    set_servo_angle(BASE, 90);
    set_servo_angle(WRIST_ROLL, 90);
    set_servo_angle(WRIST_PITCH, 145);

    // Move to starting position (max reach)
    float shoulder_angle, elbow_angle;
    if (calculate_2d_ik(current_x, current_z, &shoulder_angle, &elbow_angle)) {
        int moving_nums[] = {SHOULDER, ELBOW};
        int target_angles[] = {(int)shoulder_angle, (int)elbow_angle};
        move_servos_coordinated(moving_nums, target_angles, 2, 1500);
    }

    delay(1000);
}

void loop() {
    static unsigned long last_print_time = 0;

    // Read joystick (Nano ADC is 10-bit: 0–1023, center ~512)
    int joy_x_raw = analogRead(JOY_X_PIN);
    int joy_y_raw = analogRead(JOY_Y_PIN);

    // Calculate offset from center (10-bit)
    int dead_zone = 75;  // Scaled from 300/4096 for 10-bit
    int offset_x = joy_x_raw - 512;
    int offset_y = joy_y_raw - 512;

    if (abs(offset_x) < dead_zone) offset_x = 0;
    if (abs(offset_y) < dead_zone) offset_y = 0;

    // Convert to movement speed (mm per update)
    float speed = 15.0;
    float delta_x = (offset_x / 512.0) * speed;
    float delta_z = (offset_y / 512.0) * speed;

    float shoulder_angle, elbow_angle;

    if (delta_x != 0 || delta_z != 0) {
        float new_x = current_x + delta_x;
        float new_z = current_z + delta_z;

        if (calculate_2d_ik(new_x, new_z, &shoulder_angle, &elbow_angle)) {
            current_x = new_x;
            current_z = new_z;

            int moving_nums[] = {SHOULDER, ELBOW};
            int target_angles[] = {(int)shoulder_angle, (int)elbow_angle};
            move_servos_coordinated(moving_nums, target_angles, 2, 200);
        } else {
            // Slide along boundary
            float distance = sqrt(new_x * new_x + new_z * new_z);

            if (distance > (LINK1 + LINK2)) {
                float current_angle = atan2(current_z, current_x);
                float cross = current_x * delta_z - current_z * delta_x;
                float angle_delta = (cross > 0 ? 1 : -1) * (sqrt(delta_x * delta_x + delta_z * delta_z) / (LINK1 + LINK2));

                float new_angle = current_angle + angle_delta;
                float boundary_x = (LINK1 + LINK2) * cos(new_angle);
                float boundary_z = (LINK1 + LINK2) * sin(new_angle);

                if (calculate_2d_ik(boundary_x, boundary_z, &shoulder_angle, &elbow_angle)) {
                    current_x = boundary_x;
                    current_z = boundary_z;

                    int moving_nums[] = {SHOULDER, ELBOW};
                    int target_angles[] = {(int)shoulder_angle, (int)elbow_angle};
                    move_servos_coordinated(moving_nums, target_angles, 2, 200);
                }
            }
        }
    }

    // Read second joystick for wrist pitch
    int joy2_y_raw = analogRead(JOY2_Y_PIN);
    int offset_pitch = joy2_y_raw - 512;
    if (abs(offset_pitch) < dead_zone) offset_pitch = 0;

    if (offset_pitch != 0) {
        wrist_pitch_angle = constrain(wrist_pitch_angle + (offset_pitch > 0 ? 1 : -1), 0, 180);
        servos[WRIST_PITCH].writeMicroseconds(angle_to_us(WRIST_PITCH, wrist_pitch_angle));
        current_positions[WRIST_PITCH] = angle_to_us(WRIST_PITCH, wrist_pitch_angle);
    }

    // Print position once per second
    unsigned long current_time = millis();
    if (current_time - last_print_time >= 1000) {
        Serial.print("Current position: Z=");
        Serial.print(current_z, 1);
        Serial.print(" mm, X=");
        Serial.print(current_x, 1);
        Serial.println(" mm");
        last_print_time = current_time;
    }

    delay(50);  // 20Hz update rate
}
