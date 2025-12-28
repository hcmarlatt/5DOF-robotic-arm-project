#include "pico/stdlib.h"
#include "hardware/pwm.h"

int main(){
    // Define constants
    const uint SERVO_PIN = 15;
    const uint LED_PIN = 16;
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // **SERVO CALIBRATION VALUES** 
        // SERVO 1+2:
        // MIN - 0*: 700
        // MAX - 180*: 4550
        // TRUE MAX~190*: 4750

        // SERVO 3:
        // MIN 0*: 850
        // MAX 180*: 4650
        // TRUE MAX: 5025

    // setup section
    // set pwm frequency to 50hz  
    uint slice_num = pwm_gpio_to_slice_num(SERVO_PIN);
    float divisor = 64.0; 
    int wrap_value = 39062;
    // Make pin 15 output PWM
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    // Set clock divider to 64
    pwm_set_clkdiv(slice_num, divisor);
    // Set wrap to 39062 (this makes 20ms cycles)
    pwm_set_wrap(slice_num, wrap_value);
    // Enable the PWM
    pwm_set_enabled(slice_num, true);
    
    int channel = pwm_gpio_to_channel(SERVO_PIN);
    int pulse_0 = 1953;
    int pulse_45 = 2929;
    int pulse_90 = 3906;
    int pulse_4500 = 4500;
    int pulse_5000 = 5000;

    while (true){

        // get pulse from user
        int pulse;
        printf("Pulse: ");
        scanf("%d", &pulse);
        pwm_set_chan_level(slice_num, channel, pulse);
        gpio_put(LED_PIN, 1);
        sleep_ms(2000);
        printf("Pulse: %d sent\n", pulse);
        sleep_ms(1000);
        gpio_put(LED_PIN, 0);
        sleep_ms(100);
    }
    return 0;
}