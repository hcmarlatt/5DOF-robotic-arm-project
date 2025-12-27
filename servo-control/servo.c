#include "pico/stdlib.h"
#include "hardware/pwm.h"

int main(){
    // Define constants
    const uint SERVO_PIN = 15;
    const uint LED_PIN = 16;
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // setup section
    // set pwm frequency to 50hz  
    uint slice_num = pwm_gpio_to_slice_num(SERVO_PIN);

    // Now we need to configure this slice for 50Hz
    // I'll give you the numbers, but let's understand them:
    float divisor = 64.0; 
    int wrap_value = 39062;
    // YOUR TASK: Add these lines in the right order
    // 1. Make pin 15 output PWM
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    // 2. Set clock divider to 64
    pwm_set_clkdiv(slice_num, divisor);
    // 3. Set wrap to 39062 (this makes 20ms cycles)
    pwm_set_wrap(slice_num, wrap_value);
    // 4. Enable the PWM
    pwm_set_enabled(slice_num, true);
    
    int channel = pwm_gpio_to_channel(SERVO_PIN);
    int pulse_0 = 1953;
    int pulse_45 = 2929;
    int pulse_90 = 3906;
    int pulse_4500 = 4500;
    int pulse_5000 = 5000;

    while (true){

        // flash led at start of loop
        gpio_put(LED_PIN, 1);
        sleep_ms(500);
        gpio_put(LED_PIN, 0);
        sleep_ms(500);

        // move servo to 0* 1000us pulse
        pwm_set_chan_level(slice_num, channel, pulse_0);
        sleep_ms(1000);
        // move servo to 45*
        pwm_set_chan_level(slice_num, channel, pulse_45);
        sleep_ms(1000);
        // move servo to 90* 1500us pulse
        pwm_set_chan_level(slice_num, channel, pulse_90);
        sleep_ms(1000);
        // move to 180* 2000us pulse
        pwm_set_chan_level(slice_num, channel, pulse_4500);
        sleep_ms(1000);

        pwm_set_chan_level(slice_num, channel, pulse_5000);
        sleep_ms(1000);
    }
    return 0;
}