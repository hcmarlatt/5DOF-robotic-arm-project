#include "pico/stdlib.h"
#include "hardware/pwm.h"

int main(){
    // Define constants
    const uint SERVO1_PIN = 15;
    const uint SERVO2_PIN = 14;
    const uint LED_PIN = 16;
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // **SERVO CALIBRATION VALUES** 

    // SG90:
        // SERVO 1+2:
        // MIN - 0*: 700
        // MAX - 180*: 4550
        // TRUE MAX~190*: 4750

        // SERVO 3:
        // MIN 0*: 850
        // MAX 180*: 4650
        // TRUE MAX: 5025
    
    // MG995
        // SERVO 1+2+3
        // TRUE MIN: 685
        // 0*: 750
        // 180*: 4600
        // TRUE MAX: 5160 (#3 has MAX: ~5280)




    // setup section
    // set pwm frequency to 50hz  
    uint slice_num = pwm_gpio_to_slice_num(SERVO1_PIN);
    float divisor = 64.0; 
    int wrap_value = 39062;
    // Make pin 15 output PWM
    gpio_set_function(SERVO1_PIN, GPIO_FUNC_PWM);
    // 2nd servo output
    gpio_set_function(SERVO2_PIN, GPIO_FUNC_PWM);
    // Set clock divider to 64
    pwm_set_clkdiv(slice_num, divisor);
    // Set wrap to 39062 (this makes 20ms cycles)
    pwm_set_wrap(slice_num, wrap_value);
    // Enable the PWM
    pwm_set_enabled(slice_num, true);
    
    int channel1 = pwm_gpio_to_channel(SERVO1_PIN);
    int channel2 = pwm_gpio_to_channel(SERVO2_PIN);

    while (true){

        // get pulse from user
        int servo_num, pulse;
        printf("Pulse: ");
        scanf("%d: %d", &servo_num, &pulse);
        if (servo_num == 1){
            pwm_set_chan_level(slice_num, channel1, pulse);
            printf("sent pulse to servo 1");
        }
        else if (servo_num == 2) {
            pwm_set_chan_level(slice_num, channel2, pulse);
            printf("sent pulse to servo 2");
        }
        
        gpio_put(LED_PIN, 1);
        sleep_ms(2000);
        printf("%d sent\n", pulse);
        printf("Read: servo %d, pulse %d\n", servo_num, pulse);
        sleep_ms(1000);
        gpio_put(LED_PIN, 0);
        sleep_ms(100);
    }
    return 0;
}