#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include <stdio.h>

int main() {
    const uint LED1_PIN = 16;
    const uint LED2_PIN = 17;
    const uint VRx = 26;
    const uint VRy = 27;

    stdio_init_all();
    adc_init();
    adc_gpio_init(VRx);
    adc_gpio_init(VRy);
    
    // Setup PWM on LED pin
    gpio_set_function(LED1_PIN, GPIO_FUNC_PWM);
    gpio_set_function(LED2_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(LED1_PIN);
    pwm_set_wrap(slice_num, 4095);  // Match ADC range for easy mapping
    pwm_set_enabled(slice_num, true);
    
    while (true) {
        adc_select_input(0);
        uint16_t x = adc_read();
        adc_select_input(1);
        uint16_t y = adc_read();
        
        pwm_set_gpio_level(LED1_PIN, x);  // Joystick position = brightness
        pwm_set_gpio_level(LED2_PIN, y);
        
        printf("X: %d, Brightness: %d ", x, x);
        printf("Y: %d, Brightness: %d\n",y, y );
        sleep_ms(200);
    }
}