#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"

/* Pin and configuration definitions */

#define CLAW_PIN 8          // GPIO pin connected to the claw servo
#define SERVO_PIN 18         // GPIO pin connected to the arm servo

#define SERVO_MIN_PULSE 500  // Minimum pulse width in microseconds (for 0 degrees)
#define SERVO_MAX_PULSE 2500  // Maximum pulse width in microseconds (for 180 degrees)

#define CLAW_OPEN_PULSE 1800  // Pulse width to open the claw (130 degrees)
#define CLAW_CLOSE_PULSE 2300  // Pulse width to close the claw (165 degrees)

#define JOYSTICK_X_PIN 27  // GPIO pin connected to the joystick X-axis
#define JOYSTICK_Y_PIN 26  // GPIO pin connected to the joystick Y-axis

#define ADC_INPUT_X 1  // ADC input channel for the joystick X-axis
#define ADC_INPUT_Y 0  // ADC input channel for the joystick Y-axis

#define ADC_MIN 19  // Minimum ADC value (experimental)
#define ADC_MAX 4086  // Maximum ADC value (experimental)

#define RED_LED_PIN 13    // GPIO pin for the red LED
#define GREEN_LED_PIN 11  // GPIO pin for the green LED
#define BUTTONA_PIN 5    // GPIO pin for the emergency stop button
#define BUTTONB_PIN 6   // GPIO pin for the claw open/close button

/* Function prototypes*/
long map(long x, long in_min, long in_max, long out_min, long out_max);
void gpio_irq_handler(uint gpio, uint32_t events);


/* Global variables */

volatile bool claw_open = false; // State of the claw (open or closed)
volatile bool emergency_stop = true; // Emergency stop flag


int main()
{
    /* Initialize stdio and GPIO */
    stdio_init_all();

    gpio_init(RED_LED_PIN);
    gpio_set_dir(RED_LED_PIN, GPIO_OUT);
    gpio_put(RED_LED_PIN, 0); // Turn off red LED

    gpio_init(GREEN_LED_PIN);
    gpio_set_dir(GREEN_LED_PIN, GPIO_OUT);
    gpio_put(GREEN_LED_PIN, 1); // Turn on green LED

    gpio_init(BUTTONA_PIN);
    gpio_set_dir(BUTTONA_PIN, GPIO_IN);
    gpio_pull_up(BUTTONA_PIN);

    gpio_init(BUTTONB_PIN);
    gpio_set_dir(BUTTONB_PIN, GPIO_IN);
    gpio_pull_up(BUTTONB_PIN);
    
    /* Set up GPIO interrupts for buttons */
    gpio_set_irq_enabled_with_callback(BUTTONA_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled(BUTTONB_PIN, GPIO_IRQ_EDGE_FALL, true);

    /* Initialize ADC for joystick input */
    adc_init();
    adc_gpio_init(JOYSTICK_X_PIN);
    adc_gpio_init(JOYSTICK_Y_PIN);
    
    /* Configure PWM for servo control */
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    gpio_set_function(CLAW_PIN, GPIO_FUNC_PWM);

    uint servo_slice = pwm_gpio_to_slice_num(SERVO_PIN);
    uint claw_slice = pwm_gpio_to_slice_num(CLAW_PIN);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 125.0f);
    pwm_config_set_wrap(&config, 19999);
    pwm_init(servo_slice, &config, true);
    pwm_init(claw_slice, &config, true);

    /* Set initial positions */
    pwm_set_gpio_level(CLAW_PIN, CLAW_CLOSE_PULSE); // Close the claw


    uint16_t last_joystick_value_x = 0;


    while (true) {

        if(!emergency_stop){        // Only operate if not in emergency stop mode
            
            adc_select_input(ADC_INPUT_X);
            uint16_t joystick_value_x = adc_read();
            
            if(abs(joystick_value_x - last_joystick_value_x) > 20){ // Dead zone to prevent jitter
                last_joystick_value_x = joystick_value_x;
                int pulse_x = map(joystick_value_x, ADC_MIN, ADC_MAX, SERVO_MAX_PULSE, SERVO_MIN_PULSE);
                pwm_set_gpio_level(SERVO_PIN, pulse_x);
            }

            pwm_set_gpio_level(CLAW_PIN, (claw_open ? CLAW_OPEN_PULSE : CLAW_CLOSE_PULSE)); 
            gpio_put(RED_LED_PIN, 0); 
            gpio_put(GREEN_LED_PIN, 1); 


        }else{
            gpio_put(RED_LED_PIN, !gpio_get(RED_LED_PIN)); // Blink red LED
            gpio_put(GREEN_LED_PIN, 0); 
        }
    
        sleep_ms(100);
    }
}


void gpio_irq_handler(uint gpio, uint32_t events) {
    static uint32_t last_interrupt_timeA = 0;
    static uint32_t last_interrupt_timeB = 0;

    uint32_t current_time = us_to_ms(get_absolute_time());

    if (gpio == BUTTONA_PIN && (current_time - last_interrupt_timeA > 300)) { // Debounce time of 300 ms
        last_interrupt_timeA = current_time;
        emergency_stop = !emergency_stop; // Toggle emergency stop state
   
    }else if (gpio == BUTTONB_PIN && (current_time - last_interrupt_timeB > 300)) {
        last_interrupt_timeB = current_time;
        if(!emergency_stop) {
            claw_open = !claw_open; // Toggle claw state
        }
    }
}

/* @brief Maps a number from one range to another.
    * @param x The number to map.
    * @param in_min The lower bound of the input range.
    * @param in_max The upper bound of the input range.
    * @param out_min The lower bound of the output range.
    * @param out_max The upper bound of the output range.
    * @return The mapped value.
 */
long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

