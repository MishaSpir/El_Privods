

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>



void pwm_setup(void) {


    rcc_periph_clock_enable(RCC_GPIOE);
    // AF - шта? это alternate function
    gpio_mode_setup(GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11);


    gpio_set_af(GPIOE, GPIO_AF1, GPIO11);

    rcc_periph_clock_enable(RCC_TIM1);
    timer_set_prescaler(TIM1, 14-1); //
    timer_set_period(TIM1, 1000-1); // период счета (число-1)

    timer_set_oc_mode(TIM1, TIM_OC2, TIM_OCM_PWM1); // PWM1 включается здесь. Изначально здесь стоял TOGGLE
    timer_set_oc_value(TIM1, TIM_OC2, 700);

    timer_enable_oc_output(TIM1, TIM_OC2); // разрешаем выход
    timer_enable_break_main_output(TIM1);

    timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);


    timer_enable_counter(TIM1);


}


static void adc_setup(void)
{
    //ADC
    rcc_periph_clock_enable(RCC_ADC1);
    rcc_periph_clock_enable(RCC_GPIOA);
    //ADC
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);
    adc_power_off(ADC1);
    // adc_set_clk_prescale(ADC1, ADC_CCR_CKMODE_DIV2);
    adc_set_single_conversion_mode(ADC1);
    adc_disable_external_trigger_regular(ADC1);
    adc_set_right_aligned(ADC1);
    /* We want to read the temperature sensor, so we have to enable it. */
    // adc_enable_temperature_sensor();
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_3CYC);
    uint8_t channel_array[] = { 1 }; /* ADC1_IN1 (PA0) */
    adc_set_regular_sequence(ADC1, 1, channel_array);
    // adc_set_resolution(ADC1, ADC_CFGR1_RES_12_BIT);
    adc_power_on(ADC1);

    /* Wait for ADC starting up. */
    int i;
    for (i = 0; i < 800000; i++)
        __asm__("nop");

}

static void usart_setup(void)
{
    /* Enable clocks for GPIO port A (for GPIO_USART2_TX) and USART2. */
    rcc_periph_clock_enable(RCC_USART2);
    rcc_periph_clock_enable(RCC_GPIOA);

    /* Setup GPIO pin GPIO_USART2_TX/GPIO9 on GPIO port A for transmit. */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO2| GPIO3);

    /* Setup UART parameters. */
    usart_set_baudrate(USART2, 115200);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

    /* Finally enable the USART. */
    usart_enable(USART2);
    // хуй.execute(по лбу) // Активация важного элемента программы

}

static void gpio_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOE);
    gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO11);
}

static void my_usart_print_int(uint32_t usart, int16_t value)
{
    int8_t i;
    int8_t nr_digits = 0;
    char buffer[25];

    if (value < 0) {
        usart_send_blocking(usart, '-');
        value = value * -1;
    }

    if (value == 0) {
        usart_send_blocking(usart, '0');
    }

    while (value > 0) {
        buffer[nr_digits++] = "0123456789"[value % 10];
        value /= 10;
    }

    for (i = nr_digits-1; i >= 0; i--) {
        usart_send_blocking(usart, buffer[i]);
    }

    usart_send_blocking(usart, '\r');
    usart_send_blocking(usart, '\n');
}

static void clock_setup(void)
{
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
}


float pwm_duty =0;
int print_val;

int main(void)
{
    // rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

    float temp;
    // uint32_t leds;
    clock_setup();
    gpio_setup();
    adc_setup();
    usart_setup();
    pwm_setup();
    //65280
    while (1) {
        // leds = temp*255/16;
        adc_start_conversion_regular(ADC1);
        while (!(adc_eoc(ADC1)));
        temp=float(adc_read_regular(ADC1));

        // my_usart_print_int(USART2, leds);

        // if(temp>=4080){pwm_duty=100;}
        pwm_duty = ((30.0/4095.0)*temp) +70.0;
        if(pwm_duty>99){pwm_duty = 100.0;}
        pwm_duty *= 10;
        print_val = int(pwm_duty);
        my_usart_print_int(USART2, print_val);
        timer_set_oc_value(TIM1, TIM_OC2, print_val);


    }

    return 0;
}
