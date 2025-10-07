#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/dac.h>
#include <libopencm3/stm32/timer.h>
#include <math.h>

// Задержка (простая реализация)
void delay(void) {
    for (volatile int i = 0; i < 100000; i++);
    __asm__("nop");
}

int main(void) {
    // Настройка тактирования
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
    
    // Включение тактирования для порта A и DAC
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_DAC);
    
    // Настройка вывода PA4 (DAC_OUT1) как аналоговый
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO4);
    
    // Включение DAC канала 1
    dac_disable(DAC1,DAC_CHANNEL1);
    dac_enable(DAC1,DAC_CHANNEL1);
    int value = 0;
    // Основной цикл
    while (1) {
    // Спадающее напряжение
         for (uint16_t value = 4095; value > 0; value -= 5) {
            dac_load_data_buffer_single(DAC1,value, DAC_ALIGN_RIGHT12, DAC_CHANNEL1);
            delay();
        }

        // Нарастающее напряжение
        for (uint16_t value = 0; value < 4095; value += 5) {
            dac_load_data_buffer_single(DAC1,value, DAC_ALIGN_RIGHT12, DAC_CHANNEL1);
            delay();
        }



    }
    
    return 0;
}
