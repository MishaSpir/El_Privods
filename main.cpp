# include <libopencm3/stm32/rcc.h> //rcc.h - reset and clock control
# include <libopencm3/stm32/gpio.h> //inputs outputs
# include <libopencm3/stm32/timer.h>
# include <libopencm3/cm3/nvic.h>

int16_t turn;
// uint16_t degrees;

int main() {
    rcc_periph_clock_enable(RCC_GPIOD);
    gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT,GPIO_PUPD_NONE ,GPIO15|GPIO14|GPIO13|GPIO12);
    rcc_periph_clock_enable(RCC_TIM6);
    timer_set_prescaler(TIM6,160-1);
    timer_set_period(TIM6,200-1);
    timer_enable_counter(TIM6);
    timer_enable_irq(TIM6,TIM_DIER_UIE);//разрешение прерывания по update
    nvic_enable_irq(NVIC_TIM6_DAC_IRQ);//разерешили прохождения запросов в ЦПУ

  

    turn = (4096/8);

    while(true){

    }

  

}

  

uint8_t led_num = 0b00000001;

//функция-обработчик прерывания
void tim6_dac_isr(){
    if(turn>0){
        timer_clear_flag(TIM6,TIM_SR_UIF);  
            if(led_num >=16){led_num = 0b00000001;}

        switch(led_num){
            case 0b00000001:  
                gpio_set(GPIOD,GPIO15);
                gpio_clear(GPIOD,GPIO14);
                gpio_clear(GPIOD,GPIO13);
                gpio_clear(GPIOD,GPIO12);
            break;
            case 0b00000010:
                gpio_clear(GPIOD,GPIO15);
                gpio_set(GPIOD,GPIO14);
                gpio_clear(GPIOD,GPIO13);
                gpio_clear(GPIOD,GPIO12);

            break;
            case 0b00000100:
                gpio_clear(GPIOD,GPIO15);
                gpio_clear(GPIOD,GPIO14);
                gpio_set(GPIOD,GPIO13);
                gpio_clear(GPIOD,GPIO12);

            break;
            case 0b00001000:
                gpio_clear(GPIOD,GPIO15);
                gpio_clear(GPIOD,GPIO14);
                gpio_clear(GPIOD,GPIO13);
                gpio_set(GPIOD,GPIO12);

            break;
        }
        turn--;

      
      
    
// if(led_num == 1){

// gpio_set(GPIOD,GPIO15);

// }else{

// gpio_clear(GPIOD,GPIO15);

// }

led_num *=2;
    }else{
gpio_clear(GPIOD,GPIO15);
gpio_clear(GPIOD,GPIO14);
gpio_clear(GPIOD,GPIO13);
gpio_clear(GPIOD,GPIO12);
    }
    
}