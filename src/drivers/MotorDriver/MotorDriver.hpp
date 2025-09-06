#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include "esp_attr.h"
#include "esp_system.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

#include "context/GlobalData.hpp"

#define LEDC_TIMER     LEDC_TIMER_1        // Timer do LEDC utilizado
#define LEDC_MODE      LEDC_LOW_SPEED_MODE // Modo de velocidade do LEDC
#define PWM_A_PIN      LEDC_CHANNEL_2      // Canal do LEDC utilizado
#define PWM_B_PIN      LEDC_CHANNEL_3      // Canal do LEDC utilizado
#define LEDC_DUTY_RES  LEDC_TIMER_13_BIT   // Resolução do PWM
#define LEDC_FREQUENCY 2000                // Frequência em Hertz do sinal PWM

static const char *TAG = "MotorDriver";

struct MotorPins {
  uint8_t gpioDirectionA;
  uint8_t gpioDirectionB;
  uint8_t gpioPWMA;
  uint8_t gpioPWMB;
};

class MotorDriver {
public:
  MotorDriver(MotorPins pin);

  void pwmOutput(int32_t valueA, int32_t valueB);

private:
  MotorPins pin_;

  void initMotorPWM(gpio_num_t pin, ledc_channel_t channel);
};

void MotorDriver::initMotorPWM(gpio_num_t pin, ledc_channel_t channel) {
  ledc_timer_config_t ledc_timer;
  ledc_timer.speed_mode      = LEDC_MODE;
  ledc_timer.duty_resolution = LEDC_DUTY_RES;
  ledc_timer.timer_num       = LEDC_TIMER;
  ledc_timer.freq_hz         = LEDC_FREQUENCY; // Frequência de 5Khz
  ledc_timer.clk_cfg         = LEDC_AUTO_CLK;  // Configuração da fonte de clock
  ledc_timer_config(&ledc_timer);

  // Prepara e aplica a configuração do canal do LEDC
  ledc_channel_config_t ledc_channel;
  ledc_channel.gpio_num   = pin;
  ledc_channel.speed_mode = LEDC_MODE;
  ledc_channel.channel    = channel;
  ledc_channel.intr_type  = LEDC_INTR_DISABLE;
  ledc_channel.timer_sel  = LEDC_TIMER;
  ledc_channel.duty       = 0;
  ledc_channel.hpoint     = 0; // Ponto de início do duty cycle
  ledc_channel_config(&ledc_channel);

  ledc_fade_func_install(0);
}

MotorDriver::MotorDriver(MotorPins pin) : pin_(pin) {
  gpio_config_t io_conf;
  io_conf.intr_type = (gpio_int_type_t)GPIO_INTR_DISABLE;
  io_conf.mode      = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask =
      ((1ULL << pin_.gpioDirectionA) | (1ULL << pin_.gpioDirectionB) |
       (1ULL << pin_.gpioPWMA) | (1ULL << pin_.gpioPWMB));
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
  gpio_config(&io_conf);

  ESP_LOGD(TAG, "Init PWM Motor 0");

  initMotorPWM((gpio_num_t)pin_.gpioPWMA, PWM_A_PIN);
  ESP_LOGD(TAG, "Init PWM Motor 1");

  initMotorPWM((gpio_num_t)pin_.gpioPWMB, PWM_B_PIN);
  ESP_LOGD(TAG, "PWM Initialized");
}

void MotorDriver::pwmOutput(int32_t valueA, int32_t valueB) {
  if(valueA > 0) {
    gpio_set_level(static_cast<gpio_num_t>(pin_.gpioDirectionA), 1);
  } else {
    gpio_set_level(static_cast<gpio_num_t>(pin_.gpioDirectionA), 0);
  }
  if(valueB > 0) {
    gpio_set_level(static_cast<gpio_num_t>(pin_.gpioDirectionB), 1);
  } else {
    gpio_set_level(static_cast<gpio_num_t>(pin_.gpioDirectionB), 0);
  }
  ledc_set_duty(LEDC_MODE, PWM_A_PIN, valueA);
  ledc_set_duty(LEDC_MODE, PWM_B_PIN, valueB);
  ledc_update_duty(LEDC_MODE, PWM_A_PIN);
  ledc_update_duty(LEDC_MODE, PWM_B_PIN);
}


#endif // MOTOR_DRIVER_HPP
