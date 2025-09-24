#ifndef VACUUM_DRIVER_HPP
#define VACUUM_DRIVER_HPP

#include "esp_attr.h"
#include "esp_system.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

#include "context/GlobalData.hpp"

#define LEDC_TIMER     LEDC_TIMER_0        // Timer do LEDC utilizado
#define LEDC_MODE      LEDC_LOW_SPEED_MODE // Modo de velocidade do LEDC
#define PWM_PIN        LEDC_CHANNEL_0      // Canal do LEDC utilizado
#define LEDC_DUTY_RES  LEDC_TIMER_13_BIT   // Resolução do PWM
#define LEDC_FREQUENCY 2000                // Frequência em Hertz do sinal PWM


struct VacuumPins {
  uint8_t gpioPWM;
};

class VacuumDriver {
public:
  VacuumDriver(VacuumPins pin);

  void pwmOutput(int32_t value);

private:
  const char *tag = "VacuumDriver";
  VacuumPins  pin_;

  void initMotorPWM(gpio_num_t pin, ledc_channel_t channel);
};

void VacuumDriver::initMotorPWM(gpio_num_t pin, ledc_channel_t channel) {
  // Prepara e aplica a configuração do canal do LEDC
  ledc_channel_config_t ledc_channel = {.gpio_num   = pin,
                                        .speed_mode = LEDC_MODE,
                                        .channel    = channel,
                                        .intr_type  = LEDC_INTR_DISABLE,
                                        .timer_sel  = LEDC_TIMER,
                                        .duty       = 0,
                                        .hpoint     = 0};
  ledc_channel_config(&ledc_channel);
}

VacuumDriver::VacuumDriver(VacuumPins pin) : pin_(pin) {
  // Configure LEDC timer once
  ledc_timer_config_t ledc_timer = {
      .speed_mode      = LEDC_MODE,
      .duty_resolution = LEDC_DUTY_RES,
      .timer_num       = LEDC_TIMER,
      .freq_hz         = LEDC_FREQUENCY, // Frequência de 2Khz
      .clk_cfg         = LEDC_AUTO_CLK   // Configuração da fonte de clock
  };
  ledc_timer_config(&ledc_timer);

  // Install fade function once
  ledc_fade_func_install(0);

  ESP_LOGD(tag, "Init PWM Vacuum Cleaner");
  initMotorPWM((gpio_num_t)pin_.gpioPWM, PWM_PIN);

  ESP_LOGD(tag, "PWM Initialized");
}

void VacuumDriver::pwmOutput(int32_t value) {
  ledc_set_duty(LEDC_MODE, PWM_PIN, value);
  ledc_update_duty(LEDC_MODE, PWM_PIN);
}


#endif // VACUUM_DRIVER_HPP
