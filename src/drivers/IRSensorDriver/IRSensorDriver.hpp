#ifndef IRSENSOR_DRIVER_HPP
#define IRSENSOR_DRIVER_HPP

#include "esp_attr.h"
#include "esp_system.h"
#include <bitset>

#include "driver/gpio.h"

#include "QTRSensors/QTRSensors.h"

struct IRSensorPins {
  // defina para cada controlador
  const uint8_t *gpioMultiplexerDigitalAddress;
  const uint8_t  gpioMultiplexerAnalogInput;
};

struct IRSensorParamSchema {
  const IRSensorPins pins;
  const uint8_t      frontSensorsCount;
  const uint8_t      sideSensorsCount;
  const uint8_t      multiplexerPinCount;
};

class IRSensorDriver {
public:
  IRSensorDriver(IRSensorParamSchema param);

  void read(uint16_t *sensor_values);
  void readCalibrated(uint16_t *sensor_values);
  void calibrate();

private:
  IRSensorPins pins_;
  uint8_t      frontSensorsCount_;
  uint8_t      sideSensorsCount_;
  uint8_t      multiplexerPinCount_;

  QTRSensors sensorsArray_;

  void setMultiplexerDigitalAddress(std::bitset<4> address);
};

IRSensorDriver::IRSensorDriver(IRSensorParamSchema param)
    : pins_(param.pins), frontSensorsCount_(param.frontSensorsCount),
      sideSensorsCount_(param.sideSensorsCount),
      multiplexerPinCount_(param.multiplexerPinCount) {

  // Configure GPIO_MULTIPLEXER_DIGITAL_ADDRESS pins as outputs
  gpio_config_t io_conf = {};
  io_conf.intr_type     = GPIO_INTR_DISABLE;
  io_conf.mode          = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask  = 0;
  io_conf.pull_down_en  = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en    = GPIO_PULLUP_DISABLE;

  // Set pin bit mask for all multiplexer digital address pins
  for(int i = 0; i < multiplexerPinCount_; i++) {
    io_conf.pin_bit_mask |= (1ULL << pins_.gpioMultiplexerDigitalAddress[i]);
  }

  gpio_config(&io_conf);

  sensorsArray_.setTypeMultiplexer();
  sensorsArray_.setSensorPins(param.pins.gpioMultiplexerAnalogInput,
                              param.frontSensorsCount + param.sideSensorsCount,
                              param.pins.gpioMultiplexerDigitalAddress,
                              param.multiplexerPinCount);
}

void IRSensorDriver::setMultiplexerDigitalAddress(std::bitset<4> address) {
  // Função que controla qual sensor está conectado ao ESP32
  // Recebe um vetor com a sequencia de 4 bits de 0000 a 1111 e ajusta o nível
  // lógico das portas digitais de acordo 0010
  for(int i = 0; i < address.size(); i++) {
    if(address[i]) {
      gpio_set_level((gpio_num_t)pins_.gpioMultiplexerDigitalAddress[i], 1);
    } // se bit=1, nível lógico alto
    else {
      gpio_set_level((gpio_num_t)pins_.gpioMultiplexerDigitalAddress[i], 0);
    } // se bit=0, nível lógico baixo
  }
}

void IRSensorDriver::read(uint16_t *sensor_values) {
  sensorsArray_.read(sensor_values);
}

void IRSensorDriver::calibrate() { sensorsArray_.calibrate(); }

void IRSensorDriver::readCalibrated(uint16_t *sensor_values) {
  sensorsArray_.readCalibrated(sensor_values);
}

#endif // IRSENSOR_DRIVER_HPP