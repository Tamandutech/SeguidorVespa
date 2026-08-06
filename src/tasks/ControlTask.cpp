#include "ControlTask.hpp"

#include "context/GlobalData.hpp"
#include "drivers/EncoderDriver/EncoderDriver.hpp"
#include "drivers/IRSensorDriver/IRSensorDriver.hpp"
#include "drivers/LedRgbDriver/LedRgbDriver.hpp"
#include "drivers/MotorDriver/MotorDriver.hpp"
#include "drivers/VacuumDriver/VacuumDriver.hpp"
#include "env.hpp"
#include "esp_log.h"
#include "tasks/BluetoothTask.hpp"
#include "tasks/controllers/PathController.hpp"

namespace {
const char *TAG = "ControlTask";

constexpr uint8_t kLineSensorCount = 12;
constexpr uint8_t kSideSensorCount = 4;
constexpr TickType_t kIdleLedPeriodTicks = pdMS_TO_TICKS(250);

const uint8_t kMuxDigitalAddress[]  = GPIO_MULTIPLEXER_DIGITAL_ADDRESS;
const uint8_t kLineSensorMuxIndex[] = GPIO_MULTIPLEXER_LINE_SENSORS_INDEX;
const uint8_t kSideSensorMuxIndex[] = GPIO_MULTIPLEXER_SIDE_SENSORS_INDEX;
} // namespace

ControlTask::ControlTask(StateMachineTask *stateMachine)
    : stateMachine_(stateMachine), taskHandle_(nullptr), motorDriver_(nullptr),
      vacuumDriver_(nullptr), irSensorDriver_(nullptr), encoderLeft_(nullptr),
      encoderRight_(nullptr), pathController_(nullptr), ledRgbDriver_(nullptr),
      lineSensorValues_{}, sideSensorValues_{}, lastState_(RobotState::IDLE),
      mapPointIndex_(0), finishLineMm_(0), properlyCalibrated_(false),
      alternateLedColorFlag_(false), lastIdleLedUpdate_(0) {}

ControlTask::~ControlTask() {
  if(taskHandle_ != nullptr) {
    vTaskDelete(taskHandle_);
    taskHandle_ = nullptr;
  }
  delete pathController_;
  delete ledRgbDriver_;
  delete irSensorDriver_;
  delete encoderRight_;
  delete encoderLeft_;
  delete vacuumDriver_;
  delete motorDriver_;
}

bool ControlTask::start(uint32_t stackSizeWords, UBaseType_t priority,
                        BaseType_t coreId) {
  if(taskHandle_ != nullptr || stateMachine_ == nullptr) {
    return false;
  }

  return xTaskCreatePinnedToCore(&ControlTask::taskEntry, "control_task",
                                 stackSizeWords, this, priority, &taskHandle_,
                                 coreId) == pdPASS;
}

void ControlTask::taskEntry(void *param) {
  auto *self = static_cast<ControlTask *>(param);
  self->run();
}

void ControlTask::initHardware() {
  // Defaults when NV storage has no PID/mapping values yet.
  if(globalData.parametersConfig.pidKp == 0.0F &&
     globalData.parametersConfig.pidKd == 0.0F) {
    globalData.parametersConfig.pidKp = 0.017F;
    globalData.parametersConfig.pidKd = 0.068F;
  }
  if(globalData.parametersConfig.mappingMotorPWM == 0) {
    globalData.parametersConfig.mappingMotorPWM = MOTOR_MAPPING_PWM;
  }

  const MotorPins motorPins = {.gpioDirectionA = GPIO_DIRECTION_A,
                               .gpioDirectionB = GPIO_DIRECTION_B,
                               .gpioPWMA       = GPIO_PWM_A,
                               .gpioPWMB       = GPIO_PWM_B};
  motorDriver_              = new MotorDriver(motorPins);

  const VacuumPins vacuumPins = {.gpioPWM = GPIO_PWM_VACUUM};
  vacuumDriver_               = new VacuumDriver(vacuumPins);

  const IRSensorParamSchema irParam = {
      .pins =
          {
              .gpioMultiplexerDigitalAddress = kMuxDigitalAddress,
              .gpioMultiplexerAnalogInput    = GPIO_MULTIPLEXER_ANALOG_INPUT,
          },
      .lineSensorsCount            = kLineSensorCount,
      .lineSensorsMultiplexerIndex = kLineSensorMuxIndex,
      .sideSensorsCount            = kSideSensorCount,
      .sideSensorsMultiplexerIndex = kSideSensorMuxIndex,
      .multiplexerPinCount         = 4,
  };
  irSensorDriver_ = new IRSensorDriver(irParam);

  encoderLeft_ = new EncoderDriver();
  encoderLeft_->attachFullQuad(GPIO_ENCODER_LEFT_A, GPIO_ENCODER_LEFT_B);
  encoderRight_ = new EncoderDriver();
  encoderRight_->attachFullQuad(GPIO_ENCODER_RIGHT_A, GPIO_ENCODER_RIGHT_B);

  const LedRgbPins ledPins = {.gpioData = static_cast<gpio_num_t>(GPIO_LED_DEBUG),
                              .numLeds  = NUM_LEDS_DEBUG};
  ledRgbDriver_            = new LedRgbDriver(ledPins);

  rebuildPathController();
}

void ControlTask::calibrateSensors() {
  if(ledRgbDriver_ != nullptr) {
    ledRgbDriver_->setColor(0, LED_COLOR_YELLOW);
    ledRgbDriver_->refresh();
  }

  ESP_LOGI(TAG, "Calibrando os sensores...");
  for(int i = 0; i < 50; i++) {
    irSensorDriver_->calibrate();
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  QTRSensors::CalibrationData *calibrationData =
      &irSensorDriver_->qtrSensors().calibrationOn;
  const uint8_t sensorCount = irSensorDriver_->getSensorCount();

  if(globalData.parametersConfig.hardcodedCalibration &&
     calibrationData->initialized && calibrationData->minimum != nullptr &&
     calibrationData->maximum != nullptr) {
    constexpr uint16_t kHardcodedIrMin = 200;
    constexpr uint16_t kHardcodedIrMax = 3600;
    for(uint8_t i = 0; i < sensorCount; i++) {
      calibrationData->minimum[i] = kHardcodedIrMin;
      calibrationData->maximum[i] = kHardcodedIrMax;
    }
  }

  bool calibrationOk = calibrationData->initialized &&
                       calibrationData->minimum != nullptr &&
                       calibrationData->maximum != nullptr;

  if(calibrationOk) {
    for(uint8_t i = 0; i < sensorCount; i++) {
      const uint16_t lo   = calibrationData->minimum[i];
      const uint16_t hi   = calibrationData->maximum[i];
      const int      diff = static_cast<int>(hi) - static_cast<int>(lo);
      ESP_LOGI(TAG,
               "Calibration sensor %u: min=%u max=%u diff=%d initialized=%d",
               static_cast<unsigned>(i), static_cast<unsigned>(lo),
               static_cast<unsigned>(hi), diff, calibrationData->initialized);
      if(diff < 50) {
        calibrationOk = false;
        (void)bluetoothPushMessage(
            "Error: Sensor %u calibrado incorretamente (range: %u, "
            "%u, difference: %d, initialized: %d)",
            static_cast<unsigned>(i), static_cast<unsigned>(lo),
            static_cast<unsigned>(hi), diff, calibrationData->initialized);
      }
    }
  }

  properlyCalibrated_ = calibrationOk;

  if(!calibrationOk) {
    ESP_LOGW(TAG, "Sensores nao calibrados corretamente");
    if(!calibrationData->initialized || calibrationData->minimum == nullptr ||
       calibrationData->maximum == nullptr) {
      (void)bluetoothPushMessage(
          "Error: Sensores calibrados incorretamente "
          "(initialized: %d, pointers ok: %d)",
          calibrationData->initialized,
          calibrationData->minimum != nullptr &&
              calibrationData->maximum != nullptr);
    }
  } else {
    ESP_LOGI(TAG, "Sensores calibrados");
    (void)bluetoothPushMessage(
        "Sensores calibrados corretamente (%u sensores verificados, "
        "initialized: %d)",
        static_cast<unsigned>(sensorCount), calibrationData->initialized);
  }
}

void ControlTask::rebuildPathController() {
  delete pathController_;
  pathController_ = nullptr;

  const PathControllerConstants pidConstants = {
      .kP = globalData.parametersConfig.pidKp,
      .kI = globalData.parametersConfig.pidKi,
      .kD = globalData.parametersConfig.pidKd,
  };
  PathControllerParamSchema pathParam = {
      .constants      = pidConstants,
      .sensorQuantity = kLineSensorCount,
      .sensorValues   = lineSensorValues_,
      .maxAngle       = 45.0F,
      .radiusSensor   = 100,
      .sensorToCenter = 50,
  };
  pathController_ = new PathController(pathParam);
}

void ControlTask::stopActuators() {
  if(motorDriver_ != nullptr) {
    motorDriver_->pwmOutput(0, 0);
  }
  if(vacuumDriver_ != nullptr) {
    vacuumDriver_->pwmOutput(0);
  }
}

void ControlTask::onEnterMotionState() {
  rebuildPathController();
  mapPointIndex_          = 0;
  finishLineMm_           = 0;
  alternateLedColorFlag_  = false;
  if(!globalData.mapData.empty()) {
    finishLineMm_ = globalData.mapData.back().encoderMilimeters;
  }
  if(encoderLeft_ != nullptr) {
    encoderLeft_->clearCount();
  }
  if(encoderRight_ != nullptr) {
    encoderRight_->clearCount();
  }
  if(ledRgbDriver_ != nullptr) {
    // RUNNING/MAPPING: LED 0 verde ao entrar (feedback de movimento).
    ledRgbDriver_->setColor(0, LED_COLOR_GREEN);
    ledRgbDriver_->refresh();
  }
}

void ControlTask::updateIdleLeds() {
  if(ledRgbDriver_ == nullptr) {
    return;
  }

  const TickType_t now = xTaskGetTickCount();
  if((now - lastIdleLedUpdate_) < kIdleLedPeriodTicks) {
    return;
  }
  lastIdleLedUpdate_ = now;

  if(properlyCalibrated_) {
    if(alternateLedColorFlag_) {
      ledRgbDriver_->setColor(0, LED_COLOR_PURPLE, 0.5f);
      alternateLedColorFlag_ = false;
    } else {
      ledRgbDriver_->setColor(0, LED_COLOR_WHITE, 0.5f);
      alternateLedColorFlag_ = true;
    }
    ledRgbDriver_->setColor(1, LED_COLOR_BLUE);
    ledRgbDriver_->setColor(2, LED_COLOR_BLUE);
    ledRgbDriver_->setColor(3, LED_COLOR_BLUE);
  } else {
    ledRgbDriver_->setColor(0, LED_COLOR_RED);
    ledRgbDriver_->setColor(1, LED_COLOR_RED);
    ledRgbDriver_->setColor(2, LED_COLOR_RED);
    ledRgbDriver_->setColor(3, LED_COLOR_RED);
  }
  ledRgbDriver_->refresh();
}

int32_t ControlTask::encoderAverageMm() const {
  if(encoderLeft_ == nullptr || encoderRight_ == nullptr) {
    return 0;
  }
  const int32_t pulses =
      (encoderLeft_->getCount() + encoderRight_->getCount()) / 2;
  return (pulses * WHEEL_CIRCUMFERENCE / ENCODER_PULSES_PER_ROTATION) * -1;
}

void ControlTask::tickStopped() {
  stopActuators();
  updateIdleLeds();
}

void ControlTask::tickRunning() {
  const int32_t distanceMm = encoderAverageMm();

  if(finishLineMm_ > 0 && distanceMm > finishLineMm_) {
    stopActuators();
    if(ledRgbDriver_ != nullptr) {
      ledRgbDriver_->setColor(0, LED_COLOR_RED);
      ledRgbDriver_->refresh();
    }
    if(stateMachine_ != nullptr) {
      const Event stopEvent{EventType::STOP};
      (void)stateMachine_->postEvent(stopEvent, 0);
    }
    return;
  }

  irSensorDriver_->readCalibrated(lineSensorValues_, sideSensorValues_);

  if(!globalData.mapData.empty() &&
     distanceMm > globalData.mapData[mapPointIndex_].encoderMilimeters &&
     (mapPointIndex_ + 1U) < globalData.mapData.size()) {
    mapPointIndex_++;
    alternateLedColorFlag_ = !alternateLedColorFlag_;
    if(ledRgbDriver_ != nullptr) {
      ledRgbDriver_->setColor(0, alternateLedColorFlag_ ? LED_COLOR_ORANGE
                                                        : LED_COLOR_CYAN);
      ledRgbDriver_->refresh();
    }
  }

  const float   pathPid = pathController_->getPID();
  const int32_t basePwm = globalData.mapData.empty()
                            ? 0
                            : globalData.mapData[mapPointIndex_].baseMotorPWM;

  motorDriver_->pwmOutput(basePwm + static_cast<int32_t>(pathPid),
                          basePwm - static_cast<int32_t>(pathPid));
  vacuumDriver_->pwmOutput(globalData.parametersConfig.vacuumPWM);
}

void ControlTask::tickMapping() {
  irSensorDriver_->readCalibrated(lineSensorValues_, sideSensorValues_);

  const float pathPid = pathController_->getPID();
  const float base =
      static_cast<float>(globalData.parametersConfig.mappingMotorPWM);
  motorDriver_->pwmOutput(static_cast<int32_t>(base + pathPid),
                          static_cast<int32_t>(base - pathPid));
  vacuumDriver_->pwmOutput(globalData.parametersConfig.vacuumPWM);
}

void ControlTask::run() {
  initHardware();
  stopActuators();
  calibrateSensors();
  lastIdleLedUpdate_ = 0;
  ESP_LOGI(TAG, "control loop ready");

  for(;;) {
    const RobotState state = gRobotState;

    if(state != lastState_) {
      const bool wasMoving = lastState_ == RobotState::RUNNING ||
                             lastState_ == RobotState::MAPPING;
      const bool isMoving =
          state == RobotState::RUNNING || state == RobotState::MAPPING;
      if(isMoving && !wasMoving) {
        onEnterMotionState();
      } else if(!isMoving && wasMoving) {
        stopActuators();
        lastIdleLedUpdate_ = 0;
      }
      lastState_ = state;
    }

    switch(state) {
    case RobotState::RUNNING: tickRunning(); break;
    case RobotState::MAPPING: tickMapping(); break;
    case RobotState::IDLE:
    case RobotState::CALIBRATING:
      // Calibração de sensores já ocorreu em calibrateSensors().
      tickStopped();
      break;
    }

    // Período fixo ~1 ms; sem printf / fila bloqueante no loop crítico.
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
