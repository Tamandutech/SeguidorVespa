#ifndef ROBOT_ENV_HPP
#define ROBOT_ENV_HPP

namespace RobotEnv {

const float ROBOT_WIDTH         = 4;
const float WHEEL_RADIUS        = 4;
const float WHEEL_CIRCUMFERENCE = 4;

const float MAX_SPEED          = 4;
const float MAX_DECELERATION   = 4;
const float MAX_ROTATION_SPEED = 4;

constexpr float EPSILON_TOLERANCE =
    1e-6F; // Tolerância para comparações de ponto flutuante

// Constantes para prevenção de integral windup no PID
constexpr float INTEGRAL_MAX = 1000.0F;  // Valor máximo para o termo integral
constexpr float INTEGRAL_MIN = -1000.0F; // Valor mínimo para o termo integral

const uint8_t GPIO_DIRECTION_A = 37;
const uint8_t GPIO_DIRECTION_B = 9;
const uint8_t GPIO_PWM_A       = 38;
const uint8_t GPIO_PWM_B       = 3;

const uint8_t GPIO_MULTIPLEXER_DIGITAL_ADDRESS[] = {42, 41, 40, 39};
const uint8_t GPIO_MULTIPLEXER_ANALOG_INPUT      = 10;

} // namespace RobotEnv

#endif // ROBOT_ENV_HPP
