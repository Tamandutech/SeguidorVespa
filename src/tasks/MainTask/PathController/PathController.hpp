#ifndef PATH_CONTROLLER_HPP
#define PATH_CONTROLLER_HPP

#include <atomic>
#include <cmath>

#include "context/UselessData.hpp"

/**
 * @brief Estrutura que contém as constantes do controlador PID
 *
 * Define os parâmetros de ganho para o controlador
 * proporcional-integral-derivativo usado no controle de direção do robô.
 */
struct PathControllerConstants {
  const float kP; // Ganho proporcional do PID
  const float kI; // Ganho integral do PID
  const float kD; // Ganho derivativo do PID
};

/**
 * @brief Estrutura que define os parâmetros de inicialização do PathController
 *
 * Contém todas as configurações necessárias para inicializar o controlador de
 * caminho, incluindo constantes PID, quantidade de sensores e valores dos
 * sensores.
 */
struct PathControllerParamSchema {
  const PathControllerConstants constants; // Constantes do controlador PID
  const int sensor_quantity;               // Quantidade de sensores de linha
  int *sensor_values;           // Ponteiro para array de valores dos sensores
  const float max_angle;        // Ângulo máximo em graus
  const uint16_t radius_sensor; // Raio dos sensores
  const uint16_t sensor_to_center; // Distância do sensor ao centro
};

/**
 * @brief Classe que implementa o controlador de caminho do robô
 *
 * Esta classe é responsável por processar os dados dos sensores de linha,
 * calcular a posição relativa do robô em relação à linha e fornecer
 * comandos de correção usando um controlador PID.
 */
class PathController {
public:
  PathController(PathControllerParamSchema &param);

  float getLinePosition();

  float getLineAngle();

  float getPID();

private:
  const size_t sensor_quantity; // Quantidade de sensores de linha
  int *sensor_values;           // Ponteiro para array de valores dos sensores
  PathControllerConstants constants; // Constantes do controlador PID

  // Parâmetros geométricos para cálculo do ângulo
  const float max_angle;           // Ângulo máximo em radianos
  const uint16_t radius_sensor;    // Raio dos sensores
  const uint16_t sensor_to_center; // Distância do sensor ao centro

  float integralSummation;         // Acumulador do termo integral do PID
  float lastError; // Último erro calculado para o termo derivativo

  // Rastreamento do estado dos sensores
  uint32_t lastPosition; // Última posição calculada da linha
  bool onLine;           // Indica se o robô está sobre a linha
};

/**
 * @brief Construtor do PathController
 *
 * Inicializa o controlador de caminho com os parâmetros fornecidos,
 * configurando as constantes PID e inicializando as variáveis de estado.
 *
 * @param param Parâmetros de inicialização do controlador
 */
PathController::PathController(PathControllerParamSchema &param)
    : constants(param.constants), sensor_quantity(param.sensor_quantity),
      sensor_values(param.sensor_values),
      max_angle(param.max_angle * M_PI /
                180.0f), // Converte graus para radianos
      radius_sensor(param.radius_sensor),
      sensor_to_center(param.sensor_to_center), integralSummation(0.0f),
      lastError(0.0f), lastPosition(0), onLine(false) {}

/**
 * @brief Calcula a posição da linha em relação aos sensores
 *
 * Este método implementa o algoritmo de detecção de posição da linha baseado
 * no método QTRwithMUX::read_all(). Processa os valores dos sensores para
 * determinar a posição relativa do robô em relação à linha.
 *
 * @return Posição da linha (0 a (sensor_quantity-1)*1000), onde o centro é
 * (sensor_quantity-1)*500
 */
float PathController::getLinePosition() {
  // Baseado no método QTRwithMUX::read_all()

  bool on_Line =
      false; // se falso até o final, os sensores estão todos fora da linha
  uint32_t avg = 0; // soma ponderada das leituras
  uint32_t sum = 0; // soma das leituras

  // Simula leituras dos sensores (substituir por lógica real de leitura dos
  // sensores)
  for(int i = 0; i < sensor_quantity; i++) {
    // Este é um placeholder - substituir por leitura real do sensor
    sensor_values[i] = 0; // Valor simulado do sensor

    // Aplica a mesma lógica do QTRwithMUX::read_all()
    if(sensor_values[i] > 200) {
      on_Line = true; // verifica se tem um sensor na linha
    }
    if(sensor_values[i] >
       50) {                   // valores menores que 50 são considerados ruídos
      avg += (uint32_t)sensor_values[i] *
             (i * 1000);       // soma ponderada de cada leitura
      sum += sensor_values[i]; // soma total das leituras
    }
  }

  if(!on_Line) {
    // Se o robô está fora da linha, retorna a direção da última leitura
    // Se a última posição foi à direita do centro, retorna 0
    if(lastPosition < (sensor_quantity - 1) * 1000 / 2) {
      return 0.0f;
    }
    // Se a última posição foi à esquerda do centro, retorna o valor máximo
    else {
      return (float)((sensor_quantity - 1) * 1000);
    }
  } else {
    // Dividindo avg por sum, obtém-se a posição relativa do robô na linha
    // Com 16 sensores, o centro se encontra no valor 7.500
    lastPosition = (avg / sum);
    onLine       = true;

    // Retorna posição não normalizada (0 a (sensor_quantity-1)*1000)
    return (float)lastPosition;
  }
}

/**
 * @brief Calcula o ângulo de desvio da linha
 *
 * Calcula o ângulo de desvio do robô em relação à linha baseado na posição
 * atual da linha. Este método converte a posição da linha em um ângulo
 * para correção de direção usando geometria circular.
 *
 * @return Ângulo de desvio em radianos (-π/2 a π/2)
 */
float PathController::getLineAngle() {
  // Obtém a posição atual da linha usando o método getLinePosition
  // Agora retorna valor não normalizado (0 a (sensor_quantity-1)*1000)
  int32_t position = (int32_t)getLinePosition();

  // Subtrai a metade do valor máximo para centralizar a posição em 0
  // Para 16 sensores: vai de 0 a 15000, após subtração vai de -7500 a +7500
  position = position - ((sensor_quantity - 1) * 500);

  // Converte a posição para ângulo usando regra de três
  // position vai de -(sensor_quantity-1)*500 a +(sensor_quantity-1)*500
  // max_angle vai de -max_angle a +max_angle
  float angle_radius = (position * max_angle) / ((sensor_quantity - 1) * 500);

  // Calcula o ângulo com o centro usando geometria circular
  // Fórmula: atan(sin(angle_radius) / (cos(angle_radius) - 1 +
  // (sensor_to_center/radius_sensor)))
  float denominator =
      cos(angle_radius) - 1.0f + ((float)sensor_to_center / radius_sensor);

  // Evita divisão por zero
  if(fabs(denominator) < 1e-6f) {
    return 0.0f;
  }

  float angle_with_center = atan(sin(angle_radius) / denominator);

  // Retorna o ângulo em radianos
  return angle_with_center;
}

/**
 * @brief Calcula o valor de correção PID
 *
 * Implementa um controlador PID (Proporcional-Integral-Derivativo) para
 * calcular a correção necessária nos motores baseada no erro de posição.
 * O erro é calculado como o ângulo de desvio da linha.
 *
 * @return Valor de correção para os motores (normalizado)
 */
float PathController::getPID() {
  float error = getLineAngle();
  integralSummation += error;
  float derivative = error - lastError;
  lastError        = error;
  return constants.kP * error + constants.kI * integralSummation +
         constants.kD * derivative;
}

#endif // PATH_CONTROLLER_HPP
