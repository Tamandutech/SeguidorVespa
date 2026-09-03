# Mapeamento da Pista V1

O mapemaneto da pista pode ser feito com os dados dos encoders, de uma IMU, ou da fusão de ambos.

O importante é manter o contrato de qual medida será salva no arquivo de mapeamento.

Nesse caso, vamos salvar a contagem dos encoders de cada roda e o ângulo de curvatura da linha.

## Formato do arquivo de mapeamento

```
index,x,y,delta_theta,target_speed
```

## Algoritmo de mapeamento

### 1. Entrar no modo de mapeamento

### 2. Calcular o angulo de curvatura da linha

### 3. Definir limite de angulo para registrar ponto no mapa