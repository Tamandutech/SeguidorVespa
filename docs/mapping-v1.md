# Mapeamento da Pista V1

O mapemaneto da pista pode ser feito com os dados dos encoders, de uma IMU, ou da fusão de ambos.

O importante é manter o contrato de qual medida será salva no arquivo de mapeamento. Nesse caso, queremos salvar a posição do robô em coordenadas cartesianas e o algoritmo fica responsável por transformar os dados dos sensores escolhidos para o formato de coordenadas cartesianas.

## Formato do arquivo de mapeamento

```
index,x,y,target_speed
```

