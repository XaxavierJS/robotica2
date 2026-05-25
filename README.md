# Laboratorio 2: Navegación reactiva con filtrado y fusión de sensores en Webots

**Código:** ICI 4150 - Robótica y Sistemas Autónomos (2026-01)

## Integrantes

- Vicente Palma Lucero
- Lucas Pinto Aliste
- Javier Retamal Frez

## Objetivo

Implementar un sistema de navegación reactiva en el simulador Webots para el robot móvil diferencial e-puck, aplicando tres estrategias de procesamiento de señal sobre los sensores de proximidad: lectura cruda (raw), filtro de promedio móvil y filtro de Kalman. El objetivo es comparar cuantitativamente el impacto de cada estrategia en la calidad de la señal percibida y en el comportamiento de navegación resultante, evaluado en dos escenarios de distinta complejidad.

---

## Robot y Sensores

Se utilizó el robot **e-puck** en el simulador Webots.

| Dispositivo | Nombre en Webots | Uso en el laboratorio |
| :--- | :--- | :--- |
| Motor Izquierdo | `left wheel motor` | Actuador de avance y giro |
| Motor Derecho | `right wheel motor` | Actuador de avance y giro |
| Encoder Izquierdo | `left wheel sensor` | Estimar avance odométrico |
| Encoder Derecho | `right wheel sensor` | Estimar avance odométrico |
| Frontales IR | `ps0` y `ps7` | Detección directa del obstáculo frontal |
| Diagonales IR | `ps1` y `ps6` | Ajuste suave de trayectoria (curvas) |
| Laterales IR | `ps2` y `ps5` | Decidir dirección de evasión (izquierda o derecha) |

La conversión de valor crudo a distancia utiliza la siguiente relación empírica inversa:

$$d = \frac{0.03}{0.003 \cdot \text{raw} + 0.3}, \quad d \in [0.005,\ 0.12]\ \text{m}$$

Valores crudos por debajo de 65 (umbral de ruido de fondo) se interpretan como ausencia de obstáculo y se mapean directamente a la distancia máxima (0.12 m).

---

## Frecuencia de Muestreo

| Parámetro | Valor |
| :--- | :--- |
| TIME_STEP (simulación) | 64 ms |
| $T_s$ (período de muestreo) | 0.064 s |
| $f_s$ (frecuencia de muestreo) | 15.625 Hz |
| Pasos totales por experimento | 5 000 |
| Duración total por experimento | 320 s |

Todas las lecturas de sensores, encoders y la actualización del filtro de Kalman se ejecutan síncronamente a esta frecuencia mediante la llamada `robot.step(TIME_STEP)`.

---

## Estimación del Avance mediante Encoders

Los encoders miden el desplazamiento angular de cada rueda en radianes. El avance lineal diferencial por paso se calcula como:

$$\Delta s = \frac{r \cdot (\Delta\theta_{\text{izq}} + \Delta\theta_{\text{der}})}{2}$$

con $r = 0.0205$ m (radio de la rueda del e-puck) y $L = 0.052$ m (distancia entre ejes).

Este valor $\Delta s$ se utiliza como entrada del modelo de movimiento (etapa de predicción) del filtro de Kalman: si el robot avanzó $\Delta s$ metros, la distancia estimada al obstáculo frontal disminuye en esa misma cantidad.

El avance total acumulado refleja directamente cuán eficiente es cada modo de navegación:

| Modo | Avance total — escenario simple | Avance total — escenario complejo |
| :--- | :---: | :---: |
| Raw | 2.13 m | 2.05 m |
| Kalman | 13.97 m | 13.16 m |

El modo crudo recorre apenas el 15 % de la distancia del modo Kalman porque pasa el 89 % del tiempo girando en lugar de avanzar.

---

## Filtro Simple (Promedio Móvil)

Antes de que la medición llegue al filtro de Kalman, se aplica un filtro de promedio móvil con ventana $N = 5$ de forma independiente sobre cada sensor frontal (ps0 y ps7):

```python
class MovingAverageFilter:
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.buffer = []

    def update(self, value):
        self.buffer.append(value)
        if len(self.buffer) > self.window_size:
            self.buffer.pop(0)
        return sum(self.buffer) / len(self.buffer)
```

El promedio aritmético de ambas salidas filtradas constituye la medición $z_k$ que recibe la etapa de corrección de Kalman. Este pre-filtrado reduce el ruido de alta frecuencia de los sensores IR antes de la fusión, mejorando la calidad de la medición sin introducir complejidad adicional al estimador.

---

## Implementación del Filtro de Kalman

El filtro de Kalman 1D fusiona la predicción odométrica con la medición sensorial para estimar de manera óptima la distancia frontal al obstáculo más cercano.

Parámetros configurados:

| Parámetro | Símbolo | Valor |
| :--- | :---: | :---: |
| Ruido de proceso (odometría) | $Q$ | 0.0001 |
| Ruido de medición (sensor IR) | $R$ | 0.0005 |
| Covarianza inicial | $P_0$ | 0.01 |
| Estimación inicial de distancia | $\hat{d}_0$ | 0.12 m |

La elección $R = 5Q$ refleja que el sensor IR introduce cinco veces más incertidumbre que el modelo de movimiento.

### Etapa de Predicción

A partir del avance odométrico $\Delta s$, la estimación *a priori* descuenta la distancia recorrida y la incertidumbre del proceso crece:

```python
def predict(self, delta_d):
    self.d = max(self.d - delta_d, 0.0)   # modelo de movimiento
    self.P = self.P + self.Q               # propagación de incertidumbre
```

### Etapa de Corrección

La medición $z_k$ (promedio filtrado de ps0 y ps7) corrige la predicción ponderada por la ganancia de Kalman $K$:

```python
def correct(self, z):
    self.K = self.P / (self.P + self.R)      # ganancia óptima
    self.d = self.d + self.K * (z - self.d)  # actualización de la estimación
    self.P = (1.0 - self.K) * self.P         # reducción de incertidumbre
```

Cuando $K \to 1$ el filtro confía en el sensor; cuando $K \to 0$ confía en la predicción odométrica.

### Convergencia en Estado Estacionario

A partir de $P_0 = 0.01$, el filtro converge a su estado estacionario en menos de 5 pasos (~0.32 s):

| Parámetro | Valor estacionario |
| :--- | :---: |
| Covarianza *a posteriori* $P_\infty$ | ≈ 0.000179 |
| Covarianza *a priori* $P_\infty + Q$ | ≈ 0.000279 |
| Ganancia de Kalman $K_\infty$ | ≈ 0.358 |

En estado estacionario, el filtro asigna **35.8 %** del peso a la medición sensorial y **64.2 %** a la predicción odométrica, coherente con $R > Q$.

---

## Lógica de Navegación Reactiva

La distancia de decisión (`decision_distance`) se selecciona según el modo activo (`NAV_MODE`). La lógica sigue la jerarquía de prioridades descrita a continuación:

| Prioridad | Condición | Acción |
| :---: | :--- | :--- |
| 1 | `reverse_counter > 0` | REVERSE: retrocede y gira para salir del atasco |
| 2 | `turn_commit_counter > 0` | Continúa el giro comprometido (`TURN_COMMIT_STEPS = 8` pasos) |
| 3 | `decision_dist ≤ 0.06 m` | Gira hacia el lado con más espacio lateral libre |
| 4 | Sin obstáculo frontal | FORWARD: avanza a 4.0 rad/s con curvas suaves por diagonales |

**Curvas suaves:** cuando un sensor diagonal detecta un objeto a menos de 0.12 m, se reduce proporcionalmente la velocidad de la rueda correspondiente hasta en un 60 %, generando una curva gradual sin giro in-situ.

**Anti-atascamiento:** si el avance odométrico acumulado permanece bajo el umbral de 0.0001 m durante 20 pasos consecutivos, se activa una secuencia de reversa de 12 pasos seguida de un giro forzado.

| Parámetro | Valor |
| :--- | :--- |
| Umbral de seguridad frontal | 0.06 m |
| Velocidad base de avance | 4.0 rad/s |
| Velocidad de giro in-situ | ±1.8 rad/s |
| Pasos comprometidos de giro | 8 |
| Umbral de atasco | 0.0001 m/paso durante 20 pasos |


## Resultados por Escenario

### Análisis de Señales

| Señal | Escenario simple σ (m) | CV (%) | Escenario complejo σ (m) | CV (%) |
| :--- | :---: | :---: | :---: | :---: |
| Cruda (raw) | 0.0188 | 25.6 | 0.0195 | 27.1 |
| Promedio móvil | 0.0090 | 12.3 | 0.0106 | 14.8 |
| Kalman | 0.0068 | 9.9 | 0.0081 | 12.0 |
| Reducción vs. cruda (Kalman) | −64 % | — | −58 % | — |

### Distribución de Acciones — Escenario Simple

| Acción | Raw | Filtered | Kalman |
| :--- | :---: | :---: | :---: |
| FORWARD_CURVE_R | 381 (7.6 %) | 2 802 (56.0 %) | 2 711 (54.2 %) |
| FORWARD_CURVE_L | 110 (2.2 %) | 658 (13.2 %) | 708 (14.2 %) |
| FORWARD | 35 (0.7 %) | 190 (3.8 %) | 150 (3.0 %) |
| TURN_LEFT | 2 151 (43.0 %) | 783 (15.7 %) | 837 (16.7 %) |
| TURN_RIGHT | 2 323 (46.5 %) | 567 (11.3 %) | 594 (11.9 %) |
| **Total giros in-situ** | **4 474 (89.5 %)** | **1 350 (27.0 %)** | **1 431 (28.6 %)** |

### Distribución de Acciones — Escenario Complejo

| Acción | Raw | Filtered | Kalman |
| :--- | :---: | :---: | :---: |
| FORWARD_CURVE_R | 375 (7.5 %) | 3 788 (75.8 %) | 2 620 (52.4 %) |
| FORWARD_CURVE_L | 101 (2.0 %) | 76 (1.5 %) | 589 (11.8 %) |
| FORWARD | 32 (0.6 %) | 20 (0.4 %) | 171 (3.4 %) |
| TURN_LEFT | 2 169 (43.4 %) | 180 (3.6 %) | 882 (17.6 %) |
| TURN_RIGHT | 2 323 (46.5 %) | 936 (18.7 %) | 738 (14.8 %) |
| **Total giros in-situ** | **4 492 (89.8 %)** | **1 116 (22.3 %)** | **1 620 (32.4 %)** |

### Tiempo con Distancia de Decisión bajo el Umbral de Seguridad (0.06 m)

| Modo | Escenario simple | Escenario complejo |
| :--- | :---: | :---: |
| Raw | 48.7 % | 49.3 % |
| Filtered | 10.4 % | 7.6 % |
| Kalman | 8.3 % | 13.3 % |

En el modo crudo, casi la mitad de la simulación se percibe como una situación de obstáculo inminente cuando en realidad se trata de ruido sensorial.

---

## Análisis Final y Conclusiones

### Modo Raw

El modo crudo es ineficiente en ambos escenarios. Con el 89 % del tiempo en giro, el robot apenas avanza 2 m en 320 s. El coeficiente de variación superior al 25 % hace que el controlador interprete oscilaciones de ruido IR como obstáculos reales, generando maniobras de evasión en cascada. Este comportamiento se agrava ligeramente en el escenario complejo (CV = 27.1 % vs. 25.6 %), consistente con la mayor densidad de objetos que aumenta los rebotes infrarrojos.

### Modo Filtro de Promedio Móvil

El promedio móvil ($N=5$) reduce a la mitad la desviación estándar de la señal (−52 % simple, −46 % complejo) y disminuye el tiempo de giro a menos del 28 %. El avance efectivo sube a ~14 m. En el escenario complejo, el modo filtrado genera el 75.8 % del tiempo en curvas a la derecha, el mayor porcentaje de movimiento continuo registrado, probablemente favorecido por la geometría del mapa. Su desventaja es el retardo de ventana (~160 ms), que puede dilatar la respuesta ante obstáculos que aparecen bruscamente.

### Modo Kalman

El filtro de Kalman obtiene la mayor reducción de ruido en ambos escenarios (−64 % en simple, −58 % en complejo) y converge a su estado estacionario en menos de 5 pasos. La ganancia estacionaria $K_\infty \approx 0.358$ pondera el sensor al 35.8 % y la odometría al 64.2 %, lo que es coherente con $R = 5Q$.

En el escenario complejo, el modo Kalman presenta un comportamiento más simétrico que el filtrado (52.4 % curvas derecha + 11.8 % curvas izquierda vs. 75.8 % solo derecha), sugiriendo una adaptación más equilibrada a distintas configuraciones del mapa. El tiempo bajo el umbral de seguridad sube al 13.3 % en el complejo (vs. 8.3 % en simple), lo que es esperable pero permanece muy por debajo del 49 % del modo crudo.

### Comparación entre Escenarios

| Métrica | Simple – RAW | Simple – Kalman | Complejo – RAW | Complejo – Kalman |
| :--- | :---: | :---: | :---: | :---: |
| CV señal cruda | 25.6 % | — | 27.1 % | — |
| CV señal Kalman | — | 9.9 % | — | 12.0 % |
| Tiempo girando | 89.5 % | 28.6 % | 89.8 % | 32.4 % |
| Tiempo bajo umbral | 48.7 % | 8.3 % | 49.3 % | 13.3 % |
| Avance total | 2.13 m | 13.97 m | 2.05 m | 13.16 m |
| Eventos STUCK/REVERSE | 0 | 0 | 0 | 0 |

El escenario complejo reduce el avance total en Kalman en un **5.8 %** (13.16 m vs. 13.97 m) y exige un **3.8 %** más de tiempo en giros. A pesar de esto, la efectividad del filtro se mantiene por encima del 58 %, validando su robustez frente a entornos de mayor complejidad. La ausencia de eventos de atascamiento en todos los modos y escenarios confirma que la lógica de evasión lateral es suficiente para los mapas evaluados.

---

## Instrucciones para Ejecutar la Simulación

1. Clonar el repositorio:
   ```bash
   git clone https://github.com/XaxavierJS/robotica2.git
   ```

2. Abrir **Webots** e ir a `File > Open World...`. Seleccionar el mundo deseado (`worlds/simple.wbt` o `worlds/complejo.wbt`).

3. Configurar el modo de navegación en `epuck-kalman.py` (carpeta `controllers/epuck-kalman`):
   ```python
   # Opciones: "raw" | "filtered" | "kalman"
   NAV_MODE = "kalman"
   ```

4. Ejecutar la simulación. El controlador imprime el estado cada 50 pasos por consola y al finalizar genera el archivo `lab2_data_{MODO}.csv` en el directorio de trabajo.

5. Para comparar los tres modos, repetir la simulación cambiando `NAV_MODE` entre `"raw"`, `"filtered"` y `"kalman"`. Mover los CSV generados a `csv/simple/` o `csv/complejo/` según el escenario utilizado.

6. Abrir `analisis_simple.ipynb` o `analisis_complejo.ipynb` en Jupyter para reproducir los gráficos y el análisis:
   ```bash
   jupyter notebook
   ```
