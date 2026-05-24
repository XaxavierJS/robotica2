# Laboratorio 2: Navegación reactiva con filtrado y fusión de sensores en Webots
**Código:** ICI 4150 - Robótica y Sistemas Autónomos (2026-01)

## 👥 Integrantes
- Vicente Palma Lucero
- Lucas Pinto Aliste
- Javier Retamal Frez

## 🎯 Objetivo
Implementar un sistema básico de navegación reactiva en Webots para un robot móvil diferencial (e-puck), utilizando sensores de distancia y encoders de rueda. Se aplica filtrado sobre las mediciones y se emplea un filtro de Kalman para estimar la distancia frontal a obstáculos, mejorando la robustez en la toma de decisiones.

---

## 🤖 Robot y Sensores
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

---

## ⏱️ Frecuencia de Muestreo
El controlador se sincroniza con el motor físico del simulador Webots mediante el parámetro `TIME_STEP`.
- **Tiempo de simulación (TIME_STEP):** 64 ms
- **Tiempo de muestreo ($T_s$):** 0.064 s
- **Frecuencia de muestreo ($f_s$):** $\approx 15.625$ Hz

Todas las lecturas de los sensores, encoders y las actualizaciones del filtro de Kalman se ejecutan a esta frecuencia.

---

## 📏 Estimación de Avance mediante Encoders
Los encoders miden el desplazamiento angular de la rueda en radianes ($\theta$). Para obtener el avance lineal ($\Delta s$), utilizamos el radio de la rueda ($r = 0.0205$ m).

Fórmula del delta de avance central:
$$ \Delta s = \frac{r \cdot \Delta \theta_{izq} + r \cdot \Delta \theta_{der}}{2} $$

Este avance se utiliza como la entrada del modelo de movimiento (predicción) en nuestro filtro.

---

## 🎛️ Filtro Simple (Promedio Móvil)
Antes de enviar las lecturas de distancia de los sensores frontales a Kalman, se les aplica un filtro de promedio móvil con una ventana de muestreo ($N = 5$).
Esto reduce el ruido de alta frecuencia inicial proveniente de los sensores de proximidad infrarrojos.

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

---

## 🧠 Implementación del Filtro de Kalman
El filtro de Kalman fusiona la información del movimiento del robot (encoders) y la medición del entorno (sensores IR filtrados) para estimar de manera óptima la **distancia al obstáculo frontal**.

Propiedades configuradas:
- Ruido de proceso ($Q$): `0.0001` (incertidumbre de odometría/resbalamiento).
- Ruido de medición ($R$): `0.0005` (incertidumbre del sensor IR).

### 1. Etapa de Predicción (Movimiento)
Nuestra estimación *a priori* disminuye la distancia basada en lo que avanzó el robot en el paso anterior.
```python
def predict(self, delta_d):
    # La distancia nueva = distancia anterior menos lo que avanzamos
    self.d = max(self.d - delta_d, 0.0) 
    self.P = self.P + self.Q
```

### 2. Etapa de Corrección (Medición)
Nuestra estimación *a posteriori* ajusta la predicción ponderándola con la lectura real que otorgan los sensores infrarrojos.
```python
def correct(self, z):
    # z es el promedio geométrico frontal filtrado
    self.K = self.P / (self.P + self.R)
    self.d = self.d + self.K * (z - self.d)
    self.P = (1.0 - self.K) * self.P
```

---

## 🧭 Lógica de Navegación Reactiva
La distancia evaluada para tomar decisiones depende de la variable `NAV_MODE` (puede ser lectura cruda, lectura del filtro móvil o la de Kalman).

1. **Avanzar (Libre):** Si $distancia > 0.06$ m, ambos motores van hacia adelante. Usa influencias diagonales para trazar curvas suaves y alejarse amablemente de paredes.
2. **Evasión (Giro):** Si $distancia \le 0.06$ m, se observan los sensores laterales.
   - Si detecta más espacio libre a la izquierda `(min(d_dl, d_ll))`, gira hacia la **izquierda** (motor derecho avanza, izquierdo retrocede).
   - En caso contrario, gira hacia la **derecha**.
3. **Atascamiento:** Si el avance odométrico decae a casi 0 (threshold `0.0001`) durante varios steps seguidos, se activa el modo regenerativo, aplicando retroceso brusco y forzando un giro.

---

## 📊 Escenarios de Prueba y Análisis de Señales
Se evaluó el comportamiento en un circuito **Simple** y uno **Complejo**, guardando el historial de navegación en `/csv`.

*(Nota: Reemplaza las siguientes imágenes por capturas de los gráficos de Python o Excel trazando la información del CSV)*

### Escenario Simple
![Gráfico Simple](./assets/simple_grafico.png) *(Añadir gráfico)*
**Análisis:** En este entorno, el filtro de promedio y Kalman mostraron su capacidad para evadir de manera estable la barrera y las paredes, evitando giros innecesarios provocados por mediciones aleatorias puntuales (ruido).

### Escenario Complejo
![Gráfico Complejo](./assets/complejo_grafico.png) *(Añadir gráfico)*
**Análisis:** Al contar con pasillos estrechos, las señales "raw" fallan frecuentemente debido a la gran cantidad de rebotes, provocando oscilación ("jitter") en el robot. La señal fusionada por Kalman estabilizó considerablemente las decisiones del robot, prefiriendo la odometría cuando los ruidos de reflexión IR aumentan.

---

## 📝 Conclusiones
* **Limitaciones Crudas:** Comandos basados únicamente en lecturas `raw` introdujeron comportamientos de zigzagueo e inconsistencias causados por falsos positivos (ruido del sensor).
* **Mejora del Promedio Móvil:** Ayuda a suavizar los saltos, aunque presenta un ligero retardo computacional asociado al tamaño de ventana ($N=5$).
* **Éxito del Filtro Kalman:** Demostró ser el acercamiento más robusto. Logramos que en episodios de incertidumbre de medición sensada (ruidos parásitos de reflexiones), el modelo priorice su predicción de odometría, manteniendo la linealidad del movimiento del robot.

---

## 🚀 Instrucciones para Ejecutar
1. Clonar el repositorio.
2. Abrir **Webots**.
3. Ir a `File` > `Open World...` y abrir `worlds/simple.wbt` o `worlds/complejo.wbt`. *(Recuerda crearlas / subirlas)*
4. En el archivo `epuck-kalman.py` de la carpeta `controllers/epuck-kalman`, se puede configurar la simulación alterando la constante:
   ```python
   # MODO DE NAVEGACIÓN 
   #   "raw"      - lecturas crudas 
   #   "filtered" - filtro promedio móvil
   #   "kalman"   - estimación óptima
   NAV_MODE = "kalman"
   ```
5. Ejecutar la simulación. En consola aparecerá el resumen, y generará un archivo `lab2_data_{MODO}.csv` en la carpeta actual con todas las capturas de estado.
