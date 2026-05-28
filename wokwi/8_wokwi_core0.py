# =============================================================================
# LAB 8 — SIMULACION CORE 0 PARA WOKWI (single-core)
# Demuestra exclusivamente las tareas que realiza Core 0 en el sistema real:
#   - Interrupciones GPIO (btn_set GP14, btn_reset GP15) con debounce 200 ms
#   - Timer de hardware (ISR cada 1000 ms -> reloj HH:MM:SS en barra de estado)
#   - Maquina de estados FSM de 5 modos
#   - Pre-calculo de coeficientes de Fourier (PROP 2)
#   - Loop de calculo Fourier + escritura DAC R-2R (GP2-GP9)
#   - OLED: barra de estado + forma de onda en scroll (todo en un solo core)
#
# DIFERENCIAS respecto al sistema dual-core:
#   - Sin _thread ni lock (Wokwi no soporta multicore RP2040)
#   - Core 0 hace su propio display de onda (en el sistema real lo hace Core 1)
#   - Modos 2-5 muestran placeholder "en espera de Core 1"
#   - VSYS fijo en 5.0 V (sin Core 1 no se puede leer ADC29 simultaneamente)
#
# Raspberry Pi Pico / Pico 2W - MicroPython - Wokwi
# =============================================================================

import math
import utime
import micropython
from machine import Pin, I2C, ADC, Timer
from ssd1306 import SSD1306_I2C

micropython.alloc_emergency_exception_buf(100)

# =============================================================================
# 1. HARDWARE
# =============================================================================

i2c  = I2C(1, scl=Pin(19), sda=Pin(18), freq=400000)
oled = SSD1306_I2C(128, 64, i2c, addr=0x3C)

# =============================================================================
# 2. ESTADO GLOBAL DE MODOS
#    1 = Generador Fourier + DAC   (tarea principal de Core 0)
#    2 = Voltimetro                (Core 1 en sistema real; placeholder aqui)
#    3 = Ohmetro                   (Core 1 en sistema real; placeholder aqui)
#    4 = Amperimetro               (Core 1 en sistema real; placeholder aqui)
#    5 = DHT22 + Temp + VSYS       (Core 1 en sistema real; placeholder aqui)
# =============================================================================

modo_actual      = 1
solicitud_cambio = False   # Levantada por ISR de btn_set
solicitud_reset  = False   # Levantada por ISR de btn_reset

# =============================================================================
# 3. BOTONES CON INTERRUPCION GPIO (ISRs)
#    GP14 = btn_set  -> avanza modo ciclicamente (1->2->3->4->5->1)
#    GP15 = btn_reset -> vuelve directo al Modo 1
#    Debounce por software: ventana de 200 ms usando ticks_diff
# =============================================================================

DEBOUNCE_MS      = 200
ultimo_btn_set   = 0
ultimo_btn_reset = 0

def isr_btn_set(pin):
    """ISR GP14: verifica debounce y levanta bandera solicitud_cambio."""
    global solicitud_cambio, ultimo_btn_set
    ahora = utime.ticks_ms()
    if utime.ticks_diff(ahora, ultimo_btn_set) > DEBOUNCE_MS:
        solicitud_cambio = True
        ultimo_btn_set   = ahora

def isr_btn_reset(pin):
    """ISR GP15: verifica debounce y levanta bandera solicitud_reset."""
    global solicitud_reset, ultimo_btn_reset
    ahora = utime.ticks_ms()
    if utime.ticks_diff(ahora, ultimo_btn_reset) > DEBOUNCE_MS:
        solicitud_reset  = True
        ultimo_btn_reset = ahora

btn_set   = Pin(14, Pin.IN, Pin.PULL_UP)
btn_reset = Pin(15, Pin.IN, Pin.PULL_UP)
btn_set.irq(trigger=Pin.IRQ_FALLING,  handler=isr_btn_set)
btn_reset.irq(trigger=Pin.IRQ_FALLING, handler=isr_btn_reset)

# =============================================================================
# 4. TIMER DE HARDWARE — RELOJ RTC POR SOFTWARE
#    ISR dispara cada 1000 ms, incrementa contadores HH:MM:SS.
#    Solo aritmetica entrera dentro de la ISR, sin I/O.
# =============================================================================

horas    = 0
minutos  = 0
segundos = 0
flag_timer = False

def isr_timer(timer):
    """ISR del Timer: incrementa reloj y levanta bandera. Sin I/O."""
    global flag_timer, segundos, minutos, horas
    segundos += 1
    if segundos >= 60:
        segundos = 0
        minutos += 1
        if minutos >= 60:
            minutos = 0
            horas += 1
            if horas >= 24:
                horas = 0
    flag_timer = True

reloj_hw = Timer()
reloj_hw.init(period=1000, mode=Timer.PERIODIC, callback=isr_timer)

# =============================================================================
# 5. DAC R-2R (GP2-GP9, 8 bits efectivos)
#    GP2 = bit 0 (LSB) ... GP9 = bit 7 (MSB)
#    Valor 0 -> 0V | Valor 255 -> Vref (~3.3 V)
# =============================================================================

pines_dac  = [2, 3, 4, 5, 6, 7, 8, 9]
pines_gpio = [Pin(p, Pin.OUT) for p in pines_dac]

def GPIO_SALP(valor):
    """Escribe valor de 8 bits al DAC R-2R.
    Entradas: valor entero 0-255
    Salida:   nivel analogico proporcional en la salida del DAC R-2R"""
    valor = max(0, min(255, valor))
    for i in range(8):
        pines_gpio[i].value((valor >> i) & 1)

# =============================================================================
# 6. MATEMATICA FOURIER — PROPOSICION 2
#    f(t) periodica con T=4, w0=pi/2
#    a0 = -0.5
#    an = -(1/(n*pi)) * sin(n*pi/2)
#    bn =  (3/(n*pi)) * (1 - cos(n*pi/2))
#    DAC: fx clampeada a [-2.0, 1.0] -> mapeada a [0, 255]
# =============================================================================

T_math = 4.0
w0     = 2 * math.pi / T_math   # = pi/2
a0     = -0.5
nmax   = 50

def coef_an(n):
    """Coeficiente coseno para Proposicion 2."""
    return -(1 / (n * math.pi)) * math.sin(n * math.pi / 2)

def coef_bn(n):
    """Coeficiente seno para Proposicion 2."""
    return (3 / (n * math.pi)) * (1 - math.cos(n * math.pi / 2))

# Pre-calcular todos los coeficientes al inicio (evita recalcular en el loop)
print("Pre-calculando coeficientes Fourier (n=1..{})...".format(nmax))
lista_an = [0.0] * (nmax + 1)
lista_bn = [0.0] * (nmax + 1)
for _n in range(1, nmax + 1):
    lista_an[_n] = coef_an(_n)
    lista_bn[_n] = coef_bn(_n)
print("Listo.")

def fourier_eval(x):
    """Evalua la serie de Fourier en el punto x con nmax armonicos.
    Entradas: x (float) - punto de evaluacion
    Salida:   f(x) (float) - valor de la serie"""
    suma = 0.0
    for n in range(1, nmax + 1):
        suma += lista_an[n] * math.cos(n * w0 * x) \
              + lista_bn[n] * math.sin(n * w0 * x)
    return (a0 / 2.0) + suma

# =============================================================================
# 7. UTILIDADES OLED
#    Layout identico al sistema real:
#      Filas  0-13 : Barra de estado (reloj + VSYS)
#      Fila    14  : Separador horizontal
#      Filas 15-63 : Area de trabajo (forma de onda)
# =============================================================================

GEN_PY = 15
GEN_PH = 49
VSYS_APROX = 5.0   # Fijo: Core 0 no lee ADC29 en Wokwi single-core

def dibujar_barra_estado():
    """Dibuja reloj y VSYS en filas 0-14."""
    oled.fill_rect(0, 0, 128, 14, 0)
    oled.text("{:02d}:{:02d}:{:02d}".format(horas, minutos, segundos), 0, 0, 1)
    oled.text("{:.1f}V".format(VSYS_APROX), 92, 0, 1)
    oled.hline(0, 14, 128, 1)

def limpiar_area():
    """Borra el area de trabajo (filas 15-63)."""
    oled.fill_rect(0, GEN_PY, 128, GEN_PH, 0)

def math_to_screen_y(fx, ymin, ymax):
    """Convierte valor matematico fx al pixel y del area de trabajo."""
    ny = (fx - ymin) / (ymax - ymin) if ymax != ymin else 0.5
    yp = GEN_PY + int((1.0 - ny) * (GEN_PH - 1))
    return max(GEN_PY, min(GEN_PY + GEN_PH - 1, yp))

# Buffer de scroll para la forma de onda (1 pixel por columna)
buf_yp = [GEN_PY + GEN_PH // 2] * 128

NOMBRES_MODO = {
    1: "GENERADOR",
    2: "VOLTIMETRO",
    3: "OHMETRO",
    4: "AMPERIMETRO",
    5: "DHT22",
}

# Rangos de la grafica para PROP 2
YMIN = -2.5
YMAX =  1.5

# =============================================================================
# 8. MAIN — LOOP PRINCIPAL (simula Core 0, single-core para Wokwi)
# =============================================================================

def main():
    global modo_actual, solicitud_cambio, solicitud_reset, flag_timer

    t_actual = 0.0
    delta_t  = 0.05   # Paso de tiempo entre muestras

    # --- Pantalla de bienvenida ---
    oled.fill(0)
    oled.text("LAB 8 - CORE 0", 0,  8, 1)
    oled.text("Wokwi/1-core",   0, 22, 1)
    oled.text("ISR+Timer+DAC",  0, 36, 1)
    oled.text("Fourier PROP 2", 0, 50, 1)
    oled.show()
    utime.sleep(2)

    print("=" * 46)
    print("  LAB 8 - CORE 0 (Wokwi single-core)")
    print("  GP14 btn_set  : avanza modo (IRQ_FALLING)")
    print("  GP15 btn_reset: vuelve a Modo 1 (IRQ_FALLING)")
    print("  Timer HW 1000ms: reloj en barra de estado")
    print("  Modo 1: Fourier(50 arm.) -> DAC R-2R GP2-GP9")
    print("  Modos 2-5: placeholder (tarea de Core 1)")
    print("=" * 46)
    print(">>> MODO 1: GENERADOR")

    while True:

        # ----------------------------------------------------------------
        # Detectar cambio de modo (banderas levantadas por ISRs de botones)
        # ----------------------------------------------------------------
        if solicitud_cambio or solicitud_reset:

            if solicitud_reset:
                nuevo_modo = 1
            else:
                nuevo_modo = (modo_actual % 5) + 1

            # Pantalla de transicion (respeta barra de estado)
            limpiar_area()
            dibujar_barra_estado()
            oled.text("Cambiando a:", 10, 20, 1)
            oled.text(NOMBRES_MODO[nuevo_modo], 10, 35, 1)
            oled.show()
            utime.sleep_ms(800)

            modo_actual      = nuevo_modo
            solicitud_cambio = False
            solicitud_reset  = False
            t_actual         = 0.0
            buf_yp[:]        = [GEN_PY + GEN_PH // 2] * 128
            GPIO_SALP(0)     # DAC a 0 durante transicion

            print(">>> MODO {}: {}".format(nuevo_modo, NOMBRES_MODO[nuevo_modo]))

        # ----------------------------------------------------------------
        # Bajar bandera del timer (la barra se redibuja en cada frame)
        # ----------------------------------------------------------------
        if flag_timer:
            flag_timer = False

        # ================================================================
        # MODO 1: CALCULO FOURIER + DAC + DISPLAY DE ONDA EN SCROLL
        #   En el sistema real Core 0 solo calcula y escribe el DAC;
        #   el display lo hace Core 1. Aqui Core 0 lo hace todo.
        # ================================================================
        if modo_actual == 1:
            # Calcular siguiente punto
            t_actual += delta_t
            fx = fourier_eval(t_actual)

            # Escribir al DAC R-2R (tiempo critico en sistema real)
            fx_clamp  = max(-2.0, min(1.0, fx))
            valor_dac = int(((fx_clamp + 2.0) / 3.0) * 255)
            GPIO_SALP(valor_dac)

            # Scroll del buffer + dibujar forma de onda
            yp = math_to_screen_y(fx, YMIN, YMAX)
            for i in range(127):
                buf_yp[i] = buf_yp[i + 1]
            buf_yp[127] = yp

            limpiar_area()
            # Eje horizontal en y=0 si esta dentro del rango
            if YMIN <= 0 <= YMAX:
                ay = math_to_screen_y(0, YMIN, YMAX)
                oled.hline(0, ay, 128, 1)
            # Dibujar forma de onda con lineas
            for col in range(1, 128):
                oled.line(col - 1, buf_yp[col - 1], col, buf_yp[col], 1)
            # Barra de estado encima de la onda
            dibujar_barra_estado()
            oled.show()

        # ================================================================
        # MODOS 2-5: En el sistema real Core 0 descansa (Core 1 trabaja).
        #   Aqui se muestra un placeholder indicando que es tarea de Core 1.
        # ================================================================
        else:
            GPIO_SALP(0)   # DAC en nivel 0 cuando no genera
            limpiar_area()
            dibujar_barra_estado()
            oled.text(NOMBRES_MODO[modo_actual], 10, 24, 1)
            oled.hline(0, 33, 128, 1)
            oled.text("Tarea de", 28, 38, 1)
            oled.text("Core 1", 36, 50, 1)
            oled.show()
            utime.sleep_ms(200)

main()
