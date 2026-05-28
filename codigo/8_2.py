# =============================================================================
# LAB 8 — INTERRUPCIONES EXTERNAS Y DE TIMER
# Base: ejercicio 2 de Fourier (PROP 2) del Lab 7
# 5 modos: Generador Fourier+DAC | Voltimetro | Ohmetro | Amperimetro | DHT22
# Barra de estado permanente (filas 0-14): Reloj HH:MM:SS + VSYS
# Cambio de modo con interrupciones GPIO:
#   GP14 = btn_set   -> avanza modo ciclicamente (1->2->3->4->5->1)
#   GP15 = btn_reset -> vuelve directo al Modo 1
# Timer de hardware: dispara cada 1 segundo, incrementa reloj via ISR
# Dual Core:
#   Core 0: Logica principal + Calculo Fourier + DAC (tiempo-critico)
#   Core 1: Lectura sensores + Dibujo OLED (visual)
# Raspberry Pi Pico 2W - MicroPython
# =============================================================================

import _thread
import math
import utime
import sys
import select
import micropython
from machine import Pin, I2C, ADC, Timer
from ssd1306 import SSD1306_I2C
import dht

# Reserva buffer para reportar excepciones dentro de ISRs
micropython.alloc_emergency_exception_buf(100)

# =============================================================================
# 1. HARDWARE COMPARTIDO
# =============================================================================

i2c  = I2C(1, scl=Pin(19), sda=Pin(18), freq=400000)
oled = SSD1306_I2C(128, 64, i2c, addr=0x3C)

# Lock para sincronizar acceso a datos compartidos entre Core 0 y Core 1
lock = _thread.allocate_lock()

# =============================================================================
# 2. ESTADO GLOBAL DE MODOS
#    1 = Generador Fourier+DAC
#    2 = Voltimetro
#    3 = Ohmetro
#    4 = Amperimetro
#    5 = DHT22 + Temp interna + VSYS
# =============================================================================

modo_actual      = 1
solicitud_cambio = False   # Seteado por ISR de btn_set
solicitud_reset  = False   # Seteado por ISR de btn_reset
en_transicion    = False   # True mientras Core 0 muestra pantalla de transicion

# =============================================================================
# 3. BOTONES CON INTERRUPCION GPIO (ISRs)
#    btn_set  -> GP14 (avanza modo)
#    btn_reset -> GP15 (vuelve a Modo 1)
#    PULL_UP activo: reposo=1, presionado=0 -> disparo en flanco FALLING
# =============================================================================

DEBOUNCE_MS      = 200     # Tiempo minimo entre pulsaciones validas (ms)
ultimo_btn_set   = 0
ultimo_btn_reset = 0

def isr_btn_set(pin):
    """ISR del boton SET: solo verifica debounce y levanta bandera.
    Entradas: pin (objeto Pin, ignorado)
    Salidas:  solicitud_cambio = True si debounce OK"""
    global solicitud_cambio, ultimo_btn_set
    ahora = utime.ticks_ms()
    if utime.ticks_diff(ahora, ultimo_btn_set) > DEBOUNCE_MS:
        solicitud_cambio = True
        ultimo_btn_set   = ahora

def isr_btn_reset(pin):
    """ISR del boton RESET: solo verifica debounce y levanta bandera.
    Entradas: pin (objeto Pin, ignorado)
    Salidas:  solicitud_reset = True si debounce OK"""
    global solicitud_reset, ultimo_btn_reset
    ahora = utime.ticks_ms()
    if utime.ticks_diff(ahora, ultimo_btn_reset) > DEBOUNCE_MS:
        solicitud_reset  = True
        ultimo_btn_reset = ahora

# Inicializar pines y asignar ISRs
btn_set   = Pin(14, Pin.IN, Pin.PULL_UP)
btn_reset = Pin(15, Pin.IN, Pin.PULL_UP)
btn_set.irq(trigger=Pin.IRQ_FALLING,  handler=isr_btn_set)
btn_reset.irq(trigger=Pin.IRQ_FALLING, handler=isr_btn_reset)

# =============================================================================
# 4. TIMER DE HARDWARE — RELOJ RTC POR SOFTWARE
#    ISR dispara cada 1000 ms, incrementa contador de tiempo y levanta flag.
#    La ISR NO escribe en OLED ni hace print (buena practica ISR).
# =============================================================================

horas    = 0
minutos  = 0
segundos = 0
flag_timer = False   # Core 1 la baja despues de actualizar la barra de estado

def isr_timer(timer):
    """ISR del Timer de hardware: incrementa reloj y levanta bandera.
    Periodo: 1000 ms. Solo aritmetica simple, sin I/O.
    Entradas: timer (objeto Timer, ignorado)
    Salidas:  horas, minutos, segundos actualizados; flag_timer = True"""
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

# Inicializar timer de hardware (unico canal libre para el usuario)
reloj_hw = Timer()
reloj_hw.init(period=1000, mode=Timer.PERIODIC, callback=isr_timer)

# =============================================================================
# 5. HARDWARE MULTIMETRO
# =============================================================================

adc_volt = ADC(26)        # GP26 = ADC0 -> Voltimetro
adc_amp  = ADC(27)        # GP27 = ADC1 -> Amperimetro
adc_ohm  = ADC(Pin(28))   # GP28 = ADC2 -> Ohmetro
adc_temp = ADC(4)         # ADC4 interno -> Temperatura del chip

sensor_dht = dht.DHT22(Pin(0))  # Sensor temperatura/humedad en GP0

VREF         = 3.3
FACTOR_16    = VREF / 65535
NUM_MUESTRAS = 50

# Resistencias del circuito multimetro
R3       = 4700    # Divisor voltimetro
R4       = 220
RAMP     = 10      # Shunt amperimetro (ohm)
R5       = 1000
R6       = 220
R7       = 1000    # Resistencia serie ohmetro
RAMP_OHM = 1000

# Diccionario compartido de datos del multimetro (acceso protegido por lock)
datos_multi = {
    'voltaje':      0.0,
    'corriente_ma': 0.0,
    'rx':           None,
    'rx_str':       "---",
    'temp_int':     0.0,
    'vsys':         5.0,    # Valor inicial razonable
    'temp_dht':     0.0,
    'hum_dht':      0.0,
}

# =============================================================================
# 6. DATOS COMPARTIDOS GENERADOR (Core 0 produce, Core 1 consume y dibuja)
# =============================================================================

datos_gen = {
    'fx':     0.0,
    't':      0.0,
    'activo': False,
    'nuevo':  False,       # True = Core 0 produjo dato nuevo aun no consumido
    'layout': 'fullscreen',
    'nombre': 'PROP 2',
    'xmin':   -2.0,
    'xmax':    2.0,
    'ymin':   -2.5,
    'ymax':    1.5,
}

# =============================================================================
# 7. FUNCIONES DE LECTURA MULTIMETRO
# =============================================================================

def leer_promedio(adc):
    """Lee NUM_MUESTRAS del ADC y retorna el promedio.
    Entradas: adc (objeto ADC)
    Salida:   promedio en unidades raw u16"""
    suma = 0
    for _ in range(NUM_MUESTRAS):
        suma += adc.read_u16()
    return suma / NUM_MUESTRAS

def leer_voltaje():
    """Lee el voltaje en GP26 (ADC0). Rango: 0 a 3.3 V.
    Salida: voltaje en Volts (float)"""
    return leer_promedio(adc_volt) * FACTOR_16

def leer_corriente():
    """Mide la caida de tension en la resistencia shunt (RAMP) en GP27.
    Salida: corriente en miliamperios (float)"""
    v_shunt = leer_promedio(adc_amp) * FACTOR_16
    if v_shunt > 0.001:
        return (v_shunt / RAMP) * 1000
    return 0.0

def leer_resistencia():
    """Calcula la resistencia desconocida Rx usando divisor de tension en GP28.
    Salida: resistencia en Ohm (float) o None si circuito abierto"""
    v_ramp = leer_promedio(adc_ohm) * FACTOR_16
    if v_ramp <= 0.0001:
        return None
    I  = v_ramp / RAMP_OHM
    rx = (VREF / I) - R7 - RAMP_OHM
    return 0.0 if rx < 0 else rx

def leer_temp_interna():
    """Lee el sensor de temperatura interno del RP2040 via ADC4.
    Salida: temperatura en grados Celsius (float)"""
    voltaje = leer_promedio(adc_temp) * FACTOR_16
    return 27 - (voltaje - 0.706) / 0.001721

def leer_vsys():
    """Lee el voltaje VSYS de la Pico 2W via GP29/ADC3.
    GP25 se pone en ALTO para habilitar la lectura, luego vuelve a BAJO.
    Salida: voltaje VSYS en Volts (float)"""
    pin_cs = Pin(25, Pin.OUT)
    pin_cs.value(1)
    adc_vsys = ADC(Pin(29))
    v = leer_promedio(adc_vsys) * FACTOR_16
    pin_cs.value(0)
    return v * 3

def leer_dht22():
    """Lee temperatura y humedad del sensor DHT22 en GP0.
    Salida: (temperatura, humedad) o (None, None) si falla"""
    try:
        sensor_dht.measure()
        return sensor_dht.temperature(), sensor_dht.humidity()
    except:
        return None, None

def formatear_resistencia(rx):
    """Convierte un valor de resistencia a string legible (Ohm, k, M).
    Entradas: rx en Ohm (float) o None
    Salida:   string formateado"""
    if rx is None:        return "Abierto"
    elif rx < 1000:       return "{:.2f}".format(rx)
    elif rx < 1000000:    return "{:.3f}k".format(rx / 1000)
    else:                 return "{:.3f}M".format(rx / 1000000)

# =============================================================================
# 8. HARDWARE GENERADOR DE SENALES (DAC R2R, 8 bits efectivos, GP2-GP11)
# =============================================================================

pines_dac  = [2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
pines_gpio = None

def init_dac():
    """Configura los pines del DAC R2R como salidas digitales."""
    global pines_gpio
    pines_gpio = [Pin(p, Pin.OUT) for p in pines_dac]

def release_dac():
    """Libera los pines del DAC (los pone como entradas para no forzar nivel)."""
    global pines_gpio
    if pines_gpio:
        for p in pines_gpio:
            p.init(Pin.IN)
    pines_gpio = None

def GPIO_SALP(valor):
    """Escribe un valor de 8 bits al DAC R2R (GP2=LSB, GP9=MSB).
    Entradas: valor entero 0-255
    Salida:   nivel analogico proporcional en la salida del DAC"""
    if pines_gpio is None:
        return
    valor = max(0, min(255, valor))
    for i in range(8):
        pines_gpio[i].value((valor >> i) & 1)

# =============================================================================
# 9. MATEMATICA FOURIER — PROPOSICION 2
#    f(t) con periodo T=4, a0=-0.5
#    an = -(1/(n*pi)) * sin(n*pi/2)
#    bn = (3/(n*pi)) * (1 - cos(n*pi/2))
# =============================================================================

T_math = 4.0
w0     = 2 * math.pi / T_math
a0     = -0.5

def coef_an(n):
    """Coeficiente coseno de la serie de Fourier para Proposicion 2."""
    return -(1 / (n * math.pi)) * math.sin(n * math.pi / 2)

def coef_bn(n):
    """Coeficiente seno de la serie de Fourier para Proposicion 2."""
    return (3 / (n * math.pi)) * (1 - math.cos(n * math.pi / 2))

def precalcular_coeficientes(nmax):
    """Pre-calcula todos los coeficientes an y bn hasta nmax.
    Salida: (lista_an, lista_bn) listas indexadas del 0 al nmax"""
    lista_an = [0] * (nmax + 1)
    lista_bn = [0] * (nmax + 1)
    for n in range(1, nmax + 1):
        lista_an[n] = coef_an(n)
        lista_bn[n] = coef_bn(n)
    return lista_an, lista_bn

def fourier_eval(x, nmax, lista_an, lista_bn):
    """Evalua la serie de Fourier en el punto x usando nmax armonicos.
    Entradas: x (float), nmax (int), coeficientes pre-calculados
    Salida:   valor f(x) (float)"""
    suma = 0.0
    for n in range(1, nmax + 1):
        suma += lista_an[n] * math.cos(n * w0 * x) + lista_bn[n] * math.sin(n * w0 * x)
    return (a0 / 2) + suma

# =============================================================================
# 10. UTILIDADES DE PANTALLA OLED
#     Distribucion de la pantalla (128x64 pixeles):
#       Filas  0-13 : Barra de estado (reloj + VSYS)  -- INTOCABLE por los modos
#       Fila    14  : Linea separadora horizontal
#       Filas 15-63 : Area de trabajo de cada modo
# =============================================================================

GEN_PY = 15    # y inicial del area de graficacion (fila 15)
GEN_PH = 49    # alto del area de graficacion (15 + 49 = 64)

def limpiar_area_trabajo():
    """Borra SOLO las filas 15-63, preservando la barra de estado (0-14)."""
    oled.fill_rect(0, GEN_PY, 128, GEN_PH, 0)

def dibujar_barra_estado():
    """Dibuja el reloj y VSYS en las filas 0-14.
    Lee la hora de las variables globales (actualizadas por isr_timer).
    Lee VSYS del diccionario compartido datos_multi['vsys'].
    NO llama oled.show() -- lo hace la funcion que la invoca."""
    lock.acquire()
    vs = datos_multi['vsys']
    lock.release()
    oled.fill_rect(0, 0, 128, 14, 0)
    reloj_str = "{:02d}:{:02d}:{:02d}".format(horas, minutos, segundos)
    oled.text(reloj_str, 0, 0, 1)
    oled.text("{:.1f}V".format(vs), 92, 0, 1)
    oled.hline(0, 14, 128, 1)

def math_to_screen(xm, ym, xmin, xmax, ymin, ymax,
                   px=0, py=GEN_PY, pw=128, ph=GEN_PH):
    """Convierte coordenadas matematicas a coordenadas de pantalla OLED.
    Entradas: punto matematico (xm, ym) y rangos del grafico
    Salida:   (xp, yp) coordenadas en pixeles, clampeadas al area"""
    nx = (xm - xmin) / (xmax - xmin) if xmax != xmin else 0.0
    ny = (ym - ymin) / (ymax - ymin) if ymax != ymin else 0.0
    xp = px + int(nx * (pw - 1))
    yp = py + int((1.0 - ny) * (ph - 1))
    xp = max(px, min(px + pw - 1, xp))
    yp = max(py, min(py + ph - 1, yp))
    return xp, yp

def draw_axes(xmin, xmax, ymin, ymax, px, py, pw, ph):
    """Dibuja los ejes X e Y dentro del area de graficacion si el origen es visible."""
    if xmin <= 0 <= xmax:
        ax, _ = math_to_screen(0, ymin, xmin, xmax, ymin, ymax, px, py, pw, ph)
        oled.vline(ax, py, ph, 1)
    if ymin <= 0 <= ymax:
        _, ay = math_to_screen(xmin, 0, xmin, xmax, ymin, ymax, px, py, pw, ph)
        oled.hline(px, ay, pw, 1)

def draw_info(nombre, x_val, y_val, x0=82):
    """Dibuja el panel de informacion del generador (layout split).
    Muestra nombre de la funcion, X(t) e Y(t) actuales."""
    oled.text(nombre[:6],                 x0, GEN_PY +  0, 1)
    oled.text("X(t):",                    x0, GEN_PY +  9, 1)
    oled.text("{:.2f}".format(x_val)[:6], x0, GEN_PY + 18, 1)
    oled.text("Y(t):",                    x0, GEN_PY + 29, 1)
    oled.text("{:.2f}".format(y_val)[:6], x0, GEN_PY + 38, 1)

# =============================================================================
# 11. FUNCIONES DE DISPLAY POR MODO (todas usan filas 15-63)
# =============================================================================

def mostrar_voltimetro():
    """Dibuja la pantalla del Modo 2: Voltimetro.
    Lee datos_multi['voltaje']. Muestra en OLED y consola."""
    lock.acquire()
    v = datos_multi['voltaje']
    lock.release()
    limpiar_area_trabajo()
    dibujar_barra_estado()
    oled.text("VOLTIMETRO", 16, 17, 1)
    oled.hline(0, 26, 128, 1)
    oled.text("Volt:", 0, 30, 1)
    oled.text("{:.4f} V".format(v), 0, 42, 1)
    oled.show()
    print("Voltimetro: {:.4f} V".format(v))

def mostrar_ohmetro():
    """Dibuja la pantalla del Modo 3: Ohmetro.
    Lee datos_multi['rx_str']. Muestra en OLED y consola."""
    lock.acquire()
    rx_str = datos_multi['rx_str']
    lock.release()
    limpiar_area_trabajo()
    dibujar_barra_estado()
    oled.text("OHMETRO", 28, 17, 1)
    oled.hline(0, 26, 128, 1)
    oled.text("Ohm:", 0, 30, 1)
    oled.text("{} Ohm".format(rx_str), 0, 42, 1)
    oled.show()
    print("Ohmetro: {} Ohm".format(rx_str))

def mostrar_amperimetro():
    """Dibuja la pantalla del Modo 4: Amperimetro.
    Lee datos_multi['corriente_ma']. Muestra en OLED y consola."""
    lock.acquire()
    i_ma = datos_multi['corriente_ma']
    lock.release()
    limpiar_area_trabajo()
    dibujar_barra_estado()
    oled.text("AMPERIMETRO", 8, 17, 1)
    oled.hline(0, 26, 128, 1)
    oled.text("Amp:", 0, 30, 1)
    oled.text("{:.3f} mA".format(i_ma), 0, 42, 1)
    oled.show()
    print("Amperimetro: {:.3f} mA".format(i_ma))

def mostrar_dht22():
    """Dibuja la pantalla del Modo 5: DHT22 + Temperatura interna + VSYS.
    Lee datos_multi completo. Muestra en OLED y consola."""
    lock.acquire()
    t_dht = datos_multi['temp_dht']
    h_dht = datos_multi['hum_dht']
    t_int = datos_multi['temp_int']
    vs    = datos_multi['vsys']
    lock.release()
    limpiar_area_trabajo()
    dibujar_barra_estado()
    oled.text("DHT22+TEMP", 16, 16, 1)
    oled.hline(0, 25, 128, 1)
    oled.text("T:{:.1f}C H:{:.1f}%".format(t_dht, h_dht), 0, 28, 1)
    oled.text("CPU:{:.1f}C".format(t_int),                 0, 40, 1)
    oled.text("VSYS:{:.2f}V".format(vs),                   0, 52, 1)
    oled.show()
    print("DHT:{:.1f}C {:.1f}% | CPU:{:.1f}C | VSYS:{:.2f}V".format(
        t_dht, h_dht, t_int, vs))

# =============================================================================
# 12. CORE 1 — LECTURA DE SENSORES + DIBUJO OLED
#     - Lee sensores segun el modo activo
#     - Actualiza VSYS periodicamente (para la barra de estado en todos los modos)
#     - Dibuja la pantalla correspondiente al modo
#     - En modo Generador, consume datos de Core 0 y dibuja la forma de onda
# =============================================================================

def core1_tareas():
    global modo_actual, flag_timer

    ultimo_dht  = 0
    ultimo_vsys = 0

    # Variables locales del display del generador
    gen_inicializado = False
    gen_layout  = None
    buf_yp      = None
    buf_xp      = None
    px = py = pw = ph = 0
    xmin = xmax = ymin = ymax = 0.0
    nombre = ""
    x0p    = 64

    while True:

        # ---- Pausar si Core 0 esta ejecutando una transicion de modo ----
        lock.acquire()
        transicion = en_transicion
        lock.release()
        if transicion:
            utime.sleep_ms(50)
            continue

        # ---- Obtener modo actual ----
        lock.acquire()
        modo = modo_actual
        lock.release()

        # ---- Actualizar VSYS cada 10 s independientemente del modo ----
        ahora = utime.ticks_ms()
        if utime.ticks_diff(ahora, ultimo_vsys) > 10000:
            vs = leer_vsys()
            lock.acquire()
            datos_multi['vsys'] = vs
            lock.release()
            ultimo_vsys = ahora

        # ---- Bajar bandera del timer (la barra se redibuja en cada frame) ----
        if flag_timer:
            flag_timer = False

        # ============================================================
        # MODO 1: GENERADOR -- Core 1 dibuja la forma de onda en OLED
        # ============================================================
        if modo == 1:
            lock.acquire()
            activo = datos_gen['activo']
            lock.release()

            if not activo:
                gen_inicializado = False
                utime.sleep_ms(50)
                continue

            # Esperar dato nuevo de Core 0
            lock.acquire()
            hay_nuevo = datos_gen['nuevo']
            lock.release()
            if not hay_nuevo:
                continue

            # Inicializar buffers la primera vez que entra al modo
            if not gen_inicializado:
                lock.acquire()
                gen_layout = datos_gen['layout']
                xmin   = datos_gen['xmin']
                xmax   = datos_gen['xmax']
                ymin   = datos_gen['ymin']
                ymax   = datos_gen['ymax']
                nombre = datos_gen['nombre']
                lock.release()

                if gen_layout == 'fullscreen':
                    px, py, pw, ph = 0, GEN_PY, 128, GEN_PH
                    buf_yp = [py + ph // 2] * pw
                elif gen_layout == 'split':
                    px, py, pw, ph = 0, GEN_PY, 80, GEN_PH
                    buf_yp = [py + ph // 2] * pw
                elif gen_layout == 'portrait':
                    buf_xp = [GEN_PY + GEN_PH // 2] * GEN_PH
                    nx0 = (0.0 - ymin) / (ymax - ymin) if ymax != ymin else 0.5
                    x0p = int(nx0 * 127)
                    x0p = max(0, min(127, x0p))

                gen_inicializado = True

            # Consumir dato de Core 0
            lock.acquire()
            fx       = datos_gen['fx']
            t_actual = datos_gen['t']
            datos_gen['nuevo'] = False
            lock.release()

            # ---------- FULLSCREEN ----------
            if gen_layout == 'fullscreen':
                _, yp = math_to_screen(0, fx, 0, 1, ymin, ymax, px, py, pw, ph)
                for i in range(pw - 1):
                    buf_yp[i] = buf_yp[i + 1]
                buf_yp[pw - 1] = yp
                limpiar_area_trabajo()
                draw_axes(xmin, xmax, ymin, ymax, px, py, pw, ph)
                for col in range(1, pw):
                    oled.line(px + col - 1, buf_yp[col - 1],
                              px + col,     buf_yp[col],     1)
                dibujar_barra_estado()
                oled.show()

            # ---------- SPLIT ----------
            elif gen_layout == 'split':
                _, yp = math_to_screen(0, fx, 0, 1, ymin, ymax, px, py, pw, ph)
                for i in range(pw - 1):
                    buf_yp[i] = buf_yp[i + 1]
                buf_yp[pw - 1] = yp
                limpiar_area_trabajo()
                draw_axes(xmin, xmax, ymin, ymax, px, py, pw, ph)
                oled.vline(80, GEN_PY, GEN_PH, 1)
                draw_info(nombre, t_actual, fx, x0=82)
                for col in range(1, pw):
                    oled.line(px + col - 1, buf_yp[col - 1],
                              px + col,     buf_yp[col],     1)
                dibujar_barra_estado()
                oled.show()

            # ---------- PORTRAIT ----------
            elif gen_layout == 'portrait':
                nx = (fx - ymin) / (ymax - ymin) if ymax != ymin else 0.5
                xp_v = max(0, min(127, int(nx * 127)))
                for i in range(GEN_PH - 1):
                    buf_xp[i] = buf_xp[i + 1]
                buf_xp[GEN_PH - 1] = xp_v
                limpiar_area_trabajo()
                oled.vline(x0p, GEN_PY, GEN_PH, 1)
                for row in range(1, GEN_PH):
                    oled.line(buf_xp[row - 1], GEN_PY + row - 1,
                              buf_xp[row],     GEN_PY + row,     1)
                dibujar_barra_estado()
                oled.show()

        # ============================================================
        # MODO 2: VOLTIMETRO
        # ============================================================
        elif modo == 2:
            gen_inicializado = False
            v = leer_voltaje()
            lock.acquire()
            datos_multi['voltaje'] = v
            lock.release()
            mostrar_voltimetro()
            utime.sleep_ms(200)

        # ============================================================
        # MODO 3: OHMETRO
        # ============================================================
        elif modo == 3:
            gen_inicializado = False
            rx = leer_resistencia()
            lock.acquire()
            datos_multi['rx']     = rx
            datos_multi['rx_str'] = formatear_resistencia(rx)
            lock.release()
            mostrar_ohmetro()
            utime.sleep_ms(200)

        # ============================================================
        # MODO 4: AMPERIMETRO
        # ============================================================
        elif modo == 4:
            gen_inicializado = False
            i_ma = leer_corriente()
            lock.acquire()
            datos_multi['corriente_ma'] = i_ma
            lock.release()
            mostrar_amperimetro()
            utime.sleep_ms(200)

        # ============================================================
        # MODO 5: DHT22 + TEMPERATURA INTERNA + VSYS
        # ============================================================
        elif modo == 5:
            gen_inicializado = False
            t_int = leer_temp_interna()
            ahora = utime.ticks_ms()
            if utime.ticks_diff(ahora, ultimo_dht) > 2000:
                t_dht, h_dht = leer_dht22()
                vs = leer_vsys()
                lock.acquire()
                if t_dht is not None:
                    datos_multi['temp_dht'] = t_dht
                    datos_multi['hum_dht']  = h_dht
                datos_multi['temp_int'] = t_int
                datos_multi['vsys']     = vs
                lock.release()
                ultimo_dht  = ahora
                ultimo_vsys = ahora
            mostrar_dht22()
            utime.sleep_ms(500)

# =============================================================================
# 13. CORE 0 — LOOP GENERADOR: calculo Fourier + escritura DAC (sincronizado)
#     Core 0 produce un punto, espera que Core 1 lo consuma, luego produce el
#     siguiente. Asi la forma de onda dibujada es identica a la enviada al DAC.
# =============================================================================

def core0_generador_loop(nmax, lista_an, lista_bn):
    """Bucle de calculo Fourier y escritura al DAC R2R.
    Sale cuando detecta solicitud de cambio de modo.
    Entradas: nmax, listas de coeficientes pre-calculados"""
    t_actual = 0.0
    delta_t  = 0.05

    while True:
        # Verificar si se pidio cambio de modo
        if solicitud_cambio or solicitud_reset:
            GPIO_SALP(0)
            lock.acquire()
            datos_gen['activo'] = False
            datos_gen['nuevo']  = False
            lock.release()
            return

        # Esperar a que Core 1 haya consumido el dato anterior
        while True:
            lock.acquire()
            pendiente = datos_gen['nuevo']
            lock.release()
            if not pendiente:
                break
            if solicitud_cambio or solicitud_reset:
                GPIO_SALP(0)
                lock.acquire()
                datos_gen['activo'] = False
                datos_gen['nuevo']  = False
                lock.release()
                return

        # Calcular siguiente punto de la serie
        t_actual += delta_t
        fx = fourier_eval(t_actual, nmax, lista_an, lista_bn)

        # Escribir al DAC inmediatamente (tiempo critico)
        fx_clamp  = max(-2.0, min(1.0, fx))
        valor_dac = int(((fx_clamp + 2.0) / 3.0) * 255)
        GPIO_SALP(valor_dac)

        # Publicar dato para Core 1
        lock.acquire()
        datos_gen['fx']    = fx
        datos_gen['t']     = t_actual
        datos_gen['nuevo'] = True
        lock.release()

# =============================================================================
# 14. EJECUTAR GENERADOR (menu por consola + arranque del loop DAC)
# =============================================================================

def ejecutar_generador():
    """Muestra menu de layout en consola, espera seleccion,
    configura datos_gen y lanza el loop de calculo+DAC en Core 0.
    Sale si se presiona un boton durante el menu o durante la ejecucion."""
    global solicitud_cambio, solicitud_reset

    init_dac()

    nmax = 50
    lista_an, lista_bn = precalcular_coeficientes(nmax)

    xmin, xmax = -2.0, 2.0
    ymin, ymax = -2.5, 1.5
    nombre     = "PROP 2"

    # Pantalla de espera en OLED mientras el usuario elige en consola
    limpiar_area_trabajo()
    dibujar_barra_estado()
    oled.text("GENERADOR", 28, 17, 1)
    oled.hline(0, 26, 128, 1)
    oled.text("Elige en", 20, 32, 1)
    oled.text("consola...", 14, 44, 1)
    oled.show()

    print("")
    print("=" * 42)
    print("  GENERADOR DE SENALES -- PROP 2")
    print("=" * 42)
    print("  [1] Fullscreen")
    print("  [2] Split (Grafica + Info)")
    print("  [3] Portrait (Scroll vertical)")
    print("  Boton SET cambia de modo")
    print("=" * 42)

    # Esperar seleccion del usuario (o salida por boton)
    while True:
        if solicitud_cambio or solicitud_reset:
            release_dac()
            return
        if select.select([sys.stdin], [], [], 0)[0]:
            op = sys.stdin.readline().strip()
            if op in ("1", "2", "3"):
                break
            else:
                print("Opcion invalida. Escriba 1, 2 o 3.")
        utime.sleep_ms(100)

    # Limpiar flags en caso de activacion durante la espera
    lock.acquire()
    solicitud_cambio = False
    solicitud_reset  = False
    lock.release()

    print("Layout: {}".format({1: "Fullscreen", 2: "Split", 3: "Portrait"}[int(op)]))

    layout_map = {"1": "fullscreen", "2": "split", "3": "portrait"}

    lock.acquire()
    datos_gen['layout'] = layout_map[op]
    datos_gen['nombre'] = nombre
    datos_gen['xmin']   = xmin
    datos_gen['xmax']   = xmax
    datos_gen['ymin']   = ymin
    datos_gen['ymax']   = ymax
    datos_gen['nuevo']  = False
    datos_gen['activo'] = True
    lock.release()

    # Core 0 se queda en este loop hasta que se presione un boton
    core0_generador_loop(nmax, lista_an, lista_bn)
    release_dac()

# =============================================================================
# 15. CORE 0 — LOGICA PRINCIPAL
#     Detecta solicitudes de cambio de modo (levantadas por las ISRs de botones),
#     ejecuta la pantalla de transicion y actualiza modo_actual.
#     Cuando modo==1 ejecuta el generador. Para modos 2-5 Core 1 hace el trabajo.
# =============================================================================

NOMBRES_MODO = {
    1: "GENERADOR",
    2: "VOLTIMETRO",
    3: "OHMETRO",
    4: "AMPERIMETRO",
    5: "DHT22"
}

def core0_principal():
    global modo_actual, solicitud_cambio, solicitud_reset, en_transicion

    while True:
        # ---- Verificar solicitudes de cambio levantadas por las ISRs ----
        cambiar  = solicitud_cambio
        resetear = solicitud_reset

        if cambiar or resetear:

            # Bloquear a Core 1 para que no interfiera con la transicion
            lock.acquire()
            en_transicion = True
            lock.release()
            utime.sleep_ms(80)   # Dar tiempo a Core 1 para terminar su frame

            # Calcular nuevo modo
            if resetear:
                nuevo_modo = 1
            else:
                lock.acquire()
                m = modo_actual
                lock.release()
                nuevo_modo = (m % 5) + 1   # Ciclo: 1->2->3->4->5->1

            # Mostrar pantalla de transicion (respeta barra de estado)
            limpiar_area_trabajo()
            dibujar_barra_estado()
            oled.text("Cambiando a:", 10, 20, 1)
            oled.text(NOMBRES_MODO[nuevo_modo], 10, 35, 1)
            oled.show()
            utime.sleep_ms(900)

            # Actualizar estado global y liberar bloqueo
            lock.acquire()
            modo_actual      = nuevo_modo
            solicitud_cambio = False
            solicitud_reset  = False
            en_transicion    = False
            lock.release()

            print(">>> MODO {}: {}".format(nuevo_modo, NOMBRES_MODO[nuevo_modo]))

        # ---- Ejecutar tarea de Core 0 segun modo ----
        lock.acquire()
        modo = modo_actual
        lock.release()

        if modo == 1:
            # Core 0 corre el generador (calculo + DAC) hasta que se pida cambio
            ejecutar_generador()
        else:
            # Modos 2-5: Core 1 se encarga de todo, Core 0 descansa
            utime.sleep_ms(100)

# =============================================================================
# 16. MAIN — ARRANQUE DEL SISTEMA
# =============================================================================

def main():
    print("=" * 50)
    print("  LAB 8 - INTERRUPCIONES Y TIMER")
    print("  GP14 btn_set  : avanza modo (1->2->3->4->5->1)")
    print("  GP15 btn_reset: vuelve a Modo 1")
    print("  Timer HW      : reloj cada 1 segundo")
    print("  Core 0        : Logica + Fourier + DAC")
    print("  Core 1        : Sensores + OLED")
    print("=" * 50)

    # Pantalla de bienvenida (usa toda la pantalla, antes de que Core 1 arranque)
    oled.fill(0)
    oled.text("LAB 8", 38, 4, 1)
    oled.text("INTERRUPCIONES", 4, 16, 1)
    oled.text("Y TIMER", 34, 27, 1)
    oled.hline(0, 38, 128, 1)
    oled.text("SET -> sig.modo", 0, 44, 1)
    oled.text("RST -> Modo 1",  0, 54, 1)
    oled.show()
    utime.sleep(2)

    # Lanzar Core 1 en segundo nucleo
    _thread.start_new_thread(core1_tareas, ())

    print(">>> MODO 1: Generador (por defecto)")

    # Core 0 ejecuta la logica principal
    core0_principal()

main()
