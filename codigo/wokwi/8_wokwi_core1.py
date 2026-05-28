# =============================================================================
# LAB 8 — SIMULACION CORE 1 PARA WOKWI (single-core)
# Demuestra exclusivamente las tareas que realiza Core 1 en el sistema real:
#   - Lectura de sensores ADC con promediado de 50 muestras
#       GP26 (ADC0): Voltimetro
#       GP27 (ADC1): Amperimetro
#       GP28 (ADC2): Ohmetro
#       ADC4 interno: Temperatura del chip RP2040
#       GP29 (ADC3): VSYS (con GP25 en alto)
#   - Lectura sensor DHT22 (GP0): temperatura y humedad externa
#   - Barra de estado permanente OLED (filas 0-14): reloj HH:MM:SS + VSYS
#   - Display por modo en area de trabajo (filas 15-63)
#   - Timer de hardware: ISR cada 1000 ms -> reloj en barra de estado
#   - Manejo de botones por polling con debounce (en sistema real lo hace Core 0 via IRQ)
#
# DIFERENCIAS respecto al sistema dual-core:
#   - Sin _thread ni lock (Wokwi no soporta multicore RP2040)
#   - Botones por polling en lugar de IRQ (Core 0 gestiona IRQs en sistema real)
#   - Modo 1 muestra placeholder "GENERADOR / Tarea de Core 0" (sin DAC ni Fourier)
#   - Todo el loop corre secuencialmente en un solo core
#
# Raspberry Pi Pico / Pico 2W - MicroPython - Wokwi
# =============================================================================

import utime
import micropython
from machine import Pin, I2C, ADC, Timer
from ssd1306 import SSD1306_I2C
import dht

micropython.alloc_emergency_exception_buf(100)

# =============================================================================
# 1. HARDWARE
# =============================================================================

i2c  = I2C(1, scl=Pin(19), sda=Pin(18), freq=400000)
oled = SSD1306_I2C(128, 64, i2c, addr=0x3C)

# ADC — multimetro
adc_volt = ADC(26)        # GP26 = ADC0 -> Voltimetro
adc_amp  = ADC(27)        # GP27 = ADC1 -> Amperimetro
adc_ohm  = ADC(Pin(28))   # GP28 = ADC2 -> Ohmetro
adc_temp = ADC(4)         # ADC4 interno -> Temperatura del chip

# Sensor temperatura/humedad
sensor_dht = dht.DHT22(Pin(0))   # GP0

# Botones (polling con debounce por software)
# En el sistema real Core 0 maneja las IRQs; aqui Core 1 los lee directamente
btn_set   = Pin(14, Pin.IN, Pin.PULL_UP)
btn_reset = Pin(15, Pin.IN, Pin.PULL_UP)

# =============================================================================
# 2. CONSTANTES ADC Y RESISTENCIAS DEL CIRCUITO
# =============================================================================

VREF         = 3.3
FACTOR_16    = VREF / 65535
NUM_MUESTRAS = 50    # Muestras promediadas por lectura ADC

# Resistencias del circuito multimetro
RAMP     = 10        # Shunt amperimetro (Ohm)
R7       = 1000      # Resistencia serie ohmetro (Ohm)
RAMP_OHM = 1000      # Resistencia de referencia ohmetro (Ohm)

# =============================================================================
# 3. TIMER DE HARDWARE — RELOJ RTC POR SOFTWARE
#    ISR dispara cada 1000 ms, incrementa HH:MM:SS y levanta flag.
#    Solo aritmetica dentro de la ISR, sin I/O.
# =============================================================================

horas    = 0
minutos  = 0
segundos = 0
flag_timer = False

def isr_timer(timer):
    """ISR del Timer: incrementa reloj HH:MM:SS y levanta bandera. Sin I/O."""
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
# 4. MANEJO DE BOTONES POR POLLING CON DEBOUNCE
#    En el sistema real, las ISRs de Core 0 levantan banderas globales.
#    Aqui Core 1 hace polling directamente con la misma logica de debounce.
# =============================================================================

DEBOUNCE_MS    = 200
ultimo_set     = 0
ultimo_reset   = 0
estado_set_ant = 1    # Estado anterior del boton SET  (reposo=1, pulsado=0)
estado_rst_ant = 1    # Estado anterior del boton RESET

def chequear_botones():
    """Lee ambos botones con debounce por flanco descendente.
    Salidas: (cambio, reset) -> True si se detecto pulsacion valida"""
    global ultimo_set, ultimo_reset, estado_set_ant, estado_rst_ant
    cambio = False
    reset  = False
    ahora  = utime.ticks_ms()

    # btn_set (GP14)
    est_s = btn_set.value()
    if estado_set_ant == 1 and est_s == 0:
        if utime.ticks_diff(ahora, ultimo_set) > DEBOUNCE_MS:
            cambio     = True
            ultimo_set = ahora
    estado_set_ant = est_s

    # btn_reset (GP15)
    est_r = btn_reset.value()
    if estado_rst_ant == 1 and est_r == 0:
        if utime.ticks_diff(ahora, ultimo_reset) > DEBOUNCE_MS:
            reset        = True
            ultimo_reset = ahora
    estado_rst_ant = est_r

    return cambio, reset

# =============================================================================
# 5. FUNCIONES DE LECTURA ADC (tareas exclusivas de Core 1 en sistema real)
# =============================================================================

def leer_promedio(adc):
    """Lee NUM_MUESTRAS del ADC y retorna el promedio (reduccion de ruido).
    Entradas: adc (objeto ADC)
    Salida:   promedio en unidades raw u16 (float)"""
    suma = 0
    for _ in range(NUM_MUESTRAS):
        suma += adc.read_u16()
    return suma / NUM_MUESTRAS

def leer_voltaje():
    """Lee el voltaje en GP26 (ADC0). Rango: 0 a 3.3 V.
    Salida: voltaje en Volts (float)"""
    return leer_promedio(adc_volt) * FACTOR_16

def leer_corriente():
    """Mide la caida de tension en la resistencia shunt RAMP en GP27.
    Salida: corriente en miliamperios (float)"""
    v_shunt = leer_promedio(adc_amp) * FACTOR_16
    if v_shunt > 0.001:
        return (v_shunt / RAMP) * 1000
    return 0.0

def leer_resistencia():
    """Calcula resistencia desconocida Rx usando divisor de tension en GP28.
    Salida: resistencia en Ohm (float) o None si circuito abierto"""
    v_ramp = leer_promedio(adc_ohm) * FACTOR_16
    if v_ramp <= 0.0001:
        return None
    I  = v_ramp / RAMP_OHM
    rx = (VREF / I) - R7 - RAMP_OHM
    return max(0.0, rx)

def leer_temp_interna():
    """Lee el sensor de temperatura interno del RP2040 via ADC4.
    Salida: temperatura en grados Celsius (float)"""
    voltaje = leer_promedio(adc_temp) * FACTOR_16
    return 27 - (voltaje - 0.706) / 0.001721

def leer_vsys():
    """Lee el voltaje VSYS via GP29/ADC3.
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
    """Convierte resistencia a string legible (Ohm / k / M).
    Entradas: rx en Ohm (float) o None
    Salida:   string formateado"""
    if rx is None:        return "Abierto"
    elif rx < 1000:       return "{:.2f}".format(rx)
    elif rx < 1_000_000:  return "{:.3f}k".format(rx / 1000)
    else:                 return "{:.3f}M".format(rx / 1_000_000)

# =============================================================================
# 6. UTILIDADES OLED (tareas exclusivas de Core 1 en sistema real)
#    Layout identico al sistema real:
#      Filas  0-13 : Barra de estado (reloj + VSYS)
#      Fila    14  : Separador horizontal
#      Filas 15-63 : Area de trabajo de cada modo
# =============================================================================

GEN_PY = 15
GEN_PH = 49

vsys_actual = 5.0   # Se actualiza periodicamente

def dibujar_barra_estado():
    """Dibuja reloj HH:MM:SS y VSYS en filas 0-14.
    Usa variables globales horas/minutos/segundos y vsys_actual."""
    oled.fill_rect(0, 0, 128, 14, 0)
    oled.text("{:02d}:{:02d}:{:02d}".format(horas, minutos, segundos), 0, 0, 1)
    oled.text("{:.1f}V".format(vsys_actual), 92, 0, 1)
    oled.hline(0, 14, 128, 1)

def limpiar_area():
    """Borra SOLO el area de trabajo (filas 15-63), sin tocar la barra de estado."""
    oled.fill_rect(0, GEN_PY, 128, GEN_PH, 0)

# =============================================================================
# 7. FUNCIONES DE DISPLAY POR MODO (core 1 dibuja en filas 15-63)
# =============================================================================

def mostrar_placeholder_generador():
    """Modo 1: En sistema real Core 0 controla el generador.
    Aqui solo se muestra un cartel informativo."""
    limpiar_area()
    dibujar_barra_estado()
    oled.text("GENERADOR", 22, 20, 1)
    oled.hline(0, 30, 128, 1)
    oled.text("Tarea de", 28, 36, 1)
    oled.text("Core 0 + DAC", 8, 48, 1)
    oled.show()

def mostrar_voltimetro(v):
    """Modo 2: Muestra el voltaje medido en GP26."""
    limpiar_area()
    dibujar_barra_estado()
    oled.text("VOLTIMETRO", 16, 17, 1)
    oled.hline(0, 26, 128, 1)
    oled.text("Volt:", 0, 30, 1)
    oled.text("{:.4f} V".format(v), 0, 42, 1)
    oled.show()
    print("Voltimetro: {:.4f} V".format(v))

def mostrar_ohmetro(rx_str):
    """Modo 3: Muestra la resistencia calculada desde GP28."""
    limpiar_area()
    dibujar_barra_estado()
    oled.text("OHMETRO", 28, 17, 1)
    oled.hline(0, 26, 128, 1)
    oled.text("Ohm:", 0, 30, 1)
    oled.text("{} Ohm".format(rx_str), 0, 42, 1)
    oled.show()
    print("Ohmetro: {} Ohm".format(rx_str))

def mostrar_amperimetro(i_ma):
    """Modo 4: Muestra la corriente medida via GP27."""
    limpiar_area()
    dibujar_barra_estado()
    oled.text("AMPERIMETRO", 8, 17, 1)
    oled.hline(0, 26, 128, 1)
    oled.text("Amp:", 0, 30, 1)
    oled.text("{:.3f} mA".format(i_ma), 0, 42, 1)
    oled.show()
    print("Amperimetro: {:.3f} mA".format(i_ma))

def mostrar_dht22(t_dht, h_dht, t_int, vs):
    """Modo 5: Muestra DHT22, temperatura interna y VSYS."""
    limpiar_area()
    dibujar_barra_estado()
    oled.text("DHT22+TEMP", 16, 16, 1)
    oled.hline(0, 25, 128, 1)
    oled.text("T:{:.1f}C H:{:.1f}%".format(t_dht, h_dht), 0, 28, 1)
    oled.text("CPU:{:.1f}C".format(t_int),                 0, 40, 1)
    oled.text("VSYS:{:.2f}V".format(vs),                   0, 52, 1)
    oled.show()
    print("DHT:{:.1f}C {:.1f}% | CPU:{:.1f}C | VSYS:{:.2f}V".format(
        t_dht, h_dht, t_int, vs))

NOMBRES_MODO = {
    1: "GENERADOR",
    2: "VOLTIMETRO",
    3: "OHMETRO",
    4: "AMPERIMETRO",
    5: "DHT22",
}

# =============================================================================
# 8. MAIN — LOOP PRINCIPAL (simula Core 1, single-core para Wokwi)
# =============================================================================

def main():
    global modo_actual, flag_timer, vsys_actual

    modo_actual  = 1
    ultimo_dht   = 0
    ultimo_vsys  = 0

    # Valores DHT22 iniciales (se actualizan en modo 5)
    t_dht_cache = 0.0
    h_dht_cache = 0.0

    # --- Pantalla de bienvenida ---
    oled.fill(0)
    oled.text("LAB 8 - CORE 1", 0,  8, 1)
    oled.text("Wokwi/1-core",   0, 22, 1)
    oled.text("ADC+OLED+Reloj", 0, 36, 1)
    oled.text("Sensores+DHT22", 0, 50, 1)
    oled.show()
    utime.sleep(2)

    print("=" * 46)
    print("  LAB 8 - CORE 1 (Wokwi single-core)")
    print("  GP14 btn_set  : avanza modo (polling)")
    print("  GP15 btn_reset: vuelve a Modo 1 (polling)")
    print("  Timer HW 1000ms: reloj en barra de estado")
    print("  Modo 1: placeholder (tarea de Core 0)")
    print("  Modos 2-5: ADC lecturas en OLED")
    print("=" * 46)
    print(">>> MODO 1: GENERADOR (placeholder Core 0)")

    while True:

        # ----------------------------------------------------------------
        # Chequeo de botones por polling con debounce
        # En sistema real esto lo hacen las ISRs de Core 0 via IRQ_FALLING
        # ----------------------------------------------------------------
        cambio, reset = chequear_botones()

        if cambio or reset:
            if reset:
                nuevo_modo = 1
            else:
                nuevo_modo = (modo_actual % 5) + 1

            # Pantalla de transicion
            limpiar_area()
            dibujar_barra_estado()
            oled.text("Cambiando a:", 10, 20, 1)
            oled.text(NOMBRES_MODO[nuevo_modo], 10, 35, 1)
            oled.show()
            utime.sleep_ms(800)

            modo_actual = nuevo_modo
            print(">>> MODO {}: {}".format(nuevo_modo, NOMBRES_MODO[nuevo_modo]))

        # ----------------------------------------------------------------
        # Actualizar VSYS cada 10 segundos (barra de estado de todos los modos)
        # ----------------------------------------------------------------
        ahora = utime.ticks_ms()
        if utime.ticks_diff(ahora, ultimo_vsys) > 10000:
            vsys_actual = leer_vsys()
            ultimo_vsys = ahora

        # ----------------------------------------------------------------
        # Bajar bandera del timer (el reloj se redibuja en cada frame)
        # ----------------------------------------------------------------
        if flag_timer:
            flag_timer = False

        # ================================================================
        # MODO 1: PLACEHOLDER — En sistema real Core 0 corre el generador
        # ================================================================
        if modo_actual == 1:
            mostrar_placeholder_generador()
            utime.sleep_ms(500)

        # ================================================================
        # MODO 2: VOLTIMETRO — Lee GP26 y muestra en OLED
        # ================================================================
        elif modo_actual == 2:
            v = leer_voltaje()
            mostrar_voltimetro(v)
            utime.sleep_ms(200)

        # ================================================================
        # MODO 3: OHMETRO — Lee GP28, calcula Rx, muestra en OLED
        # ================================================================
        elif modo_actual == 3:
            rx = leer_resistencia()
            mostrar_ohmetro(formatear_resistencia(rx))
            utime.sleep_ms(200)

        # ================================================================
        # MODO 4: AMPERIMETRO — Lee GP27, calcula corriente, muestra en OLED
        # ================================================================
        elif modo_actual == 4:
            i_ma = leer_corriente()
            mostrar_amperimetro(i_ma)
            utime.sleep_ms(200)

        # ================================================================
        # MODO 5: DHT22 + TEMPERATURA INTERNA + VSYS
        #   DHT22 se lee cada 2 s (limite del sensor), temp interna cada frame
        # ================================================================
        elif modo_actual == 5:
            t_int = leer_temp_interna()
            ahora = utime.ticks_ms()
            if utime.ticks_diff(ahora, ultimo_dht) > 2000:
                t_dht, h_dht = leer_dht22()
                vs = leer_vsys()
                vsys_actual = vs
                if t_dht is not None:
                    t_dht_cache = t_dht
                    h_dht_cache = h_dht
                ultimo_dht  = ahora
                ultimo_vsys = ahora
            mostrar_dht22(t_dht_cache, h_dht_cache, t_int, vsys_actual)
            utime.sleep_ms(500)

main()
