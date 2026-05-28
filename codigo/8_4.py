# =============================================================================
# LAB 8 — INTERRUPCIONES EXTERNAS Y DE TIMER
# Ejercicio 4: Función Exponencial
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

micropython.alloc_emergency_exception_buf(100)

# =============================================================================
# 1. HARDWARE COMPARTIDO
# =============================================================================

i2c  = I2C(1, scl=Pin(19), sda=Pin(18), freq=400000)
oled = SSD1306_I2C(128, 64, i2c, addr=0x3C)
lock = _thread.allocate_lock()

# =============================================================================
# 2. ESTADO GLOBAL DE MODOS
# =============================================================================

modo_actual      = 1
solicitud_cambio = False
solicitud_reset  = False
en_transicion    = False

# =============================================================================
# 3. BOTONES CON INTERRUPCION GPIO (ISRs)
# =============================================================================

DEBOUNCE_MS      = 200
ultimo_btn_set   = 0
ultimo_btn_reset = 0

def isr_btn_set(pin):
    global solicitud_cambio, ultimo_btn_set
    ahora = utime.ticks_ms()
    if utime.ticks_diff(ahora, ultimo_btn_set) > DEBOUNCE_MS:
        solicitud_cambio = True
        ultimo_btn_set   = ahora

def isr_btn_reset(pin):
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
# =============================================================================

horas    = 0
minutos  = 0
segundos = 0
flag_timer = False

def isr_timer(timer):
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
# 5. HARDWARE MULTIMETRO
# =============================================================================

adc_volt = ADC(26)
adc_amp  = ADC(27)
adc_ohm  = ADC(Pin(28))
adc_temp = ADC(4)

sensor_dht = dht.DHT22(Pin(0))

VREF         = 3.3
FACTOR_16    = VREF / 65535
NUM_MUESTRAS = 50

R3       = 4700
R4       = 220
RAMP     = 10
R5       = 1000
R6       = 220
R7       = 1000
RAMP_OHM = 1000

datos_multi = {
    'voltaje':      0.0,
    'corriente_ma': 0.0,
    'rx':           None,
    'rx_str':       "---",
    'temp_int':     0.0,
    'vsys':         5.0,
    'temp_dht':     0.0,
    'hum_dht':      0.0,
}

# =============================================================================
# 6. DATOS COMPARTIDOS GENERADOR
# =============================================================================

datos_gen = {
    'fx':     0.0,
    't':      0.0,
    'activo': False,
    'nuevo':  False,
    'layout': 'fullscreen',
    'nombre': 'EXPON',
    'xmin':   -0.5,
    'xmax':    1.5,
    'ymin':    0.0,
    'ymax':    1.5,
}

# =============================================================================
# 7. FUNCIONES DE LECTURA MULTIMETRO
# =============================================================================

def leer_promedio(adc):
    suma = 0
    for _ in range(NUM_MUESTRAS):
        suma += adc.read_u16()
    return suma / NUM_MUESTRAS

def leer_voltaje():
    return leer_promedio(adc_volt) * FACTOR_16

def leer_corriente():
    v_shunt = leer_promedio(adc_amp) * FACTOR_16
    if v_shunt > 0.001:
        return (v_shunt / RAMP) * 1000
    return 0.0

def leer_resistencia():
    v_ramp = leer_promedio(adc_ohm) * FACTOR_16
    if v_ramp <= 0.0001:
        return None
    I  = v_ramp / RAMP_OHM
    rx = (VREF / I) - R7 - RAMP_OHM
    return 0.0 if rx < 0 else rx

def leer_temp_interna():
    voltaje = leer_promedio(adc_temp) * FACTOR_16
    return 27 - (voltaje - 0.706) / 0.001721

def leer_vsys():
    pin_cs = Pin(25, Pin.OUT)
    pin_cs.value(1)
    adc_vsys = ADC(Pin(29))
    v = leer_promedio(adc_vsys) * FACTOR_16
    pin_cs.value(0)
    return v * 3

def leer_dht22():
    try:
        sensor_dht.measure()
        return sensor_dht.temperature(), sensor_dht.humidity()
    except:
        return None, None

def formatear_resistencia(rx):
    if rx is None:        return "Abierto"
    elif rx < 1000:       return "{:.2f}".format(rx)
    elif rx < 1000000:    return "{:.3f}k".format(rx / 1000)
    else:                 return "{:.3f}M".format(rx / 1000000)

# =============================================================================
# 8. HARDWARE GENERADOR DE SENALES (DAC R2R, GP2-GP9)
# =============================================================================

pines_dac  = [2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
pines_gpio = None

def init_dac():
    global pines_gpio
    pines_gpio = [Pin(p, Pin.OUT) for p in pines_dac]

def release_dac():
    global pines_gpio
    if pines_gpio:
        for p in pines_gpio:
            p.init(Pin.IN)
    pines_gpio = None

def GPIO_SALP(valor):
    if pines_gpio is None:
        return
    valor = max(0, min(255, valor))
    for i in range(8):
        pines_gpio[i].value((valor >> i) & 1)

# =============================================================================
# 9. MATEMATICA FOURIER — EJERCICIO 4: FUNCION EXPONENCIAL
#    Periodo T = 0.5  =>  w0 = 4*pi
#    Constante C = 1 - e^(-0.5)  ~= 0.3935
#    a0 = 4*C
#    an = 4*C / (1 + (4*pi*n)^2)
#    bn = 16*pi*n*C / (1 + (4*pi*n)^2)
#    Rango aprox: [0, 1.5]
#    Clamp DAC: [0.0, 1.5]  => valor_dac = int((fx_clamp/1.5)*255)
# =============================================================================

T_math = 0.5
w0     = 4.0 * math.pi           # w0 = 4*pi
C_exp  = 1.0 - math.exp(-0.5)    # C ~= 0.3935
a0     = 4.0 * C_exp

def coef_an(n):
    return (4.0 * C_exp) / (1.0 + (4.0 * math.pi * n) ** 2)

def coef_bn(n):
    return (16.0 * math.pi * n * C_exp) / (1.0 + (4.0 * math.pi * n) ** 2)

def precalcular_coeficientes(nmax):
    lista_an = [0.0] * (nmax + 1)
    lista_bn = [0.0] * (nmax + 1)
    for n in range(1, nmax + 1):
        lista_an[n] = coef_an(n)
        lista_bn[n] = coef_bn(n)
    return lista_an, lista_bn

def fourier_eval(x, nmax, lista_an, lista_bn):
    suma = 0.0
    for n in range(1, nmax + 1):
        suma += lista_an[n] * math.cos(n * w0 * x) + lista_bn[n] * math.sin(n * w0 * x)
    return (a0 / 2) + suma

# =============================================================================
# 10. UTILIDADES DE PANTALLA OLED
# =============================================================================

GEN_PY = 15
GEN_PH = 49

def limpiar_area_trabajo():
    oled.fill_rect(0, GEN_PY, 128, GEN_PH, 0)

def dibujar_barra_estado():
    lock.acquire()
    vs = datos_multi['vsys']
    lock.release()
    oled.fill_rect(0, 0, 128, 14, 0)
    oled.text("{:02d}:{:02d}:{:02d}".format(horas, minutos, segundos), 0, 0, 1)
    oled.text("{:.1f}V".format(vs), 92, 0, 1)
    oled.hline(0, 14, 128, 1)

def math_to_screen(xm, ym, xmin, xmax, ymin, ymax,
                   px=0, py=GEN_PY, pw=128, ph=GEN_PH):
    nx = (xm - xmin) / (xmax - xmin) if xmax != xmin else 0.0
    ny = (ym - ymin) / (ymax - ymin) if ymax != ymin else 0.0
    xp = px + int(nx * (pw - 1))
    yp = py + int((1.0 - ny) * (ph - 1))
    xp = max(px, min(px + pw - 1, xp))
    yp = max(py, min(py + ph - 1, yp))
    return xp, yp

def draw_axes(xmin, xmax, ymin, ymax, px, py, pw, ph):
    if xmin <= 0 <= xmax:
        ax, _ = math_to_screen(0, ymin, xmin, xmax, ymin, ymax, px, py, pw, ph)
        oled.vline(ax, py, ph, 1)
    if ymin <= 0 <= ymax:
        _, ay = math_to_screen(xmin, 0, xmin, xmax, ymin, ymax, px, py, pw, ph)
        oled.hline(px, ay, pw, 1)

def draw_info(nombre, x_val, y_val, x0=82):
    oled.text(nombre[:6],                 x0, GEN_PY +  0, 1)
    oled.text("X(t):",                    x0, GEN_PY +  9, 1)
    oled.text("{:.2f}".format(x_val)[:6], x0, GEN_PY + 18, 1)
    oled.text("Y(t):",                    x0, GEN_PY + 29, 1)
    oled.text("{:.2f}".format(y_val)[:6], x0, GEN_PY + 38, 1)

# =============================================================================
# 11. FUNCIONES DE DISPLAY POR MODO
# =============================================================================

def mostrar_voltimetro():
    lock.acquire(); v = datos_multi['voltaje']; lock.release()
    limpiar_area_trabajo(); dibujar_barra_estado()
    oled.text("VOLTIMETRO", 16, 17, 1); oled.hline(0, 26, 128, 1)
    oled.text("Volt:", 0, 30, 1); oled.text("{:.4f} V".format(v), 0, 42, 1)
    oled.show(); print("Voltimetro: {:.4f} V".format(v))

def mostrar_ohmetro():
    lock.acquire(); rx_str = datos_multi['rx_str']; lock.release()
    limpiar_area_trabajo(); dibujar_barra_estado()
    oled.text("OHMETRO", 28, 17, 1); oled.hline(0, 26, 128, 1)
    oled.text("Ohm:", 0, 30, 1); oled.text("{} Ohm".format(rx_str), 0, 42, 1)
    oled.show(); print("Ohmetro: {} Ohm".format(rx_str))

def mostrar_amperimetro():
    lock.acquire(); i_ma = datos_multi['corriente_ma']; lock.release()
    limpiar_area_trabajo(); dibujar_barra_estado()
    oled.text("AMPERIMETRO", 8, 17, 1); oled.hline(0, 26, 128, 1)
    oled.text("Amp:", 0, 30, 1); oled.text("{:.3f} mA".format(i_ma), 0, 42, 1)
    oled.show(); print("Amperimetro: {:.3f} mA".format(i_ma))

def mostrar_dht22():
    lock.acquire()
    t_dht = datos_multi['temp_dht']; h_dht = datos_multi['hum_dht']
    t_int = datos_multi['temp_int']; vs    = datos_multi['vsys']
    lock.release()
    limpiar_area_trabajo(); dibujar_barra_estado()
    oled.text("DHT22+TEMP", 16, 16, 1); oled.hline(0, 25, 128, 1)
    oled.text("T:{:.1f}C H:{:.1f}%".format(t_dht, h_dht), 0, 28, 1)
    oled.text("CPU:{:.1f}C".format(t_int), 0, 40, 1)
    oled.text("VSYS:{:.2f}V".format(vs), 0, 52, 1)
    oled.show()
    print("DHT:{:.1f}C {:.1f}% | CPU:{:.1f}C | VSYS:{:.2f}V".format(t_dht, h_dht, t_int, vs))

# =============================================================================
# 12. CORE 1
# =============================================================================

def core1_tareas():
    global modo_actual, flag_timer
    ultimo_dht = 0; ultimo_vsys = 0
    gen_inicializado = False; gen_layout = None
    buf_yp = buf_xp = None
    px = py = pw = ph = 0; xmin = xmax = ymin = ymax = 0.0
    nombre = ""; x0p = 64

    while True:
        lock.acquire(); transicion = en_transicion; lock.release()
        if transicion: utime.sleep_ms(50); continue

        lock.acquire(); modo = modo_actual; lock.release()

        ahora = utime.ticks_ms()
        if utime.ticks_diff(ahora, ultimo_vsys) > 10000:
            vs = leer_vsys()
            lock.acquire(); datos_multi['vsys'] = vs; lock.release()
            ultimo_vsys = ahora

        if flag_timer: flag_timer = False

        if modo == 1:
            lock.acquire(); activo = datos_gen['activo']; lock.release()
            if not activo: gen_inicializado = False; utime.sleep_ms(50); continue

            lock.acquire(); hay_nuevo = datos_gen['nuevo']; lock.release()
            if not hay_nuevo: continue

            if not gen_inicializado:
                lock.acquire()
                gen_layout = datos_gen['layout']
                xmin = datos_gen['xmin']; xmax = datos_gen['xmax']
                ymin = datos_gen['ymin']; ymax = datos_gen['ymax']
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
                    x0p = max(0, min(127, int(nx0 * 127)))
                gen_inicializado = True

            lock.acquire()
            fx = datos_gen['fx']; t_actual = datos_gen['t']; datos_gen['nuevo'] = False
            lock.release()

            if gen_layout == 'fullscreen':
                _, yp = math_to_screen(0, fx, 0, 1, ymin, ymax, px, py, pw, ph)
                for i in range(pw - 1): buf_yp[i] = buf_yp[i + 1]
                buf_yp[pw - 1] = yp
                limpiar_area_trabajo()
                draw_axes(xmin, xmax, ymin, ymax, px, py, pw, ph)
                for col in range(1, pw): oled.line(px+col-1, buf_yp[col-1], px+col, buf_yp[col], 1)
                dibujar_barra_estado(); oled.show()

            elif gen_layout == 'split':
                _, yp = math_to_screen(0, fx, 0, 1, ymin, ymax, px, py, pw, ph)
                for i in range(pw - 1): buf_yp[i] = buf_yp[i + 1]
                buf_yp[pw - 1] = yp
                limpiar_area_trabajo()
                draw_axes(xmin, xmax, ymin, ymax, px, py, pw, ph)
                oled.vline(80, GEN_PY, GEN_PH, 1)
                draw_info(nombre, t_actual, fx, x0=82)
                for col in range(1, pw): oled.line(px+col-1, buf_yp[col-1], px+col, buf_yp[col], 1)
                dibujar_barra_estado(); oled.show()

            elif gen_layout == 'portrait':
                nx = (fx - ymin) / (ymax - ymin) if ymax != ymin else 0.5
                xp_v = max(0, min(127, int(nx * 127)))
                for i in range(GEN_PH - 1): buf_xp[i] = buf_xp[i + 1]
                buf_xp[GEN_PH - 1] = xp_v
                limpiar_area_trabajo()
                oled.vline(x0p, GEN_PY, GEN_PH, 1)
                for row in range(1, GEN_PH): oled.line(buf_xp[row-1], GEN_PY+row-1, buf_xp[row], GEN_PY+row, 1)
                dibujar_barra_estado(); oled.show()

        elif modo == 2:
            gen_inicializado = False
            v = leer_voltaje()
            lock.acquire(); datos_multi['voltaje'] = v; lock.release()
            mostrar_voltimetro(); utime.sleep_ms(200)

        elif modo == 3:
            gen_inicializado = False
            rx = leer_resistencia()
            lock.acquire(); datos_multi['rx'] = rx; datos_multi['rx_str'] = formatear_resistencia(rx); lock.release()
            mostrar_ohmetro(); utime.sleep_ms(200)

        elif modo == 4:
            gen_inicializado = False
            i_ma = leer_corriente()
            lock.acquire(); datos_multi['corriente_ma'] = i_ma; lock.release()
            mostrar_amperimetro(); utime.sleep_ms(200)

        elif modo == 5:
            gen_inicializado = False
            t_int = leer_temp_interna()
            ahora = utime.ticks_ms()
            if utime.ticks_diff(ahora, ultimo_dht) > 2000:
                t_dht, h_dht = leer_dht22(); vs = leer_vsys()
                lock.acquire()
                if t_dht is not None: datos_multi['temp_dht'] = t_dht; datos_multi['hum_dht'] = h_dht
                datos_multi['temp_int'] = t_int; datos_multi['vsys'] = vs
                lock.release()
                ultimo_dht = ahora; ultimo_vsys = ahora
            mostrar_dht22(); utime.sleep_ms(500)

# =============================================================================
# 13. CORE 0 — LOOP GENERADOR
# =============================================================================

def core0_generador_loop(nmax, lista_an, lista_bn):
    t_actual = 0.0; delta_t = 0.05

    while True:
        if solicitud_cambio or solicitud_reset:
            GPIO_SALP(0)
            lock.acquire(); datos_gen['activo'] = False; datos_gen['nuevo'] = False; lock.release()
            return

        while True:
            lock.acquire(); pendiente = datos_gen['nuevo']; lock.release()
            if not pendiente: break
            if solicitud_cambio or solicitud_reset:
                GPIO_SALP(0)
                lock.acquire(); datos_gen['activo'] = False; datos_gen['nuevo'] = False; lock.release()
                return

        t_actual += delta_t
        fx = fourier_eval(t_actual, nmax, lista_an, lista_bn)

        # Clamp y mapeo DAC — Funcion Exponencial: rango [0, 1.5]
        fx_clamp  = max(0.0, min(1.5, fx))
        valor_dac = int((fx_clamp / 1.5) * 255)
        GPIO_SALP(valor_dac)

        lock.acquire()
        datos_gen['fx'] = fx; datos_gen['t'] = t_actual; datos_gen['nuevo'] = True
        lock.release()

# =============================================================================
# 14. EJECUTAR GENERADOR
# =============================================================================

def ejecutar_generador():
    global solicitud_cambio, solicitud_reset
    init_dac()
    nmax = 50
    lista_an, lista_bn = precalcular_coeficientes(nmax)

    xmin, xmax = -0.5, 1.5
    ymin, ymax =  0.0, 1.5
    nombre     = "EXPON"

    limpiar_area_trabajo(); dibujar_barra_estado()
    oled.text("GENERADOR", 28, 17, 1); oled.hline(0, 26, 128, 1)
    oled.text("Elige en", 20, 32, 1); oled.text("consola...", 14, 44, 1)
    oled.show()

    print("\n" + "=" * 42)
    print("  GENERADOR — EJ4 FUNCION EXPONENCIAL")
    print("  T=0.5  w0=4pi  C=1-e^(-0.5)")
    print("=" * 42)
    print("  [1] Fullscreen  [2] Split  [3] Portrait")
    print("=" * 42)

    while True:
        if solicitud_cambio or solicitud_reset: release_dac(); return
        if select.select([sys.stdin], [], [], 0)[0]:
            op = sys.stdin.readline().strip()
            if op in ("1", "2", "3"): break
            else: print("Opcion invalida. Escriba 1, 2 o 3.")
        utime.sleep_ms(100)

    lock.acquire(); solicitud_cambio = False; solicitud_reset = False; lock.release()
    print("Layout: {}".format({1:"Fullscreen",2:"Split",3:"Portrait"}[int(op)]))

    layout_map = {"1": "fullscreen", "2": "split", "3": "portrait"}
    lock.acquire()
    datos_gen['layout'] = layout_map[op]; datos_gen['nombre'] = nombre
    datos_gen['xmin'] = xmin; datos_gen['xmax'] = xmax
    datos_gen['ymin'] = ymin; datos_gen['ymax'] = ymax
    datos_gen['nuevo'] = False; datos_gen['activo'] = True
    lock.release()

    core0_generador_loop(nmax, lista_an, lista_bn)
    release_dac()

# =============================================================================
# 15. CORE 0 — LOGICA PRINCIPAL
# =============================================================================

NOMBRES_MODO = {1:"GENERADOR", 2:"VOLTIMETRO", 3:"OHMETRO", 4:"AMPERIMETRO", 5:"DHT22"}

def core0_principal():
    global modo_actual, solicitud_cambio, solicitud_reset, en_transicion

    while True:
        if solicitud_cambio or solicitud_reset:
            lock.acquire(); en_transicion = True; lock.release()
            utime.sleep_ms(80)
            if solicitud_reset:
                nuevo_modo = 1
            else:
                lock.acquire(); m = modo_actual; lock.release()
                nuevo_modo = (m % 5) + 1
            limpiar_area_trabajo(); dibujar_barra_estado()
            oled.text("Cambiando a:", 10, 20, 1)
            oled.text(NOMBRES_MODO[nuevo_modo], 10, 35, 1)
            oled.show(); utime.sleep_ms(900)
            lock.acquire()
            modo_actual = nuevo_modo; solicitud_cambio = False
            solicitud_reset = False; en_transicion = False
            lock.release()
            print(">>> MODO {}: {}".format(nuevo_modo, NOMBRES_MODO[nuevo_modo]))

        lock.acquire(); modo = modo_actual; lock.release()
        if modo == 1: ejecutar_generador()
        else: utime.sleep_ms(100)

# =============================================================================
# 16. MAIN
# =============================================================================

def main():
    print("=" * 50)
    print("  LAB 8 — EJ4 FUNCION EXPONENCIAL")
    print("  GP14 btn_set  : avanza modo (1->2->3->4->5->1)")
    print("  GP15 btn_reset: vuelve a Modo 1")
    print("=" * 50)

    oled.fill(0)
    oled.text("LAB 8 - EJ4", 16, 4, 1)
    oled.text("EXPONENCIAL", 16, 16, 1)
    oled.hline(0, 28, 128, 1)
    oled.text("T=0.5 w0=4pi", 0, 35, 1)
    oled.text("SET->modo RST->1", 0, 50, 1)
    oled.show()
    utime.sleep(2)

    _thread.start_new_thread(core1_tareas, ())
    print(">>> MODO 1: Generador (por defecto)")
    core0_principal()

main()
