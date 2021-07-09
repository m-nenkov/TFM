# Open_MV_CAM - By: Martin - feb. 10 2021

# Valores para la detección de la esfera
RADIO_MAX = 7
RADIO_MIN = 2

import sensor, image, time
from pyb import Pin, SPI, UART, I2C
import utime

# cs  = Pin("P3", Pin.OUT_OD)
# rst = Pin("P7", Pin.OUT_PP)
# rs  = Pin("P8", Pin.OUT_PP)


# Variable circulo "falso"
class circulo_falso_t:
    def __init__(self, posX, posY):
        self.posx = posX
        self.posy = posY
    def x(self):
        return self.posx
    def y(self):
        return self.posy

circulo_falso = circulo_falso_t(180,150)

# Inicialización de la cámara
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.HQQVGA)
sensor.skip_frames(time = 2000)

# Inicialización de la comunicación
# spi = SPI(2, SPI.SLAVE,  polarity=0, phase=0)
# i2c = I2C(2, I2C.SLAVE, addr=0x42)
uart = UART(1, 115200);
# rs = Pin("P8", Pin.OUT_PP)
# rs.high()

def send_data(c):
    # Se envía la posción del círculo
    bytx = c.x().to_bytes(1,'little')
    byty = c.y().to_bytes(1,'little')
    # print(c.x())
    byt = bytearray(2)
    byt = byty + bytx
    # Se activa (a nivel bajo) la señal ReadyToSend
    # rs.low()
    try:
        # print(byt)
        # spi.send(byt, timeout=10)
        # i2c.send(byt, timeout=10)
        uart.write(byt);
    except:
        utime.sleep_us(500)
        # print("com fallida")
    # Se desactiva la señal ReadyToSend
    # rs.high()

def en_rango(radio):
    if ((radio>=RADIO_MIN) and (radio<=RADIO_MAX)):
        return True
    return False


while(True):
    # Se toma una imagen
    img = sensor.snapshot().lens_corr(1.8)

    # Se obtienen los círculos
    list_c = img.find_circles(threshold = 2000, x_margin = 10, y_margin = 10, r_margin = 10,    # Se ajustan los parámetros para que sólo detecte el cículo
                                         r_min = 1, r_max = 20, r_step = 2)                     # correspondiente a la esfera
    # Si no se han detectado círculos
    if not list_c:
        send_data(circulo_falso)
    else:
        for c in list_c:
            img.draw_circle(c.x(), c.y(), c.r(), color = (255, 0, 0))
            # Se comprueba que el tamaño del círculo es el deseado
            if(en_rango(c.r())):
                img.draw_circle(c.x(), c.y(), c.r(), color = (0, 255, 0))
                # Se envía el círculo detectado, o no, al microncontrolador
                send_data(c)
                break
            else:
                send_data(circulo_falso)


