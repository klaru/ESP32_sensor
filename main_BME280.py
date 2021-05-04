from machine import Pin, I2C, reset, unique_id, freq, UART
from sys import platform
from time import sleep, time, localtime
from ubinascii import hexlify
from boot import load_config
from umqttsimple import MQTTClient
from ssd1306 import SSD1306_I2C
from ssd1306_flipped import SSD1306_I2C_FLIPPED
from bme280 import BME280
from sgp30 import SGP30
from writer_minimal import Writer
from pms7003 import Pms7003
from mhz19bsensor import MHZ19BSensor
import Arial11
import Arial15
import tsl2561

QOS=1
# select GPIO pins
if platform == "esp32":
    pin_scl = 22
    pin_sda = 21
    pin_sens = 16
elif platform == "esp8266":                 
    pin_scl = 5
    pin_sda = 4
    pin_sens = 2

pms_set_pin = 26
pms_enable = Pin(pms_set_pin)

frequency = 100000
BME280_OSAMPLE_16 = 5
OP_SINGLE_HRES2 = 0x21

display_address1 = 0x3C
bmx_address1 = 0x76
display_address2 = 0x3D
light_address = 0x39
sgp30_address = 0x58

display_present1 = False
bmx_present1 = False
display_present2 = False
light_present = False
sgp30_present = False 

i2c = I2C(scl=Pin(pin_scl), sda=Pin(pin_sda), freq=frequency)
devices = i2c.scan()


for device in devices:
    if device == display_address1:
        display_present1 = True
    if device == bmx_address1:
        bmx_present1 = True
    if device == display_address2:
        display_present2 = True   
    if device == light_address:
        light_present = True
    if device == sgp30_address:
        sgp30_present = True
            
if display_present1:
    oled1 = SSD1306_I2C_FLIPPED(128, 64, i2c, addr=display_address1)
if display_present2:
    oled2 = SSD1306_I2C(64, 48, i2c, addr=display_address2)

if bmx_present1:
    sensor = BME280(mode=BME280_OSAMPLE_16, address=bmx_address1,i2c=i2c)

if light_present:
    light_sensor = tsl2561.device(i2c, i2cAddr = light_address)
    
topic1 = b'Temp_Bedroom'
topic2 = b'Humid_Bedroom'
topic3 = b'Press_Bedroom'
topic4 = b'Light_Bedroom'
topic5 = b'CO2_Bedroom'
topic6 = b'TVOC_Bedroom'

client_id = hexlify(unique_id())

def connect_to_mqtt(config):
  client = MQTTClient(client_id, config['mqtt']['broker'])
  client.connect()
  print('Connected to %s MQTT broker' % (config['mqtt']['broker']))
  return client
        
def restart_and_reconnect():
  print('Failed to connect to MQTT broker. Restarting and reconnecting...')
  sleep(10)
  reset()
    
# Main loop that will run forever:
def main(config):
  
    try:
        client = connect_to_mqtt(config)
    except OSError:
        sleep(10)
        restart_and_reconnect()
        
    if display_present1:
        writer1 = Writer(oled1, Arial15)
    if display_present2:
        writer2 = Writer(oled2, Arial11)
    while True:
        light_sensor.init()
        light = light_sensor.getLux()   
        if display_present1:
            oled1.fill(0)
            oled1.show()            
        if display_present2:
            oled2.fill(0)
            oled2.show()            
        pms_enable.value(1)
#        uart1 = UART(1)
#        uart1.init(baudrate=9600, rx=1, bits=8, parity=None, stop=1)
#        pms = Pms7003(uart1)
#        pms.read()              
        pms_enable.value(0)
        uart2 = UART(2)
        uart2.init(baudrate=9600, tx=19, rx=18, bits=8, parity=None, stop=1)
        sensorCO2 = MHZ19BSensor(uart2)
        co2 = sensorCO2.measure()
        if bmx_present1:
            temp = float(sensor.temperature[:-3])
            humid = float(sensor.humidity[:-3])            
            press = float(sensor.pressure[:-3])    
        if (display_present1 and light > 0):       
            if bmx_present1:
                writer1.set_textpos(0,0)
                writer1.printstring("Temp: %.0f °C" % temp)
                writer1.set_textpos(16,0)
                writer1.printstring("Humid: %.0f %%" % humid)
                writer1.set_textpos(32,0)
                writer1.printstring("%.0f hPa" % press)   
                oled1.show()                
        if (display_present2 and light > 0):       
            if bmx_present1:
                writer2.set_textpos(0,0)
                writer2.printstring("Temp: %.0f °C" % temp)
                writer2.set_textpos(12,0)
                writer2.printstring("Humid: %.0f %%" % humid)
                writer2.set_textpos(24,0)
                writer2.printstring("%.0f hPa" % press)
            writer2.set_textpos(36,0)
            writer2.printstring("%.0f ppm" % co2)    
            oled2.show()
        if bmx_present1:
            print("Temperature: %.0f °C" % temp)
            print("Humidity: %.0f %%" % humid)             
            print("Pressure: %.0f hPa" % press)          
            print("CO2:  %.f ppm" % co2)
        try:        
            if bmx_present1:            
                client.publish(topic1, str("%.0f" % temp), qos=QOS)
                client.publish(topic2, str("%.0f" % humid), qos=QOS)
                client.publish(topic3, str("%.0f" % press), qos=QOS)             
        except OSError:
            restart_and_reconnect()

        sleep(5)
        print("Light Intensity: %.4s lux" % light)
        if sgp30_present:                           
            airquality = SGP30(i2c)
            print("eCO2: %d ppm" % airquality.co2_equivalent)
            print("TVOC: %d ppb" % airquality.total_organic_compound) 
        if (display_present1 and light > 0):
            oled1.fill(0)
            oled1.show()   
            if bmx_present1:
                writer1.set_textpos(0,0)
                writer1.printstring("Temp: %.0f °C" % temp)
                writer1.set_textpos(16,0)
                writer1.printstring("Humid: %.0f %%" % humid)
                writer1.set_textpos(32,0)
                writer1.printstring("%.0f hPa" % press)
                oled1.show()                
        if (display_present2 and light > 0):
            oled2.fill(0)
            oled2.show()
            writer2.set_textpos(0,0)
            writer2.printstring("light:%.4s lux" % light)            
            if sgp30_present:
                writer2.set_textpos(24,0)
                writer2.printstring("co2:%d ppm" % airquality.co2_equivalent)
                writer2.set_textpos(36,0)
                writer2.printstring("Tvoc:%d ppb" % airquality.total_organic_compound)                              
            oled2.show()
        try:
            client.publish(topic4, str(light), qos=QOS)
            client.publish(topic5, str(co2), qos=QOS)
            if sgp30_present:                           
                client.publish(topic6, str(airquality.total_organic_compound), qos=QOS)
        except OSError:
            restart_and_reconnect()
        sleep(5)
        if display_present1:
            oled1.fill(0)
            oled1.show()
        if display_present2:
            oled2.fill(0)
            oled2.show()      
        sleep(5)
            
if __name__ == "__main__":
    main(load_config())