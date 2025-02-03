#!/usr/bin/python
# -*- coding:utf-8 -*-

from os import popen
from time import sleep, time
from datetime import datetime
from math import sqrt
from influxdb import InfluxDBClient
import ADS1263
import RPi.GPIO as GPIO

from paho.mqtt import client as mqtt_client

# class RaspberryPi:
#     # Pin definition
#     RST_PIN = 18
#     CS_PIN = 22
#     DRDY_PIN = 17

#     def __init__(self):
#         # SPI device, bus = 0, device = 0
#         import spidev
#         import RPi.GPIO

#         self.GPIO = RPi.GPIO
#         self.SPI = spidev.SpiDev(0, 0)

#     def digital_write(self, pin, value):
#         self.GPIO.output(pin, value)

#     def digital_read(self, pin):
#         return self.GPIO.input(pin)

#     def delay_ms(self, delaytime):
#         sleep(delaytime / 1000.0)

#     def spi_writebyte(self, data):
#         self.SPI.writebytes(data)

#     def spi_readbytes(self, reg):
#         return self.SPI.readbytes(reg)

#     def module_init(self):
#         self.GPIO.setmode(self.GPIO.BCM)
#         self.GPIO.setwarnings(False)
#         self.GPIO.setup(self.RST_PIN, self.GPIO.OUT)
#         self.GPIO.setup(self.CS_PIN, self.GPIO.OUT)

#         self.GPIO.setup(self.DRDY_PIN, self.GPIO.IN, pull_up_down=self.GPIO.PUD_UP)
#         self.SPI.max_speed_hz = 2000000
#         self.SPI.mode = 0b01
#         return 0

#     def module_exit(self):
#         self.SPI.close()
#         self.GPIO.output(self.RST_PIN, 0)
#         self.GPIO.output(self.CS_PIN, 0)
#         self.GPIO.cleanup()


# hostname = popen("uname -n").read().strip()
# implementation = RaspberryPi()

# for func in [x for x in dir(implementation) if not x.startswith('_')]:
#     setattr(sys.modules[__name__], func, getattr(implementation, func))

debug = True

# ADS1263
# samples = 200
# precision = int(2)
sensors_count = 9
time_elapsed = 0

# MQTT
port = 1883
broker = "192.168.10.18"
client_id = "rohan-python-sct013"


def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code {}\n".format(rc))

    def on_disconnect(client, userdata, rc):
        client.connect(broker, port, keepalive=60)

    client = mqtt_client.Client(client_id)
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.connect(broker, port, keepalive=60)
    return client


def publish(client, topic, msg):
    result = client.publish(topic, msg)
    status = result[0]
    if status == 0:
        print("Sent `{}` to topic `{}`".format(msg, topic))
    else:
        raise RuntimeError("Failed to send message to topic {}".format(topic))


def measure(ADC):
    while True:
        # start timer for kWh calculations
        start_time = time.time()

        channelList = [0] * sensors_count  # The channel must be less than 10
        while 1:
            ADC_Value = ADC.ADS1263_GetAll(channelList)  # get ADC1 value
            for i in channelList:
                if ADC_Value[i] >> 31 == 1:
                    print(
                        "ADC1 IN%d = -%lf"
                        % (
                            i,
                            (REF_VOLTAGE * 2 - ADC_Value[i] * REF_VOLTAGE / 0x80000000),
                        )
                    )
                else:
                    print(
                        "ADC1 IN%d = %lf"
                        % (i, (ADC_Value[i] * REF_VOLTAGE / 0x7FFFFFFF))
                    )  # 32bit
            for i in channelList:
                print("\33[2A")

        # count = int(0)
        # data = [0] * sensors_count
        # peak = [0] * sensors_count
        # IrmsA = [0] * sensors_count
        # ampsA = [0] * sensors_count
        # voltage = float(0)
        # kW = float(0)

        # while count < samples:
        #     count += 1

        #     for i in range(0, sensors_count):
        #         data[i] = abs(adc1.read_adc(i, gain=GAIN_A))

        #         # see if you have a new peak
        #         if data[i] > peak[i]:
        #             peak[i] = data[i]

        #     # Calibrated for SCT-013 30A/1V
        #     for i in range(0, sensors_count):
        #         IrmsA[i] = float(peak[i] / float(2047) * 30)
        #         IrmsA[i] = round(IrmsA[i], precision)
        #         ampsA[i] = IrmsA[i] / sqrt(2)
        #         ampsA[i] = round(ampsA[i], precision)

        # # Calculate total AMPS from all sensors and convert to kW
        # kW = 0.0

        # # convert kW to kW / hour (kWh)
        # kWh = round((kW * time_elapsed) / 3600, 8)

        # iso = datetime.now(datetime.timezone.utc).isoformat() + "Z"

        # json_data = [
        #     {
        #         "measurement": "current",
        #         "tags": {},
        #         "time": iso,
        #         "fields": {
        #             "voltage": LINEV,
        #             "kWh": kWh,
        #             "kW_0": kW[0],
        #             "A_0": amps[0],
        #         },
        #     }
        # ]

        # client = InfluxDBClient(
        #     "192.168.10.18", 8086, "", "", "ampread", timeout=60, retries=0
        # )
        # try:
        #     client.write_points(json_data)
        # except ConnectionError:
        #     print("influxdb server not responding")
        #     continue

        # humidity, temperature = Adafruit_DHT.read_retry(Adafruit_DHT.DHT22, 4)
        # print("Temp: {0:0.1f} C  Humidity: {1:0.1f} %".format(temperature, humidity))
        # publish(client, "/sensors/living_room/dht22/temperature", temperature)
        # publish(client, "/sensors/living_room/dht22/humidity", humidity)

        delay = 10
        if debug:
            delay = 30
            print("Measurement done, sleeping for {}s".format(delay))
        sleep(delay)

        time_elapsed = time() - start_time


if __name__ == "__main__":
    try:
        print("Initializing ADS1263")
        ADC = ADS1263.ADS1263()

        # The faster the rate, the worse the stability
        # and the need to choose a suitable digital filter(REG_MODE1)
        if ADC.ADS1263_init_ADC1("ADS1263_400SPS") == -1:
            exit()
        ADC.ADS1263_SetMode(0)  # 0 is singleChannel, 1 is diffChannel

        print("Running measurements loop")
        while True:
            client = connect_mqtt()
            client.loop_start()

            while True:
                try:
                    measure()
                except IOError as e:
                    print("Exception: ", error)
                except Exception as error:
                    print("Exception: ", error)
                    break

            delay = 300
            if debug:
                delay = 60
            print("Retrying in {}s".format(delay))
            sleep(delay)

    except Exception as error:
        print("Exception: ", error)
        ADC.ADS1263_Exit()
        exit()
