import paho.mqtt.client as mqtt
from influxdb import InfluxDBClient
import datetime
import logging
import re

logging.basicConfig(level=logging.INFO)

influx_client_iot = InfluxDBClient("influxdb", 8086, database="iot")
influx_client_mqtt = InfluxDBClient("influxdb", 8086, database="mqtt")


def save_msg_legacy(msg):
    if msg.payload == "nan":
        logging.info("Skipping invalid measurement")
        pass

    current_time = datetime.datetime.utcnow().isoformat()
    json_body = [
        {
            "measurement": msg.topic,
            "tags": {},
            "time": current_time,
            "fields": {"value": msg.payload},
        }
    ]
    logging.info(json_body)
    influx_client_iot.write_points(json_body)


def extract_sensor_data(path):
    match = re.match(r"/sensors/([^/]+)/(lastwill)")
    if match:
        return match.group(1), match.group(2), None

    match = re.match(r"/sensors/([^/]+)/(status)")
    if match:
        return match.group(1), match.group(2), None

    match = re.match(r"/sensors/([^/]+)/(wifi)/(ip)")
    if match:
        return match.group(1), match.group(2), match.group(3)

    raise RuntimeError("Failed to parse message {}".format(path))


def save_msg(msg):
    if msg.payload == "nan":
        logging.info("Skipping invalid measurement")
        pass

    current_time = datetime.datetime.now(datetime.timezone.utc).isoformat()

    value = msg.payload
    try:
        value = float(msg.payload)
    except:
        logging.info("Couldn't convert {} to float".format(msg.payload))

    location, sensor, measurement = extract_sensor_data(msg.topic)
    if not measurement:
        measurement = sensor
        sensor = "system"

    json_body = [
        {
            "measurement": measurement,
            "time": current_time,
            "tags": {
                "location": location,
                "sensor": sensor,
            },
            "fields": {
                measurement: value,
                "location": location,
                "sensor": sensor,
            },
        }
    ]
    logging.info(json_body)
    influx_client_mqtt.write_points(json_body)


def save_msg_wrapper(msg):
    save_msg_legacy(msg)
    save_msg(msg)


client = mqtt.Client()
client.on_connect = lambda self, mosq, obj, rc: self.subscribe("#")
client.on_disconnect = lambda self, mosq, obj, rc: client.connect(
    "mosquitto", 1883, keepalive=60
)
client.on_message = lambda client, userdata, msg: save_msg_wrapper(msg)
client.connect("mosquitto", 1883, keepalive=60)
client.loop_forever()
