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
    match = re.match(r"/sensors/([^/]+)/(lastwill)", path)
    if match:
        return match.group(1), match.group(2), None

    match = re.match(r"/sensors/([^/]+)/(status)", path)
    if match:
        return match.group(1), match.group(2), None

    match = re.match(r"/sensors/([^/]+)/(wifi)/(ip)", path)
    if match:
        return match.group(1), match.group(2), match.group(3)

    match = re.match(r"/sensors/([^/]+)/([^/]+)/([^/]+)", path)
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
        pass

    location, sensor, measurement = extract_sensor_data(msg.topic)
    if not measurement:
        measurement = sensor
        sensor = "system"

    json_body = [
        {
            "measurement": sensor,
            "time": current_time,
            "tags": {
                "location": location,
            },
            "fields": {
                measurement: value,
            },
        }
    ]
    logging.info(json_body)

    try:
        influx_client_mqtt.write_points(json_body)
    except Exception as error:
        print("Failed to write to influxdb: ", error)
        print("Data topic: ", msg.topic)
        print("Data payload: ", msg.payload)
        pass


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
