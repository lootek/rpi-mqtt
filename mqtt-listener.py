import paho.mqtt.client as mqtt
from influxdb import InfluxDBClient
import datetime
import logging
import pprint

logging.basicConfig(level=logging.INFO)

def persists(msg):
    if msg.payload == "nan":
        logging.info("Skipping invalid measurement")
        pass

    current_time = datetime.datetime.utcnow().isoformat()
    json_body = [
        {
            "measurement": msg.topic,
            "tags": {},
            "time": current_time,
            "fields": {
                "value": msg.payload
            }
        }
    ]
    logging.info(json_body)
    influx_client.write_points(json_body)


influx_client = InfluxDBClient('influxdb', 8086, database='iot')

client = mqtt.Client()
client.on_connect = lambda self, mosq, obj, rc: self.subscribe("#")
client.on_message = lambda client, userdata, msg: persists(msg)
client.connect('mosquitto', 1883, 60)
client.loop_forever()
