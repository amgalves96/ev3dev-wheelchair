#!/usr/bin/env python3

import paho.mqtt.client as mqtt

# This is the Publisher

client = mqtt.Client("EV3")
client.connect("192.168.0.107", 1883)
client.publish("TopicTest", "Hello world!");
client.disconnect();
