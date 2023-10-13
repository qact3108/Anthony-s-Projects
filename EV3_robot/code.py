
import time
from motor_kit import Motor
from display import Display
from sensors import Color_sensor, Distance_sensor
import supervisor
import sys
import os
import microcontroller

supervisor.runtime.autoreload =False
"""
motorA = Motor('A')
motorC = Motor('C')


color = Color_sensor(1) # initialize object for Color sensor connected at port 1
#gyro = Gyroscope(2) # initialize object for gyroscope sensor connected at port 2
#distance = Distance_sensor(2) # initialize object for distance sensor connected at port 3

display = Display() #initialize object for Display

color.led_on()
"""
"""
while True:
    display.generate_text(20,40,"{}".format(color.get_value_415()))
    time.sleep(0.5)
    display.refresh()
"""
"""
a = 0
k = 2400
while True:
    #while(distance.get_distance() > 10.0 or distance.get_distance() == 0.0 ):
    while True:
        b = color.get_value_415() # reads intensity value for wavelength 415 using the color sensor
        if b >=k:  # if b > 100 write text "The distance is: distance value"
            if a == 0:
                 # while value for 415 wavelength is > 200 run the motor at 50%
                motorA.run(-0.65)
                motorC.run(-0.15)

                a = 1
        elif b< k:
            if a == 1:
                motorA.run(-0.15)
                motorC.run(-0.65)
                a = 0
    motorA.stop()
    motorC.stop()
    display.refresh()
    display.generate_text(20,40,"There is an Object at {} cm".format(distance.get_distance()))
    time.sleep(5)
    display.refresh()
    x = color.get_value_415()
    if x>k:
        a =0
    else:
        a = 1
    pass
"""

# SPDX-FileCopyrightText: 2023 Michał Pokusa
#
# SPDX-License-Identifier: Unlicense

import socketpool
import wifi

from adafruit_httpserver import Server, Request, Response, GET, POST

try:
    from secrets import secrets
except ImportError:
    print("WiFi secrets not found in secrets.py")
    raise

try:
    print("Connecting to {}...".format(secrets["ssid"]))
    wifi.radio.connect(secrets["ssid"], secrets["password"])
except:
    print("Error connecting to WiFi")
    raise

pool = socketpool.SocketPool(wifi.radio)
server = Server(pool, debug=True)


FORM_HTML_TEMPLATE = """
<html lang="en">
    <head>
        <title>Form with {enctype} enctype</title>
    </head>
    <body>
        <a href="/form?enctype=application/x-www-form-urlencoded">
            <button>Load <strong>application/x-www-form-urlencoded</strong> form</button>
        </a><br />
        <a href="/form?enctype=multipart/form-data">
            <button>Load <strong>multipart/form-data</strong> form</button>
        </a><br />
        <a href="/form?enctype=text/plain">
            <button>Load <strong>text/plain</strong> form</button>
        </a><br />

        <h2>Form with {enctype} enctype</h2>
        <form action="/form" method="post" enctype="{enctype}">
            <input type="file" id="file" name="file" accept=".txt,.py, image/jpeg" />
            <input type="submit" value="Submit">
        </form>
        {submitted_value}
    </body>
</html>
"""


@server.route("/form", [GET, POST])
def form(request: Request):
    """
    Serve a form with the given enctype, and display back the submitted value.
    """
    enctype = request.query_params.get("enctype", "text/plain")
    if request.method == POST:
        posted_value = request.form_data.get("file")

        if posted_value != None:
            f = open(posted_value, "r")

            content = f.read()
            print(content)
            f.close()
            fin = open('temp.py', 'w')
            fin.write(content)
            fin.flush()
            time.sleep(2)
            fin.close()
            microcontroller.reset()




    return Response(
        request,
        FORM_HTML_TEMPLATE.format(
            enctype=enctype,
            submitted_value=(
                f"<h3>Enctype: {enctype}</h3>\n<h3>Submitted form value: {posted_value}</h3>"
                if request.method == POST
                else ""
            ),
        ),
        content_type="text/html",
    )


server.serve_forever(str(wifi.radio.ipv4_address))
print("Connected to {}, Web server running on http://{}:80".format(secrets["ssid"], wifi.radio.ipv4_address))
