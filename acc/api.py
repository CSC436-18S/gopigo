"""
This is the module of the program that contains all of the functions related to
the webserver that hosts the user interface for the ACC.
"""
import struct
import socket
import fcntl
from flask import Flask, render_template, request, render_template, jsonify
from flask_restful import Api, Resource
from flask_cors import CORS
import json
from collections import OrderedDict

import multiprocessing

import commands


def get_ip_address(ifname):
    """
      used to grab correct network interface from for the hostname
      instead of localhost, which allows for mobile device connection
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,
        struct.pack('256s', ifname[:15])
    )[20:24])


"""
  - create application instance of Flask
  - create an API instance of the Flask application
  - enable CORS to so web can correctly communicate with API
  - create PORT and HOSTNAME that are sent to the index.html file
    for requests
"""
app = Flask(__name__)
api = Api(app)
CORS(app)
PORT = 8080

COMMAND_QUEUE = None

"""
  wlp2s0 is more standard version then wlan0 and eth0,
  if wlp2s0 cannot be found, default to wlan0
"""
try:
    HOSTNAME = get_ip_address('wlp2s0')
except IOError:
    HOSTNAME = get_ip_address('wlan0')


@app.route('/')
def index():
    """
      root path of application that renders the index.html
      file from the templates folder
    """
    return render_template('index.html', hostname=HOSTNAME, port=PORT)


def run(isDebug, command_queue, system_info_temp):
    """
      function that handles starting the application on port 8080
    """
    global COMMAND_QUEUE
    global system_info
    COMMAND_QUEUE = command_queue

    system_info = system_info_temp

    app.run(port=PORT, debug=isDebug, threaded=True, host=HOSTNAME)


@app.route('/api/system-info', methods=['GET'])
def get_settings():
    """
      handles getting the user settings
      by returing the user settings from ordered tuples
      of the settings so JSON does not serialize and re-order data
    """
    state = OrderedDict([
        ('currentSpeed', system_info.getCurrentSpeed()),
        ('obstacleDistance', system_info.getObstacleDistance())])
    settings = OrderedDict([
        ('userSetSpeed', system_info.getUserSetSpeed()),
        ('safeDistance', system_info.getSafeDistance())
    ])
    res = json.dumps({
        "state": state,
        "settings": settings
    })
    return res


@app.route('/api/turn-off', methods=['POST'])
def turn_off():
    """
      handles POST request of user settings from fetch,
      and returns the render_template index.html
      because we do not need immediate access to the data
      after the POST request
    """
    system_info.setPower(False)

    COMMAND_QUEUE.put(commands.TurnOffCommand())

    return json.dumps(system_info.__dict__)


@app.route('/api/user-settings', methods=['POST'])
def post_settings():
    """
      handles POST request of user settings from fetch,
      and returns the render_template index.html
      because we do not need immediate access to the data
      after the POST request
    """
    dataDict = json.loads(request.data)
    speed = dataDict['speed']
    distance = dataDict['distance']
    system_info.setUserSetSpeed(speed)
    system_info.setSafeDistance(distance)

    COMMAND_QUEUE.put(commands.ChangeSettingsCommand(speed, distance))

    return json.dumps(system_info.__dict__)


@app.route('/api/power-status', methods=['GET'])
def get_power():
    """
      handles GET request for the power status,
      by returing the power status jsonified
    """
    res = jsonify({
        "power": system_info.getPower()
    })
    res.status_code = 200
    return res


@app.route('/api/power-status', methods=['POST'])
def post_power():
    """
      handles POST request for setting the power from fetch,
      since the fetch needs the value of the power status
      after setting it, we return the power status,
      and not the render template
    """
    dataDict = json.loads(request.data)
    global power
    power = dataDict['power']
    return jsonify(power)
