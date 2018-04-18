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

from multiprocessing import Pool

import commands

_pool = None


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
    global _pool
    COMMAND_QUEUE = command_queue

    system_info = system_info_temp

    _pool = Pool(processes=4)

    try:
        app.run(port=PORT, debug=isDebug, threaded=True, host=HOSTNAME)
    except KeyboardInterrupt:
        _pool.close()
        _pool.join()


def getJson():
    state = OrderedDict([
        ('currentSpeed', system_info.getCurrentSpeed()),
        ('obstacleDistance', system_info.getObstacleDistance()),
        ('tickDifference', system_info.getTicksLeft() - system_info.getTicksRight())
    ])
    settings = OrderedDict([
        ('userSetSpeed', system_info.getUserSetSpeed()),
        ('safeDistance', system_info.getSafeDistance()),
        ('criticalDistance', system_info.getCriticalDistance()),
        ('alertDistance', system_info.getAlertDistance())
    ])
    res = json.dumps({
        "state": state,
        "settings": settings
    })
    return res


@app.route('/api/system-info', methods=['GET'])
def get_settings():
    """
      handles getting the user settings
      by returing the user settings from ordered tuples
      of the settings so JSON does not serialize and re-order data
    """
    req = _pool.apply_async(getJson)
    res = req.get()
    resDict = json.loads(res)
    settings = resDict['settings']
    state = resDict['state']

    temp = json.dumps({
        "state": state,
        "settings": settings
    })
    return temp


@app.route('/api/turn-off', methods=['POST'])
def turn_off():
    """
      handles POST request of user settings from fetch,
      and returns the render_template index.html
      because we do not need immediate access to the data
      after the POST request
    """
    COMMAND_QUEUE.put(commands.TurnOffCommand())
    _pool.apply_async(powerOff)
    print('POWERRRR', system_info.getPower())
    json = jsonify({
        "power": False
    })
    json.status_code = 200
    return json


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
    req = _pool.apply_async(getJson)
    res = req.get()
    resDict = json.loads(res)
    settings = resDict['settings']
    return json.dumps({
        "settings": settings
    })


def getPower():
    return system_info.getPower()

def powerOff():
    system_info.setPower(False)


@app.route('/api/power-status', methods=['GET'])
def get_power():
    """
      handles GET request for the power status,
      by returing the power status jsonified
    """
    req = _pool.apply_async(getPower)
    res = req.get()
    json = jsonify({
        "power": res
    })
    json.status_code = 200
    return json

