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

from settings import SystemInfo

"""
  used to grab correct network interface from for the hostname
  instead of localhost, which allows for mobile device connection
"""
def get_ip_address(ifname):
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
"""
  wlp2s0 is more standard version then wlan0 and eth0,
  if wlp2s0 cannot be found, default to wlan0
"""
try:
  HOSTNAME = get_ip_address('wlp2s0')
except IOError:
  HOSTNAME = get_ip_address('wlan0')

"""
  default values for needed variables for application,
  speed 0 so not moving, distance 9999 because this is
  technically safeDistance, power False because we don't
  want to start the rover immediately after the application
"""
system_info = SystemInfo(userSetSpeed=0, safeDistance=9999, currentSpeed=0, obstacleDistance=9999, power=True)

"""
  root path of application that renders the index.html
  file from the templates folder
"""
@app.route('/')
def index():
    return render_template('index.html', hostname=HOSTNAME, port=PORT)

"""
  function that handles starting the application on port 8080
"""
def run(isDebug):
  app.run(port=PORT, debug=isDebug, threaded=True, host=HOSTNAME)

"""
  handles getting the user settings
  by returing the user settings from ordered tuples
  of the settings so JSON does not serialize and re-order data
"""
@app.route('/api/system-info', methods=['GET'])
def get_settings():
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

"""
  handles POST request of user settings from fetch,
  and returns the render_template index.html
  because we do not need immediate access to the data
  after the POST request
"""
@app.route('/api/turn-off', methods=['POST'])
def turn_off():
  system_info.setPower(False)
  return json.dumps(system_info.__dict__)


"""
  handles POST request of user settings from fetch,
  and returns the render_template index.html
  because we do not need immediate access to the data
  after the POST request
"""
@app.route('/api/user-settings', methods=['POST'])
def post_settings():
  dataDict = json.loads(request.data)
  speed = dataDict['speed']
  distance = dataDict['distance']
  system_info.setUserSetSpeed(speed)
  system_info.setSafeDistance(distance)
  return json.dumps(system_info.__dict__)


"""
  handles GET request for the power status,
  by returing the power status jsonified
"""
@app.route('/api/power-status', methods=['GET'])
def get_power():
  res = jsonify({
    "power": system_info.getPower()
  })
  res.status_code = 200
  return res

"""
  handles POST request for setting the power from fetch,
  since the fetch needs the value of the power status
  after setting it, we return the power status,
  and not the render template
"""
@app.route('/api/power-status', methods=['POST'])
def post_power():
  dataDict = json.loads(request.data)
  global power
  power = dataDict['power']
  return jsonify(power)