"""
This is the module of the program that contains all of the functions related to
the webserver that hosts the user interface for the ACC.
"""
import os
from flask import Flask, render_template, request, render_template, jsonify
from flask_restful import Api, Resource
from flask_cors import CORS
import json
from collections import OrderedDict

from settings import SystemInfo

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
PORT = 3000
HOSTNAME = "http://localhost"

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
  app.run(port=PORT, debug=isDebug, threaded=True)

"""
  handles getting the user settings
  by returing the user settings jsonified
"""
@app.route('/api/system-info', methods=['GET'])
def get_settings():
  data = jsonify({
    'state': {
      'currentSpeed': system_info.getCurrentSpeed(),
      'obstacleDistance': system_info.getObstacleDistance()
    },
    'settings': {
      'safeDistance': system_info.getSafeDistance(),
      'userSetSpeed': system_info.getUserSetSpeed()
    }
  })
  data.status_code = 200
  return data

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
  print('speed: ', speed)
  print('distance: ', distance)
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
  print('powerRes: ', res)
  return res

"""
  handles POST request for setting the power from fetch,
  since the fetch needs the value of the power status
  after setting it, we return the power status,
  and not the render template
"""
@app.route('/api/power-status', methods=['POST'])
def post_power():
  print('data: ', request.data)
  dataDict = json.loads(request.data)
  global power
  power = dataDict['power']
  print('power: ', power)
  return jsonify(power)