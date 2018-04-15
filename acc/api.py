"""
This is the module of the program that contains all of the functions related to
the webserver that hosts the user interface for the ACC.
"""
from flask import request, render_template, jsonify
from flask_restful import Api, Resource
import json
from flask_cors import CORS

"""
  import Flask appication instance so we can
  convert to API
"""
from acc import app

"""
  create api instance of Flask application,
  and enable CORS so web can correctly communicate
  with our API
"""
api = Api(app)
CORS(app)

"""
  default values for needed variables for application,
  speed 0 so not moving, distance 9999 because this is
  technically safeDistance, power False because we don't
  want to start the rover immediately after the application
"""
speed = 0
distance = 9999
power = False

"""
  handles getting the user settings
  by returing the user settings jsonified
"""
@app.route('/api/user-settings', methods=['GET'])
def get_settings():
  res = jsonify({
    "speed": speed,
    "distance": distance
  })
  res.status_code = 200
  return res

"""
  handles POST request of user settings from fetch,
  and returns the render_template index.html
  because we do not need immediate access to the data
  after the POST request
"""
@app.route('/api/user-settings', methods=['POST'])
def post_settings():
  dataDict = json.loads(request.data)
  global speed
  global distance
  speed = dataDict['speed']
  distance = dataDict['distance']
  print('speed: ', speed)
  print('distance: ', distance)
  return render_template('index.html')

"""
  handles GET request for the power status,
  by returing the power status jsonified
"""
@app.route('/api/power-status', methods=['GET'])
def get_power():
  res = jsonify({
    "power": power
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