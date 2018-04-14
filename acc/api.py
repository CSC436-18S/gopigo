"""
This is the module of the program that contains all of the functions related to
the webserver that hosts the user interface for the ACC.
"""
from flask import request, render_template, jsonify
from flask_restful import Api, Resource
from acc import app
import json

"""
  create api instance of Flask application
"""
api = Api(app)

"""
  default values for needed variables for application
"""
speed = 0
distance = 9999
power = False

"""
  handles getting the user settings
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
  handles POST request of user settings from fetch
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
  handles GET request for the power status
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
  handles POST request for setting the power from fetch
"""
@app.route('/api/power-status', methods=['POST'])
def post_power():
  print('data: ', request.data)
  dataDict = json.loads(request.data)
  global power
  power = dataDict['power']
  print('power: ', power)
  return jsonify(power)