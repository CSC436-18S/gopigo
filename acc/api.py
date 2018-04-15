"""
This is the module of the program that contains all of the functions related to
the webserver that hosts the user interface for the ACC.
"""
from flask import Flask, render_template, request, render_template, jsonify
from flask_restful import Api, Resource
from flask_cors import CORS
import json


"""
  - create application instance of Flask
  - create an API instance of the Flask application
  - enable CORS to so web can correctly communicate with API
"""
app = Flask(__name__)
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
  root path of application that renders the index.html
  file from the templates folder
"""
@app.route('/')
def index():
    return render_template('index.html')

"""
  function that handles starting the application on port 8080
"""
def run(isDebug):
  app.run(port=8080, debug=isDebug, threaded=True)

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