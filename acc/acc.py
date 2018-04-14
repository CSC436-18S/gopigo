"""
This is the module of the program that contains all of the functionality
related to the ACC's control of the rover.
"""
from flask import Flask, render_template, url_for

"""
  application instance using Flask
"""
app = Flask(__name__)  

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