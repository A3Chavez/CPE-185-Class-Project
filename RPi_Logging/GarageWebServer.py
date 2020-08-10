import time
from datetime import datetime
from flask import Flask, render_template, request

import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)  
GPIO.setwarnings(False)
GPIO.setup(17, GPIO.IN, GPIO.PUD_DOWN) # set up pin 17 as an input with a pull-down resistor
GPIO.setup(27, GPIO.IN, GPIO.PUD_DOWN) # set up pin 27 as an input with a pull-down resistor
GPIO.setup(22, GPIO.IN, GPIO.PUD_DOWN) # set up pin 22 as an input with a pull-down resistor

app = Flask(__name__)

@app.route('/', methods=['GET', 'POST'])
def index():

    if GPIO.input(17) == GPIO.LOW and GPIO.input(27) == GPIO.LOW and GPIO.input(22) == GPIO.HIGH:
        print ("Garage is Closed")
        return app.send_static_file('Closed.html')
    elif GPIO.input(17) == GPIO.LOW and GPIO.input(27) == GPIO.HIGH and GPIO.input(22) == GPIO.LOW:
        print("Garage is Opening/Closing")
        return app.send_static_file('Closing-Opening.html')
    elif GPIO.input(17) == GPIO.HIGH and GPIO.input(27) == GPIO.LOW and GPIO.input(22) == GPIO.LOW:
        print ("Garage is Open")
        return app.send_static_file('Open.html')

@app.route('/stylesheet.css')
def stylesheet():
        return app.send_static_file('stylesheet.css')

@app.route('/Log')
def logfile():
        return app.send_static_file('log.txt')

@app.route('/images/<picture>')
def images(picture):
        return app.send_static_file('images/' + picture)

if __name__ == '__main__':
        app.run(debug=True, host='0.0.0.0', port=5000)
