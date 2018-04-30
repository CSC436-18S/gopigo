## CSC436 GoPiGo Project
This program is an adaptive cruise control program (ACC) developed as a part of the CSC 436 Software Requirements class. It can control a GoPiGo rover in order to try to maintain a set desired speed and safe distance from any obstacles (walls, other rovers, etc.).

## Usage
In order to start the ACC, you must run the following command in the project directory.

```
$ python acc
```

You can provide an initial user set speed and safe distance by providing command line arguments like follows.

```
$ python acc speed=100 distance=50
```

The ACC can also be controlled though a web-based user interface that be accessed through a browser by going to the port specified when starting up the ACC.

### Dependencies
To install the Python library dependencies of the program you can run the following command.

```
$ pip install -r requirements.txt
```

### Virtual Environment
If you want to run the ACC in a virtual environment, perhaps to simulate the Python setup on the GoPiGo, you can run the following command to setup the virtual environemnt.

```
$ virtualenv -p /usr/bin/python2.7 env --no-site-packages
```

Then, whenever you want to open the virtual environment you can run the follwing command in the root directory of the project.

```
$ source env/bin/activate env
```
