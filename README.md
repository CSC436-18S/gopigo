## CSC436 GoPiGo Project

## Usage
In order to start the ACC, you must run the following command in the project directory.

```
$ python acc
```

### Dependencies
To install the Python library dependencies of the program you can run the folliwng command.

```
$ pip install -r requirements.txt
```

### Virtual Environment
If you want to run the ACC in a virutal environment, perhaps to simulate the Python setup on the GoPiGo, you can run the following command to setup the virtual environemnt.

```
$ virtualenv -p /usr/bin/python2.7 env --no-site-packages
```

Then, whenever you want to open the virtual environment you can run the follwing command in the root directory of the project.

```
$ source env/bin/activate env
```
