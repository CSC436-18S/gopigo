"""
This module is the main module of the ACC program.

Running this module will start up the ACC and run the webserver for the user
interface.
"""

import multiprocessing

import acc
import api

def main():
    """
    The main function of the ACC program. It starts up the ACC and runs the
    webserver for the user interface.
    """
    command_queue = multiprocessing.Queue()

    listener_process = multiprocessing.Process(target=api.run, args=(True, command_queue))

    listener_process.start()

    acc_instance = acc.ACC(command_queue, 150, 80)
    acc_instance.run()

if __name__ == "__main__":
    main()
