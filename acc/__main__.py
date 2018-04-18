"""
This module is the main module of the ACC program.

Running this module will start up the ACC and run the webserver for the user
interface.
"""
from __future__ import print_function

import multiprocessing
from multiprocessing.managers import BaseManager
import settings

import acc
import api


class SettingsManager(BaseManager):
    """
    A class used to allow the SystemInfo object to be multiprocess safe.
    """
    pass


SettingsManager.register('SystemInfo', settings.SystemInfo)

def main():
    """
    The main function of the ACC program. It starts up the ACC and runs the
    webserver for the user interface.
    """
    command_queue = multiprocessing.Queue()
    manager = SettingsManager()
    manager.start()

    system_info = manager.SystemInfo()  # pylint: disable=no-member

    listener_process = multiprocessing.Process(
        target=api.run, args=(False, command_queue, system_info))

    listener_process.start()

    acc_instance = acc.ACC(system_info, command_queue, 150, 30)
    acc_instance.run()


if __name__ == "__main__":
    main()
