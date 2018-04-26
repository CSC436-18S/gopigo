"""
This module is the main module of the ACC program.

Running this module will start up the ACC and run the webserver for the user
interface.
"""
from __future__ import print_function

import os.path
import multiprocessing
from multiprocessing.managers import BaseManager
import time
import settings

#import acc
import api


class SettingsManager(BaseManager):
    pass


SettingsManager.register('SystemInfo', settings.SystemInfo)

LOCK_FILE = "lock.txt"


def main():
    """
    The main function of the ACC program. It starts up the ACC and runs the
    webserver for the user interface.
    """
    command_queue = multiprocessing.Queue()
    manager = SettingsManager()
    manager.start()

    """
    default values for needed variables for application,
    speed 0 so not moving, distance 9999 because this is
    technically safeDistance, power False because we don't
    want to start the rover immediately after the application
    """
    system_info = manager.SystemInfo()  # pylint: disable=no-member

    # listener_process = multiprocessing.Process(target=lambda x: print("P"), args=(True,))
    listener_process = multiprocessing.Process(
        target=api.run, args=(False, command_queue, system_info))
    #api.run(True, command_queue)

    #listener_process = api.test(command_queue)

    listener_process.start()

    # for testing purposes
    listener_process.join()

    #listen_pid = listener_process.pid

    #print(str(os.getpid()) + " <> " + str(listen_pid))

    #if os.getpid() != listen_pid:
    acc_instance = acc.ACC(system_info, command_queue, 150, 30)
    acc_instance.run()
    # acc.main(command_queue)


if __name__ == "__main__":
    # if not os.path.isfile(LOCK_FILE):
    #    open(LOCK_FILE, "a").close()
    main()
