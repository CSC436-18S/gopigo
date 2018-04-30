"""
This module is the main module of the ACC program.

Running this module will start up the ACC and run the webserver for the user
interface.
"""
import multiprocessing
from multiprocessing.managers import BaseManager
import sys

import acc
import api
import settings


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
    user_set_speed, safe_distance = get_initial_user_settings(sys.argv)

    command_queue = multiprocessing.Queue()
    manager = SettingsManager()
    manager.start()

    system_info = manager.SystemInfo()  # pylint: disable=no-member

    listener_process = multiprocessing.Process(
        target=api.run, args=(False, command_queue, system_info))

    listener_process.start()

    acc_instance = acc.ACC(system_info, command_queue, user_set_speed, safe_distance)
    acc_instance.run()


def get_initial_user_settings(argv):
    """
    Parses the given command line arguments to extract the user's desired
    user_set_speed and safe_distance.

    If a value is not given then None is returned in place of that value.

    >>> get_initial_user_settings(["speed=20", "distance=150"])
    (20, 150)

    >>> get_initial_user_settings(["foo", "distance=150"])
    (None, 150)

    :param list[str] argv: The command line arguments to parse.
    :return: A tuple containing the desired user_set_speed and safe_distance.
    :rtype: tuple[int, int]
    """
    user_set_speed = None
    safe_distance = None
    for a in argv[1:]:
        splitted = a.split("=")
        if splitted[0] == "speed":
            user_set_speed = int(splitted[1])
        elif splitted[0] == "distance":
            safe_distance = int(splitted[1])

    return user_set_speed, safe_distance

if __name__ == "__main__":
    main()
