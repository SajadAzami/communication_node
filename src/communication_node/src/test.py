#!/usr/bin/env python


"""
Testing the communication_node
"""

from messenger_api import send_message
from messenger_api import register

register('robot1')  # TODO register for test(TAHER)
send_message("this is me", "robot1")
