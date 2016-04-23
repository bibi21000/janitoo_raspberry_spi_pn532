# -*- coding: utf-8 -*-
"""The Raspberry ili9341 thread

See https://github.com/adafruit/Adafruit_Python_CharLCD/blob/master/examples/char_lcd.py

"""

__license__ = """
    This file is part of Janitoo.

    Janitoo is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Janitoo is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Janitoo. If not, see <http://www.gnu.org/licenses/>.

"""
__author__ = 'Sébastien GALLET aka bibi21000'
__email__ = 'bibi21000@gmail.com'
__copyright__ = "Copyright © 2013-2014-2015-2016 Sébastien GALLET aka bibi21000"

import logging
logger = logging.getLogger(__name__)
import os, sys
import threading

from janitoo.thread import JNTBusThread, BaseThread
from janitoo.options import get_option_autostart
from janitoo.utils import HADD
from janitoo.node import JNTNode
from janitoo.value import JNTValue
from janitoo.component import JNTComponent

try:
    import Adafruit_PN532 as PN532
    import Adafruit_GPIO as GPIO
    import Adafruit_GPIO.SPI as SPI
except:
    logger.exception("Can't import Adafruit_ILI9341")

##############################################################
#Check that we are in sync with the official command classes
#Must be implemented for non-regression
from janitoo.classes import COMMAND_DESC

COMMAND_MOTOR = 0x3100
COMMAND_SWITCH_MULTILEVEL = 0x0026
COMMAND_SWITCH_BINARY = 0x0025

assert(COMMAND_DESC[COMMAND_SWITCH_MULTILEVEL] == 'COMMAND_SWITCH_MULTILEVEL')
assert(COMMAND_DESC[COMMAND_SWITCH_BINARY] == 'COMMAND_SWITCH_BINARY')
assert(COMMAND_DESC[COMMAND_MOTOR] == 'COMMAND_MOTOR')
##############################################################

def make_reader(**kwargs):
    return ReaderComponent(**kwargs)

class ReaderComponent(JNTComponent):
    """ A Screen component for spi """

    def __init__(self, bus=None, addr=None, **kwargs):
        """
        """
        oid = kwargs.pop('oid', 'rpispi.pn532reader')
        name = kwargs.pop('name', "Screen")
        product_name = kwargs.pop('product_name', "NFC reader")
        product_type = kwargs.pop('product_type', "NFC reader")
        product_manufacturer = kwargs.pop('product_manufacturer', "Janitoo")
        JNTComponent.__init__(self, oid=oid, bus=bus, addr=addr, name=name,
                product_name=product_name, product_type=product_type, product_manufacturer=product_manufacturer, **kwargs)
        logger.debug("[%s] - __init__ node uuid:%s", self.__class__.__name__, self.uuid)
        uuid="device"
        self.values[uuid] = self.value_factory['config_byte'](options=self.options, uuid=uuid,
            node_uuid=self.uuid,
            help='Either the device number on the hardware bus or the SPI CS pin of the software one',
            label='device',
            default=0,
        )
        poll_value = self.values[uuid].create_poll_value(default=300)
        self.pn532 = None

    def start(self, mqttc):
        """Start the bus
        """
        JNTComponent.start(self, mqttc)
        self._bus.spi_acquire()
        try:
            device = self.values["device"].data
            dc_pin = self.get_spi_device_pin(device)
            self.pn532 = PN532.PN532(dc_pin,
                spi=self._bus.get_spi_device(device, max_speed_hz=1000000),
                gpio=self._ada_gpio)
        except:
            logger.exception("[%s] - Can't start component", self.__class__.__name__)
        finally:
            self._bus.spi_release()

    def stop(self):
        """
        """
        JNTComponent.stop(self)
        self._bus.spi_acquire()
        try:
            self.pn532 = None
        except:
            logger.exception('[%s] - Exception when stopping', self.__class__.__name__)
        finally:
            self._bus.spi_release()

    def check_heartbeat(self):
        """Check that the component is 'available'

        """
        return self.pn532 is not None