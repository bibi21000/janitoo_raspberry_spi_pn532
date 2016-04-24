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

import cStringIO
import base64

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

COMMAND_CONTROLLER = 0x1050

assert(COMMAND_DESC[COMMAND_CONTROLLER] == 'COMMAND_CONTROLLER')
##############################################################

def make_pn532(**kwargs):
    return PN532Component(**kwargs)

class PN532Component(JNTComponent):
    """ A NFC reader component for spi """

    def __init__(self, bus=None, addr=None, **kwargs):
        """
        """
        oid = kwargs.pop('oid', 'rpispi.pn532')
        name = kwargs.pop('name', "Screen")
        product_name = kwargs.pop('product_name', "RFID component")
        product_type = kwargs.pop('product_type', "RFID component")
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
        uuid="listen"
        self.values[uuid] = self.value_factory['action_boolean'](options=self.options, uuid=uuid,
            node_uuid=self.uuid,
            help='Activate/deactivate the listen mode',
            label='Listen',
            default=False,
            set_data_cb=self.set_listen,
            is_writeonly = True,
            cmd_class=COMMAND_CONTROLLER,
            genre=0x01,
        )
        self.pn532 = None
        self.listen_timer = None
        uuid="listen_delay"
        self.values[uuid] = self.value_factory['config_integer'](options=self.options, uuid=uuid,
            node_uuid=self.uuid,
            help='The delay between 2 listens',
            label='Delay',
            default=1,
        )
        uuid="status"
        self.values[uuid] = self.value_factory['sensor_list'](options=self.options, uuid=uuid,
            node_uuid=self.uuid,
            help='The status of the pn532',
            label='Status',
            default='sleeping',
            list_items=['sleeping', 'listening', 'write_ok', 'write_error'],
        )
        poll_value = self.values[uuid].create_poll_value(default=300)
        self.values[poll_value.uuid] = poll_value

    def start(self, mqttc):
        """Start the component
        """
        JNTComponent.start(self, mqttc)
        self._bus.spi_acquire()
        try:
            device = self.values["device"].data
            dc_pin = self._bus.get_spi_device_pin(device)
            self.pn532 = PN532.PN532(dc_pin,
                spi=self._bus.get_spi_device(device, max_speed_hz=1000000),
                gpio=self._ada_gpio)
            self.pn532.begin()
            self.pn532.SAM_configuration()
        except:
            logger.exception("[%s] - Can't start component", self.__class__.__name__)
        finally:
            self._bus.spi_release()

    def stop(self):
        """
        """
        self.stop_listen()
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

    def set_listen(self, node_uuid, index, data):
        """Reset the screen
        """
        if data == True:
            if self.listen_timer is None:
                self.listen_timer = threading.Timer(self.values['listen_delay'].data, self.on_listen)
                self.listen_timer.start()
                self.values["status"].data='listening'
        else:
            self.values["status"].data='sleeping'
            self.stop_listen()

    def stop_listen(self):
        """Check that the component is 'available'

        """
        if self.listen_timer is not None:
            self.listen_timer.cancel()
            self.listen_timer = None

    def on_listen(self):
        """Make a check using a timer.

        """
        self.stop_listen()
        state = True
        self._bus.spi_acquire()
        try:
            self.values["status"].data='listening'
            uid = self.pn532.read_passive_target()
            if uid is not None:
                logger.debug('[%s] - Read rfid %s', self.__class__.__name__, binascii.hexlify(uid))
                # Authenticate and read block 4.
                if not self.pn532.mifare_classic_authenticate_block(uid, 4, PN532.MIFARE_CMD_AUTH_B, CARD_KEY):
                    raise RuntimeError('Failed to authenticate with card')
                data = self.pn532.mifare_classic_read_block(4)
                if data is None:
                    raise RuntimeError('Failed to read data from card')
                #We should notify something hear
        except:
            logger.exception('[%s] - Exception when reading rfid', self.__class__.__name__)
        finally:
            self._bus.spi_release()
        if self.listen_timer is None and self._bus.is_started:
            self.listen_timer = threading.Timer(self.values['listen_delay'].data, self.on_listen)
            self.listen_timer.start()

    def set_write(self, node_uuid, index, data):
        """Write base64 data to card
        """
        self._bus.spi_acquire()
        try:
            sio = base64.base64_decode(data)
            uid = self.pn532.read_passive_target()
            if uid is None:
                raise RuntimeError('No card found')
            logger.debug('[%s] - Read rfid %s', self.__class__.__name__, binascii.hexlify(uid))
            # Authenticate and read block 4.
            if not self.pn532.mifare_classic_authenticate_block(uid, 4, PN532.MIFARE_CMD_AUTH_B, CARD_KEY):
                raise RuntimeError('Failed to authenticate with card')
            if not self.pn532.mifare_classic_write_block(4, sio):
                raise RuntimeError('Failed to write data to the card.')
            self.values["status"].data='write_ok'
            #We should notify something hear
        except:
            logger.exception('[%s] - Exception when writing RFID card', self.__class__.__name__)
            self.values["status"].data='write_error'
            #We should notify something hear
        finally:
            self._bus.spi_release()
