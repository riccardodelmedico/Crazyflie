# -*- coding: utf-8 -*-
#
# ,---------,       ____  _ __
# |  ,-^-,  |      / __ )(_) /_______________ _____  ___
# | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
# | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
# Copyright (C) 2019 - 2021 Bitcraze AB
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, in version 3.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
import logging
import math
import struct

from .memory_element import MemoryElement

logger = logging.getLogger(__name__)


class Poly4D:
    class Poly:
        def __init__(self, values=[0.0] * 8):
            self.values = values

    def __init__(self, duration, x=None, y=None, z=None, yaw=None):
        self.duration = duration
        self.x = x if x else self.Poly()
        self.y = y if y else self.Poly()
        self.z = z if z else self.Poly()
        self.yaw = yaw if yaw else self.Poly()

    def pack(self):
        data = bytearray()

        data += struct.pack('<ffffffff', *self.x.values)
        data += struct.pack('<ffffffff', *self.y.values)
        data += struct.pack('<ffffffff', *self.z.values)
        data += struct.pack('<ffffffff', *self.yaw.values)
        data += struct.pack('<f', self.duration)

        return data


class _CompressedBase:
    def _encode_spatial(self, coordinate):
        '''
        Spatial coordinates (X, Y and Z) are represented as millimeters and are stored as signed 2-byte integers,
        meaning that the maximum spatial volume that this representation can cover is roughly 64m x 64m x 64m,
        assuming that the origin is at the center.
        '''
        return int(coordinate * 1000)

    def _encode_spatial_element(self, element):
        return map(self._encode_spatial, element)

    def _encode_yaw(self, angle_rad):
        '''
        Angles (for the yaw coordinate) are represented as 1/10th of degrees and are stored as signed 2-byte
        integers.
        '''
        return int(math.degrees(angle_rad) * 10)

    def _encode_yaw_element(self, element):
        '''
        Angles (for the yaw coordinate) are represented as 1/10th of degrees and are stored as signed 2-byte
        integers.
        '''
        return map(self._encode_yaw, element)


class CompressedStart(_CompressedBase):
    def __init__(self, x, y, z, yaw) -> None:
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw

    def pack(self):
        data = bytearray()

        data += struct.pack(
            '<hhhh',
            self._encode_spatial(self.x),
            self._encode_spatial(self.y),
            self._encode_spatial(self.z),
            self._encode_yaw(self.yaw))

        return data


class CompressedSegment(_CompressedBase):
    def __init__(self, duration, element_x, element_y, element_z, element_yaw) -> None:
        self._validate(element_x)
        self._validate(element_y)
        self._validate(element_z)
        self._validate(element_yaw)

        self.duration = duration
        self.x = element_x
        self.y = element_y
        self.z = element_z
        self.yaw = element_yaw

    def pack(self):
        element_types = (self._encode_type(self.x) << 0) | (self._encode_type(self.y) << 2) | (
            self._encode_type(self.z) << 4) | (self._encode_type(self.yaw) << 6)
        duration_ms = int(self.duration * 1000.0)

        data = bytearray()

        data += struct.pack('<BH', element_types, duration_ms)
        data += self._pack_element(self._encode_spatial_element(self.x))
        data += self._pack_element(self._encode_spatial_element(self.y))
        data += self._pack_element(self._encode_spatial_element(self.z))
        data += self._pack_element(self._encode_yaw_element(self.yaw))

        return data

    def _validate(self, element):
        length = len(element)
        if length != 0 and length != 1 and length != 3 and length != 7:
            raise Exception('length of element must be 0, 1, 3, or 7')

    def _encode_type(self, element):
        if len(element) == 0:
            return 0
        if len(element) == 1:
            return 1
        if len(element) == 3:
            return 2
        if len(element) == 7:
            return 3

    def _pack_element(self, encoded_element):
        data = bytearray()

        for part in encoded_element:
            data += struct.pack('<h', part)

        return data


class TrajectoryMemory(MemoryElement):
    """
    Memory interface for trajectories used by the high level commander
    """

    def __init__(self, id, type, size, mem_handler):
        """Initialize trajectory memory"""
        super(TrajectoryMemory, self).__init__(id=id, type=type, size=size,
                                               mem_handler=mem_handler)
        self._write_finished_cb = None
        self._write_failed_cb = None

        # A list of trajectory elements to write to the Crazyflie. The elements can either be
        # Poly4D instances for uncomressed trajectorys or one CompressedStart instance followed
        # by CompressedSegment instances. It is not possible to mix uncompressed and compressed
        # elements in the same trajectory.
        self.trajectory = []

    # Deprecated. replaced by self.trajectory
    @property
    def poly4Ds(self):
        return self.trajectory

    # Deprecated. replaced by self.trajectory
    @poly4Ds.setter
    def poly4Ds(self, trajectory):
        self.trajectory = trajectory

    def write_data(self, write_finished_cb, write_failed_cb=None):
        """Write trajectory data to the Crazyflie"""
        self._write_finished_cb = write_finished_cb
        self._write_failed_cb = write_failed_cb
        data = bytearray()

        for element in self.trajectory:
            data += element.pack()

        self.mem_handler.write(self, 0x00, data, flush_queue=True)

    def write_done(self, mem, addr):
        if self._write_finished_cb and mem.id == self.id:
            logger.debug('Write trajectory data done')
            self._write_finished_cb(self, addr)
            self._write_finished_cb = None
            self._write_failed_cb = None

    def write_failed(self, mem, addr):
        if mem.id == self.id:
            if self._write_failed_cb:
                logger.debug('Write of trajectory data failed')
                self._write_failed_cb(self, addr)
            self._write_finished_cb = None
            self._write_failed_cb = None

    def disconnect(self):
        self._write_finished_cb = None
