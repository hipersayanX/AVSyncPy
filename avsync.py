#!/usr/bin/env python
# -*- coding: UTF-8 -*-
#
# AVSyncPy, webcam capture plasmoid.
# Copyright (C) 2011-2013  Gonzalo Exequiel Pedone
#
# AVSyncPy is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# AVSyncPy is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with AVSyncPy. If not, see <http://www.gnu.org/licenses/>.
#
# Email   : hipersayan DOT x AT gmail DOT com
# Web-Site: http://github.com/hipersayanX/AVSyncPy

import time
import random

# Timming in seconds

# no AV sync correction is done if below the minimum AV sync threshold.
AV_SYNC_THRESHOLD_MIN = 0.01

# AV sync correction is done if above the maximum AV sync threshold.
AV_SYNC_THRESHOLD_MAX = 0.1


class Decoder:
    def __init__(self):
        self.aPacket = {'mimeType': 'audio/x-raw',
                        'channels': 6,
                        'rate': 48000,
                        'samples': 512}

        self.vPacket = {'mimeType': 'video/x-raw',
                        'fps': 13978 / 583}

        # Initialize AV frames.
        self.aN = 0
        self.aDuration = self.aPacket['samples'] / self.aPacket['rate']

        self.vN = 0
        self.vDuration = 1 / self.vPacket['fps']

    # Simulate frame capturing.
    def getFrame(self):
        aPts = self.aN * self.aDuration
        vPts = self.vN * self.vDuration

        # Get an audio frame.
        if aPts <= vPts:
            packet = {'pts': aPts, 'duration': self.aDuration}
            packet.update(self.aPacket)
            self.aN += 1

            return packet
        # Get an video frame.
        else:
            packet = {'pts': vPts, 'duration': self.vDuration}
            packet.update(self.vPacket)
            self.vN += 1

            return packet

class Output:
    # Simulate frame processing.
    def releaseFrame(self, frame={}):
        jitter = 0.0 #random.uniform(0.0, 0.02)
        time.sleep(frame['duration'] + jitter)

class Clock:
    def __init__(self, slave=False):
        self.lastClock = 0
        self.drift = 0
        self.slave = slave
        self.clock0 = None # Nan

    def clock(self, pts=0):
        clock = time.time()

        if self.clock0 == None:
            self.clock0 = pts if self.slave else clock

        if not self.slave:
            pts = clock

        self.lastClock = pts - self.clock0 + self.drift

        return self.lastClock

    def syncTo(self, pts=0):
        self.drift += pts - self.lastClock


if __name__== "__main__":
    decoder = Decoder()
    output = Output()
    fst = True

    while True:
        frame = decoder.getFrame()

        if fst:
            audioClock = Clock(True)
            videoClock = Clock(True)
            extrnClock = Clock()
            fst = False

        clock = extrnClock.clock()
        streamType = frame['mimeType']

        if streamType == 'audio/x-raw':
            pts = audioClock.clock(frame['pts'])
        else:
            pts = videoClock.clock(frame['pts'])

        diff = pts - clock
        show = False

        if abs(diff) < AV_SYNC_THRESHOLD_MIN:
            show = True
            output.releaseFrame(frame)
        elif abs(diff) < AV_SYNC_THRESHOLD_MAX:
            if diff < 0:
                # Discard frame
                pass
            else:
                # Add a delay
                if streamType == 'audio/x-raw':
                    pass
                else:
                    show = True
                    time.sleep(abs(diff))
                    output.releaseFrame(frame)
        else:
            show = True
            # Resync to the master clock
            if streamType == 'audio/x-raw':
                audioClock.syncTo(clock)
            else:
                videoClock.syncTo(clock)

            output.releaseFrame(frame)

        print(streamType[0], '{0:.2f}'.format(clock), '{0:.2f}'.format(pts), '{0:.2f}'.format(diff), show)
