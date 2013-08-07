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
        # Initialize AV frames.
        self.aPts = 0
        self.aDuration = 0.01

        self.vPts = 0
        self.vDuration = 0.041

    # Simulate frame capturing.
    def getFrame(self):
        # Get an audio frame.
        if self.aPts <= self.vPts:
            pts = self.aPts
            self.aPts += self.aDuration

            return ['a', pts, self.aDuration]
        # Get an video frame.
        elif self.aPts > self.vPts:
            pts = self.vPts
            self.vPts += self.vDuration

            return ['v', pts, self.vDuration]

class Output:
    # Simulate frame processing.
    def releaseFrame(self, frame=['a', 0, 0]):
        time.sleep(frame[2])

class Clock:
    def __init__(self, slave=False):
        self.lastClock = 0
        self.drift = 0
        self.slave = slave
        self.clock0 = None # Nan

    def clock(self, pts=0):
        clock = time.time()

        if not self.clock0:
            self.clock0 = pts if self.slave else clock

        if not self.slave:
            pts = clock

        self.lastClock = pts - self.clock0 + self.drift

        return self.lastClock

    def syncTo(self, pts=0):
        self.drift += pts - self.lastClock


if __name__== "__main__":
    decoder = Decoder()
    audioClock = Clock(True)
    videoClock = Clock(True)
    extrnClock = Clock()

    while True:
        frame = decoder.getFrame()

        clock = extrnClock.clock()
        streamType = frame[0]

        if streamType == 'a':
            pts = audioClock.clock(frame[1])
        else:
            pts = videoClock.clock(frame[1])

        diff = clock - pts

        print(streamType, '{0:.2f}'.format(clock), '{0:.2f}'.format(pts), '{0:.2f}'.format(diff))

        if abs(diff) < AV_SYNC_THRESHOLD_MIN:
            Output.releaseFrame(frame)
        elif abs(diff) < AV_SYNC_THRESHOLD_MAX:
            if diff < 0:
                # Add a delay
                time.sleep(abs(diff))
                Output.releaseFrame(frame)
            else:
                # Discard frame
                pass
        else:
            # Resync to the master clock
            if streamType == 'a':
                audioClock.syncTo(clock)
            else:
                videoClock.syncTo(clock)

#            Output.releaseFrame()
