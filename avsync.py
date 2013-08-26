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
import threading

# Timming in seconds

# no AV sync correction is done if below the minimum AV sync threshold.
AV_SYNC_THRESHOLD_MIN = 0.01

# AV sync correction is done if above the maximum AV sync threshold.
AV_SYNC_THRESHOLD_MAX = 0.1

# maximum audio speed change to get correct sync
SAMPLE_CORRECTION_PERCENT_MAX = 0.1

# we use about AUDIO_DIFF_AVG_NB A-V differences to make the average
AUDIO_DIFF_AVG_NB = 20


class Decoder:
    def __init__(self):
        self.aPacket = {'mimeType': 'audio/x-raw',
                        'bps': 4,
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
    def getPacket(self):
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

class Lock:
    def __init__(self, nLocks=1):
        self.lock = threading.Lock()
        self.counterLock = threading.Lock()
        self.nLocked = 0
        self.nLocks = nLocks

    def acquire(self):
        self.counterLock.acquire()
        self.nLocked += 1

        if self.nLocked == self.nLocks:
            self.lock.acquire()

        self.counterLock.release()

    def release(self):
        self.counterLock.acquire()

        if self.nLocked == self.nLocks:
            self.lock.release()

        self.nLocked -= 1
        self.counterLock.release()

    def wait(self):
        self.lock.acquire()
        self.lock.release()

class Sync:
    def __init__(self):
        self.output = Output()
        self.init()

    def init(self):
        self.audioClock = Clock(True)
        self.videoClock = Clock(True)
        self.extrnClock = Clock()

        self.audioLock = threading.Lock()
        self.videoLock = threading.Lock()
        self.globalLock = Lock(2)

    def processAudioFrame(self, packet={}):
        self.audioLock.acquire()
        self.globalLock.acquire()

        clock = self.extrnClock.clock()
        pts = self.audioClock.clock(packet['pts'])
        diff = clock - pts
        show = False
        duration = packet['samples'] / packet['rate']
        minThreshhold = duration * (1 - SAMPLE_CORRECTION_PERCENT_MAX)
        maxThreshhold = duration * (1 + SAMPLE_CORRECTION_PERCENT_MAX)

        if abs(diff) < minThreshhold:
            show = True
            self.output.releaseFrame(packet)
        elif abs(diff) < maxThreshhold:
            if diff < 0:
                # Add a delay
                show = True
                newPacket = self.growAudio(packet, abs(diff))

                if newPacket != {}:
                    self.output.releaseFrame(newPacket)
            else:
                # Discard frame
                show = True
                newPacket = self.shrinkAudio(packet, diff)

                if newPacket != {}:
                    self.output.releaseFrame(newPacket)
        else:
            # Resync to the master clock
            show = True
            self.audioClock.syncTo(clock)
            self.output.releaseFrame(packet)

        print(packet['mimeType'][0],
              '{0:.2f}'.format(clock),
              '{0:.2f}'.format(pts),
              '{0:.2f}'.format(diff),
              show)

        self.globalLock.release()
        self.audioLock.release()

    def processVideoFrame(self, packet={}):
        self.videoLock.acquire()
        self.globalLock.acquire()

        clock = self.extrnClock.clock()
        pts = self.videoClock.clock(packet['pts'])
        diff = clock - pts
        show = False

        if abs(diff) < AV_SYNC_THRESHOLD_MIN:
            show = True
            self.output.releaseFrame(packet)
        elif abs(diff) < AV_SYNC_THRESHOLD_MAX:
            if diff < 0:
                # Add a delay
                show = True
                time.sleep(abs(diff))
                self.output.releaseFrame(packet)
            else:
                # Discard frame
                pass
        else:
            # Resync to the master clock
            show = True
            self.videoClock.syncTo(clock)
            self.output.releaseFrame(packet)

        print(packet['mimeType'][0],
              '{0:.2f}'.format(clock),
              '{0:.2f}'.format(pts),
              '{0:.2f}'.format(diff),
              show)

        self.globalLock.release()
        self.videoLock.release()

    def growAudio(self, packet={}, diff=0):
        samplesDiff = round(diff * packet['rate'])
        bytesDiff = samplesDiff * packet['bps'] * packet['channels']

        print('grow', samplesDiff, bytesDiff)

        if samplesDiff > 0:
            return packet

        return packet

    def shrinkAudio(self, packet={}, diff=0):
        samplesDiff = round(diff * packet['rate'])
        samplesDiff = max(0, min(samplesDiff, packet['samples']))

        # You must remove bytesDiff
        bytesDiff = samplesDiff * packet['bps'] * packet['channels']

        if samplesDiff < packet['samples']:
            newPacket = packet
            samples = packet['samples'] - samplesDiff
            duration = samples / packet['rate']
            pts = packet['pts'] + packet['duration'] - duration
            newPacket['samples'] = samples
            newPacket['duration'] = duration
            newPacket['pts'] = pts

            return newPacket

        return {}


if __name__== "__main__":
    decoder = Decoder()
    sync = Sync()
    fst = True

    while True:
        packet = decoder.getPacket()
        sync.globalLock.wait()

        if fst:
            sync.init()
            fst = False

        streamType = packet['mimeType']

        if streamType == 'audio/x-raw':
            threading.Thread(target=sync.processAudioFrame,
                             args=(packet, )).start()
        else:
            threading.Thread(target=sync.processVideoFrame,
                             args=(packet, )).start()
