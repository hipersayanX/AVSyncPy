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

import sys
import math
import time
import random
import threading

MAX_QUEUE_SIZE = 65536

# no AV sync correction is done if below the minimum AV sync threshold.
AV_SYNC_THRESHOLD_MIN = 0.01

# AV sync correction is done if above the maximum AV sync threshold.
AV_SYNC_THRESHOLD_MAX = 0.1

# no AV correction is done if too big error.
AV_NOSYNC_THRESHOLD = 5.0

# maximum audio speed change to get correct sync
SAMPLE_CORRECTION_PERCENT_MAX = 0.1


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
    def __init__(self):
        self.setClock(float('nan'))

    def clock(self):
        return time.time() + self.ptsDrift

    def setClockAt(self, pts=0, time=0):
        self.pts = pts
        self.ptsDrift = self.pts - time

    def setClock(self, pts=0):
        self.setClockAt(pts, time.time())

    def syncTo(self, slave=None):
        clock = self.clock()
        slaveClock = slave.clock()

        if not math.isnan(slaveClock) and \
            (math.isnan(clock) or \
            abs(clock - slaveClock) > AV_NOSYNC_THRESHOLD):
            self.setClock(slaveClock)

class AVQueue:
    def __init__(self):
        self.maxSize = MAX_QUEUE_SIZE
        self.audioQueue = []
        self.videoQueue = []
        self.fill = True
        self.mutex = threading.Lock()
        self.audioQueueNotEmpty = threading.Condition(self.mutex)
        self.videoQueueNotEmpty = threading.Condition(self.mutex)
        self.queueNotFull = threading.Condition(self.mutex)
        self.queueThreshold = threading.Condition(self.mutex)

    def size(self, mimeType=''):
        size = 0

        if mimeType == 'audio/x-raw':
            size = len(self.audioQueue)
        elif mimeType == 'video/x-raw':
            size = len(self.videoQueue)
        else:
            size = len(self.audioQueue) + len(self.videoQueue)

        return size

    def setMaxSize(self, maxSize=MAX_QUEUE_SIZE):
        self.maxSize = maxSize

    def enqueue(self, packet={}):
        self.mutex.acquire()

        if self.size() >= self.maxSize:
            self.queueNotFull.wait()

        if packet['mimeType'] == 'audio/x-raw':
            self.audioQueue.append(packet)
            self.audioQueueNotEmpty.notifyAll()
        elif packet['mimeType'] == 'video/x-raw':
            self.videoQueue.append(packet)
            self.videoQueueNotEmpty.notifyAll()

        if self.fill:
            sys.stdout.write('\rfilling buffer {0:3.1f}'.format(100 * self.size() / self.maxSize))

        if self.fill and self.size() >= self.maxSize:
            self.fill = False
            self.queueThreshold.notifyAll()

        self.mutex.release()

    def dequeue(self, mimeType=''):
        self.mutex.acquire()

        packet = {}

        if mimeType == 'audio/x-raw':
            if self.size(mimeType) < 1:
                self.audioQueueNotEmpty.wait()

            packet = self.audioQueue.pop(0)
        elif mimeType == 'video/x-raw':
            if self.size(mimeType) < 1:
                self.videoQueueNotEmpty.wait()

            packet = self.videoQueue.pop(0)

        self.queueNotFull.notifyAll()

        if self.size(mimeType) < 1:
            self.fill = True
            self.queueThreshold.wait()

        self.mutex.release()

        return packet

class Sync:
    PackageProcessingRelease = 0
    PackageProcessingDiscard = 1
    PackageProcessingReSync = 2

    def __init__(self):
        self.log = True
        self.plainLog = False

        self.avqueue = AVQueue()
        self.output = Output()

        self.audioClock = Clock()
        self.videoClock = Clock()
        self.extrnClock = Clock()

        threading.Thread(target=self.processAudioFrame).start()
        threading.Thread(target=self.processVideoFrame).start()

    def printLog(self, packet, diff):
        if self.log:
            logFmt = '\r {} {:7.2f} A-V: {:7.3f} aq={:5d} vq={:5d}'

            if self.plainLog:
                logFmt += '\n'

            log = logFmt.format(packet['mimeType'][0],
                                self.extrnClock.clock(),
                                -diff,
                                self.avqueue.size('audio/x-raw'),
                                self.avqueue.size('video/x-raw'))

            sys.stdout.write(log)

    def iStream(self, packet={}):
        self.avqueue.enqueue(packet)

    def synchronizeAudio(self, diff=0.0, delay=0.0):
        # syncThreshold = 2 * delay
        #
        # (resync)
        # -syncThreshold
        # (discard)
        # [-delay * SAMPLE_CORRECTION_PERCENT_MAX]
        # (release)
        # [delay * SAMPLE_CORRECTION_PERCENT_MAX]
        # (wait)
        # syncThreshold
        # (resync)

        syncThreshold = 2 * delay
        correctionThreshold = delay * SAMPLE_CORRECTION_PERCENT_MAX

        if diff != None and abs(diff) < syncThreshold:
            if diff > -syncThreshold:
                # stream is ahead the external clock.
                if diff > correctionThreshold:
                    time.sleep(diff)

                return self.PackageProcessingRelease
            # stream is backward the external clock.
            else:
                return self.PackageProcessingDiscard

        return self.PackageProcessingReSync

    def processAudioFrame(self):
        while True:
            packet = self.avqueue.dequeue('audio/x-raw')

            diff = self.audioClock.clock() - self.extrnClock.clock()
            pts = packet['pts']
            delay = packet['duration']

            self.audioClock.setClock(pts)
            operation = self.synchronizeAudio(diff, delay)

            if operation == self.PackageProcessingDiscard:
                continue
            elif operation == self.PackageProcessingReSync:
                # update current video pts
                self.extrnClock.syncTo(self.audioClock)

            self.printLog(packet, diff)
            self.output.releaseFrame(packet)

    def synchronizeVideo(self, diff=0.0, delay=0.0):
        # (resync)
        # -AV_NOSYNC_THRESHOLD
        # (discard)
        # -syncThreshold
        # (release)
        # syncThreshold
        # (wait)
        # AV_NOSYNC_THRESHOLD
        # (resync)

        syncThreshold = max(AV_SYNC_THRESHOLD_MIN,
                            min(delay,
                                AV_SYNC_THRESHOLD_MAX))

        if diff != None and abs(diff) < AV_NOSYNC_THRESHOLD:
            if diff > -syncThreshold:
                # stream is ahead the external clock.
                if diff > syncThreshold:
                    time.sleep(diff)

                return self.PackageProcessingRelease
            # stream is backward the external clock.
            else:
                return self.PackageProcessingDiscard

        # Update clocks.

        return self.PackageProcessingReSync

    def processVideoFrame(self):
        while True:
            packet = self.avqueue.dequeue('video/x-raw')

            diff = self.videoClock.clock() - self.extrnClock.clock()
            pts = packet['pts']
            delay = packet['duration']

            self.videoClock.setClock(pts)
            operation = self.synchronizeVideo(diff, delay)

            if operation == self.PackageProcessingDiscard:
                continue
            elif operation == self.PackageProcessingReSync:
                # update current video pts
                self.extrnClock.syncTo(self.videoClock)

            self.printLog(packet, diff)
            self.output.releaseFrame(packet)


if __name__== "__main__":
    decoder = Decoder()
    sync = Sync()

    while True:
#        time.sleep(0.01)
        sync.iStream(decoder.getPacket())
