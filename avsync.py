#!/usr/bin/env python
# -*- coding: UTF-8 -*-
#
# AVSyncPy, Lip synchronization algorith implemented in Python.
# Copyright (C) 2013  Gonzalo Exequiel Pedone
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

LOG = True
PLAIN_LOG = True

MAX_QUEUE_SIZE = 65536
OUTPUT_AUDIO_BUFFER_SIZE = 512

# no AV sync correction is done if below the minimum AV sync threshold.
AV_SYNC_THRESHOLD_MIN = 0.01

# AV sync correction is done if above the maximum AV sync threshold.
AV_SYNC_THRESHOLD_MAX = 0.1

# no AV correction is done if too big error.
AV_NOSYNC_THRESHOLD = 5.0

# maximum audio speed change to get correct sync.
SAMPLE_CORRECTION_PERCENT_MAX = 0.1

# we use about AUDIO_DIFF_AVG_NB A-V differences to make the average.
AUDIO_DIFF_AVG_NB = 20


class Decoder:
    def __init__(self):
        self.aPacket = {'mimeType': 'audio/x-raw',
                        'rate': 48000,
                        'samples': 1536}

        self.vPacket = {'mimeType': 'video/x-raw',
                        'fps': 30000 / 1001}

        # Initialize AV frames.
        self.aN = 0
        self.aDuration = self.aPacket['samples'] / self.aPacket['rate']

        self.vN = 0
        self.vDuration = 1 / self.vPacket['fps']

    # Simulate frame capturing.
    def readPacket(self):
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
        self.log = LOG
        self.plainLog = PLAIN_LOG

        self.maxSize = MAX_QUEUE_SIZE
        self.audioQueue = []
        self.videoQueue = []

        self.fill = False

        self.queueMutex = threading.Lock()
        self.iMutex = threading.Lock()
        self.aoMutex = threading.Lock()
        self.voMutex = threading.Lock()

        self.bufferNotFull = threading.Condition(self.iMutex)
        self.audioBufferNotEmpty = threading.Condition(self.aoMutex)
        self.videoBufferNotEmpty = threading.Condition(self.voMutex)

    def size(self, mimeType=''):
        self.queueMutex.acquire()

        if mimeType == 'audio/x-raw':
            size = len(self.audioQueue)
        elif mimeType == 'video/x-raw':
            size = len(self.videoQueue)
        else:
            size = len(self.audioQueue) + len(self.videoQueue)

        self.queueMutex.release()

        return size

    def setMaxSize(self, maxSize=MAX_QUEUE_SIZE):
        self.maxSize = maxSize

    def dequeueAudio(self):
        self.aoMutex.acquire()

        if self.size('audio/x-raw') < 1:
            self.audioBufferNotEmpty.wait()

        self.queueMutex.acquire()
        packet = self.audioQueue.pop(0)
        self.queueMutex.release()

        self.bufferNotFull.acquire()
        self.bufferNotFull.notifyAll()
        self.bufferNotFull.release()

        self.aoMutex.release()

        return packet

    def dequeueVideo(self):
        self.voMutex.acquire()

        if self.size('video/x-raw') < 1:
            self.videoBufferNotEmpty.wait()

        self.queueMutex.acquire()
        packet = self.videoQueue.pop(0)
        self.queueMutex.release()

        self.iMutex.acquire()
        self.bufferNotFull.notifyAll()
        self.iMutex.release()

        self.voMutex.release()

        return packet

    def enqueue(self, packet={}):
        bufferSize = self.size()

        if bufferSize < 1:
            self.fill = True

        if bufferSize >= self.maxSize:
            if self.fill:
                if self.size('audio/x-raw') > 0:
                    self.audioBufferNotEmpty.acquire()
                    self.audioBufferNotEmpty.notifyAll()
                    self.audioBufferNotEmpty.release()

                if self.size('video/x-raw') > 0:
                    self.videoBufferNotEmpty.acquire()
                    self.videoBufferNotEmpty.notifyAll()
                    self.videoBufferNotEmpty.release()

                self.fill = False

            self.iMutex.acquire()
            self.bufferNotFull.wait()
            self.iMutex.release()

        self.queueMutex.acquire()

        if packet['mimeType'] == 'audio/x-raw':
            self.audioQueue.append(packet)
        elif packet['mimeType'] == 'video/x-raw':
            self.videoQueue.append(packet)

        self.queueMutex.release()

        if self.fill and self.log:
            if self.plainLog:
                print('filling buffer {0:3.1f}'.format(100 * self.size() / self.maxSize))
            else:
                sys.stdout.write('\rfilling buffer {0:3.1f}'.format(100 * self.size() / self.maxSize))

        if not self.fill:
            if self.size('audio/x-raw') > 0:
                self.audioBufferNotEmpty.acquire()
                self.audioBufferNotEmpty.notifyAll()
                self.audioBufferNotEmpty.release()

            if self.size('video/x-raw') > 0:
                self.videoBufferNotEmpty.acquire()
                self.videoBufferNotEmpty.notifyAll()
                self.videoBufferNotEmpty.release()

    def dequeue(self, mimeType=''):
        if mimeType == 'audio/x-raw':
            return self.dequeueAudio()
        elif mimeType == 'video/x-raw':
            return self.dequeueVideo()

        return {}

class Sync:
    PackageProcessingRelease = 0
    PackageProcessingDiscard = 1
    PackageProcessingReSync = 2

    def __init__(self):
        self.log = LOG
        self.plainLog = PLAIN_LOG

        self.audioDiffAvgCoef = math.exp(math.log(0.01) / AUDIO_DIFF_AVG_NB)
        self.audioDiffCum = 0
        self.audioDiffAvgCount = 0
        self.setOutputAudioBufferSize(OUTPUT_AUDIO_BUFFER_SIZE)

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

    def setOutputAudioBufferSize(self, outputAudioBufferSize=OUTPUT_AUDIO_BUFFER_SIZE):
        self.outputAudioBufferSize = outputAudioBufferSize

    def compensateAudio(self, packet={}, wantedSamples=0):
        packet['samples'] = wantedSamples
        packet['duration'] = wantedSamples / packet['rate']

        return packet

    def synchronizeAudio(self, diff=0.0, packet={}):
        wantedSamples = packet['samples']

        if not math.isnan(diff) and abs(diff) < AV_NOSYNC_THRESHOLD:
            self.audioDiffCum = diff + self.audioDiffAvgCoef * self.audioDiffCum

            if self.audioDiffAvgCount < AUDIO_DIFF_AVG_NB:
                # not enough measures to have a correct estimate
                self.audioDiffAvgCount += 1
            else:
                # estimate the A-V difference
                avgDiff = self.audioDiffCum * (1.0 - self.audioDiffAvgCoef)
                audioDiffThreshold = 2.0 * self.outputAudioBufferSize / packet['rate']

                if abs(avgDiff) >= audioDiffThreshold:
                    samples = packet['samples']
                    wantedSamples = samples + diff * packet['rate']
                    minSamples = samples * (1.0 - SAMPLE_CORRECTION_PERCENT_MAX)
                    maxSamples = samples * (1.0 + SAMPLE_CORRECTION_PERCENT_MAX)
                    wantedSamples = int(max(minSamples, min(wantedSamples, maxSamples)))
        else:
            # too big difference: may be initial PTS errors, so
            # reset A-V filter
            self.audioDiffAvgCount = 0
            self.audioDiffCum = 0

        return wantedSamples

    def processAudioFrame(self):
        while True:
            packet = self.avqueue.dequeue('audio/x-raw')

            pts = packet['pts']

            diff = self.audioClock.clock() - self.extrnClock.clock()
            self.audioClock.setClock(pts)

            wantedSamples = self.synchronizeAudio(diff, packet)
            packet = self.compensateAudio(packet, wantedSamples)

            self.printLog(packet, diff)
            self.output.releaseFrame(packet)
            self.extrnClock.syncTo(self.audioClock)

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
                            min(delay, AV_SYNC_THRESHOLD_MAX))

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
#        time.sleep(0.02)
        sync.iStream(decoder.readPacket())
