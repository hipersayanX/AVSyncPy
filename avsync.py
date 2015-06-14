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

SAMPLE_RATE = 48000

MAX_QUEUE_SIZE = 15 * 1024 * 1024

# Output audio buffer in samples.
OUTPUT_AUDIO_BUFFER_SIZE = 512

# no AV sync correction is done if below the minimum AV sync threshold.
AV_SYNC_THRESHOLD_MIN = 0.01

# AV sync correction is done if above the maximum AV sync threshold.
AV_SYNC_THRESHOLD_MAX = 0.1

# If a frame duration is longer than this, it will not be duplicated to compensate AV sync
AV_SYNC_FRAMEDUP_THRESHOLD = 0.1

# no AV correction is done if too big error.
AV_NOSYNC_THRESHOLD = 5.0

# maximum audio speed change to get correct sync.
SAMPLE_CORRECTION_PERCENT_MAX = 0.1

# we use about AUDIO_DIFF_AVG_NB A-V differences to make the average.
AUDIO_DIFF_AVG_NB = 20

# Max number of samples.
SAMPLE_ARRAY_SIZE = 8 * 65536

# Max number of video frames.
MAX_VIDEO_QUEUE_SIZE = 3

class Stream:
    def __init__(self, aPacket, vPacket):
        self.aPacket = aPacket
        self.vPacket = vPacket

        # Initialize AV frames.
        self.aN = 0

        if self.aPacket == {}:
            self.aDuration = 0
            self.aRawSize = 0
        else:
            self.aDuration = aPacket['samples'] / aPacket['rate']
            self.aRawSize = aPacket['bps'] * aPacket['channels'] * aPacket['samples']

        self.vN = 0

        if self.vPacket == {}:
            self.vDuration = 0
            self.vRawSize = 0
        else:
            self.vDuration = 1 / vPacket['fps']
            self.vRawSize = vPacket['width'] * vPacket['height'] * vPacket['bpp']

        # Packet queue.
        self.queueSize = 0
        self.audioQueueSize = 0
        self.videoQueueSize = 0

        self.audioPacketQueue = []
        self.videoPacketQueue = []

        self.queueMutex = threading.Lock()
        self.audioMutex = threading.Lock()
        self.videoMutex = threading.Lock()
        self.queueNotFull = threading.Condition(self.queueMutex)
        self.aQueueNotEmpty = threading.Condition(self.audioMutex)
        self.vQueueNotEmpty = threading.Condition(self.videoMutex)

    # Simulate packet capturing.
    def readPackets(self):
        while True:
            aPts = self.aN * self.aDuration
            vPts = self.vN * self.vDuration

            # Simulate AV encoding.
            encRatio = random.uniform(0.25, 0.75)

            self.queueMutex.acquire()

            if self.queueSize >= MAX_QUEUE_SIZE:
                self.queueNotFull.wait()

            self.queueMutex.release()

            # Get an audio frame.
            if (aPts <= vPts and self.aPacket != {}) or self.vPacket == {}:
                    packet = {'pts': aPts,
                              'duration': self.aDuration,
                              'rawSize': self.aRawSize,
                              'encRatio': encRatio,
                              'compressedSize': encRatio * self.aRawSize}
                    packet.update(self.aPacket)
                    self.aN += 1

                    self.audioMutex.acquire()
                    self.audioPacketQueue.append(packet)
                    self.audioQueueSize += packet['compressedSize']
                    self.queueSize += packet["compressedSize"]
                    self.aQueueNotEmpty.notifyAll()
                    self.audioMutex.release()
            # Get an video frame.
            else:
                if self.vPacket != {}:
                    packet = {'pts': vPts,
                              'duration': self.vDuration,
                              'rawSize': self.vRawSize,
                              'encRatio': encRatio,
                              'compressedSize': encRatio * self.vRawSize}
                    packet.update(self.vPacket)
                    self.vN += 1

                    self.videoMutex.acquire()
                    self.videoPacketQueue.append(packet)
                    self.videoQueueSize += packet['compressedSize']
                    self.queueSize += packet["compressedSize"]
                    self.vQueueNotEmpty.notifyAll()
                    self.videoMutex.release()

    def readAudioPacket(self):
        self.audioMutex.acquire()

        if len(self.audioPacketQueue) < 1:
            if not self.aQueueNotEmpty.wait(1):
                self.audioMutex.release()

                return {}

        packet = self.audioPacketQueue.pop(0)
        self.audioMutex.release()

        self.audioQueueSize -= packet['compressedSize']
        self.queueSize -= packet["compressedSize"]

        self.queueMutex.acquire()

        if self.queueSize < MAX_QUEUE_SIZE:
            self.queueNotFull.notifyAll()

        self.queueMutex.release()

        return packet

    def readVideoPacket(self):
        self.videoMutex.acquire()

        if len(self.videoPacketQueue) < 1:
            if not self.vQueueNotEmpty.wait(1):
                self.videoMutex.release()

                return {}

        packet = self.videoPacketQueue.pop(0)
        self.videoMutex.release()

        self.videoQueueSize -= packet['compressedSize']
        self.queueSize -= packet["compressedSize"]

        self.queueMutex.acquire()

        if self.queueSize < MAX_QUEUE_SIZE:
            self.queueNotFull.notifyAll()

        self.queueMutex.release()

        return packet

    def start(self):
        threading.Thread(target=self.readPackets, name='PacketRead').start()

class Decoder:
    def __init__(self, audioReadFunc, videoReadFunc):
        self.audioReadFunc = audioReadFunc
        self.videoReadFunc = videoReadFunc

        # Frame queue.
        self.audioSamples = 0
        self.videoFrames = 0

        self.audioFrameQueue = []
        self.videoFrameQueue = []

        self.audioMutex = threading.Lock()
        self.videoMutex = threading.Lock()
        self.audioQueueNotFull = threading.Condition(self.audioMutex)
        self.videoQueueNotFull = threading.Condition(self.videoMutex)
        self.aQueueNotEmpty = threading.Condition(self.audioMutex)
        self.vQueueNotEmpty = threading.Condition(self.videoMutex)

    def decodeAudio(self):
        while True:
            packet = self.audioReadFunc()

            if packet == {}:
                continue

            # Simulate frame decoding
            time.sleep(random.uniform(0.0, 0.5) * packet['duration'] * packet['encRatio'])

            self.audioMutex.acquire()

            if self.audioSamples >= SAMPLE_ARRAY_SIZE:
                self.audioQueueNotFull.wait()

            self.audioFrameQueue.append(packet)
            self.audioSamples += packet['samples']
            self.aQueueNotEmpty.notifyAll()
            self.audioMutex.release()

    def decodeVideo(self):
        while True:
            packet = self.videoReadFunc()

            if packet == {}:
                continue

            # Simulate frame decoding
            time.sleep(random.uniform(0.0, 0.5) * packet['duration'] * packet['encRatio'])

            self.videoMutex.acquire()

            if self.videoFrames >= MAX_VIDEO_QUEUE_SIZE:
                self.videoQueueNotFull.wait()

            self.videoFrameQueue.append(packet)
            self.videoFrames += 1
            self.vQueueNotEmpty.notifyAll()
            self.videoMutex.release()

    def readAudioFrame(self):
        self.audioMutex.acquire()

        if len(self.audioFrameQueue) < 1:
            if not self.aQueueNotEmpty.wait(1):
                self.audioMutex.release()

                return {}

        frame = self.audioFrameQueue.pop(0)
        self.audioSamples -= frame["samples"]

        if self.audioSamples < SAMPLE_ARRAY_SIZE:
            self.audioQueueNotFull.notifyAll()

        self.audioMutex.release()

        return frame

    def readVideoFrame(self):
        self.videoMutex.acquire()

        if len(self.videoFrameQueue) < 1:
            if not self.vQueueNotEmpty.wait(1):
                self.videoMutex.release()

                return {}

        frame = self.videoFrameQueue.pop(0)
        self.videoFrames -= 1

        if self.videoFrames < MAX_VIDEO_QUEUE_SIZE:
            self.videoQueueNotFull.notifyAll()

        self.videoMutex.release()

        return frame

    def start(self):
        threading.Thread(target=self.decodeAudio, name='AudioDecoding').start()
        threading.Thread(target=self.decodeVideo, name='VideoDecoding').start()

class Output:
    def __init__(self, sampleRate, syncObject):
        self.sampleRate = sampleRate
        self.audioCallbackFunc = syncObject.audioCallback
        syncObject.drawFrame = self.drawFrame

    # Simulate frame drawing in the screen.
    def drawFrame(self, frame):
        time.sleep(random.uniform(0.0, 0.25) * frame['duration'])

    # Simulate audio consuming.
    def audioLoop(self):
        while True:
            samplesRead = self.audioCallbackFunc(OUTPUT_AUDIO_BUFFER_SIZE)
            time.sleep(0.1 * OUTPUT_AUDIO_BUFFER_SIZE / self.sampleRate)

    def start(self):
        threading.Thread(target=self.audioLoop, name='AudioThread').start()

class Sync:
    def __init__(self, readAudioFunc, readVideoFunc):
        self.readAudioFunc = readAudioFunc
        self.readVideoFunc = readVideoFunc

        self.globalClock = 0

        # Audio Parameters
        self.audioTimeDrift = 0
        self.audioDiffAvgCoef = math.exp(math.log(0.01) / AUDIO_DIFF_AVG_NB)
        self.audioDiffCum = 0
        self.audioDiffAvgCount = 0
        self.sampleBufferSize = 0
        self.audioClock = 0

        # Video Parameters
        self.videoTimeDrift = 0
        self.lastPts = 0
        self.curFrame = {}
        self.framesDropped = 0

    def setClock(self, clock):
        self.globalClock = clock
        self.videoTimeDrift = time.time() - clock

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

    def audioCallback(self, nSamples):
        outSamples = 0

        if self.sampleBufferSize < nSamples:
            frame = self.readAudioFunc()

            if frame == {}:
                outSamples = min(self.sampleBufferSize, nSamples)
                self.audioClock += outSamples / SAMPLE_RATE
                self.sampleBufferSize = 0
            else:
                self.audioClock = frame['pts'] - self.sampleBufferSize / SAMPLE_RATE
                self.sampleBufferSize += frame['samples']
                outSamples = min(self.sampleBufferSize, nSamples)
                self.sampleBufferSize -= outSamples
        else:
            self.sampleBufferSize -= nSamples
            self.audioClock += nSamples / SAMPLE_RATE
            outSamples = nSamples

        self.globalClock = time.time() - self.audioTimeDrift
        diff = self.audioClock - self.globalClock
        #self.audioClock.setClock(pts)

        #wantedSamples = self.synchronizeAudio(diff, packet)
        #packet = self.compensateAudio(packet, wantedSamples)

        self.logger.log('a', diff)
        #self.output.releaseFrame(packet)
        #self.extrnClock.syncTo(self.audioClock)

        return outSamples

    def processVideoFrame(self):
        while True:
            if self.curFrame == {}:
                self.curFrame = self.readVideoFunc()

            if self.curFrame == {}:
                continue

            pts = self.curFrame['pts']
            self.globalClock = time.time() - self.videoTimeDrift
            diff = pts - self.globalClock
            delay = pts - self.lastPts

            # Skip or repeat frame. We take into account the
            # delay to compute the threshold. I still don't know
            # if it is the best guess
            syncThreshold = max(AV_SYNC_THRESHOLD_MIN,
                                min(delay, AV_SYNC_THRESHOLD_MAX))

            if not math.isnan(diff) \
               and abs(diff) < AV_NOSYNC_THRESHOLD \
               and delay < AV_SYNC_FRAMEDUP_THRESHOLD:
                if diff <= -syncThreshold:
                    # video is backward the external clock.
                    self.curFrame = {}
                    self.lastPts = pts
                    self.framesDropped += 1

                    continue
                elif diff > syncThreshold:
                    # video is ahead the external clock.
                    time.sleep(diff - syncThreshold)

                    continue
            else:
                # Resync video.
                self.globalClock = pts
                self.videoTimeDrift = time.time() - pts

            self.logger.log('v', diff)
            self.drawFrame(self.curFrame)
            self.curFrame = {}
            self.lastPts = pts

    def start(self):
        threading.Thread(target=self.processVideoFrame, name='SyncVideo').start()

class Logger:
    def __init__(self, stream, sync):
        self.stream = stream
        self.sync = sync
        self.sync.logger = self

    def log(self, streamType, diff):
        if LOG:
            logFmt = '\r {} {:7.2f} A-V: {:7.3f} fd={:5d} aq={:5d} vq={:5d}'

            if PLAIN_LOG:
                logFmt += '\n'

            log = logFmt.format(streamType,
                                self.sync.globalClock,
                                -diff,
                                self.sync.framesDropped,
                                int(self.stream.audioQueueSize),
                                int(self.stream.videoQueueSize))

            sys.stdout.write(log)


if __name__== "__main__":
    stream = Stream({'mimeType': 'audio/x-raw',
                     'channels': 2,
                     'bps': 2,
                     'rate': SAMPLE_RATE,
                     'samples': 1536},
                    {})
    #stream = Stream({},
                    #{'mimeType': 'video/x-raw',
                     #'width': 640,
                     #'height': 480,
                     #'bpp': 2,
                     #'fps': 30000 / 1001})
    #stream = Stream({'mimeType': 'audio/x-raw',
                     #'channels': 2,
                     #'bps': 2,
                     #'rate': SAMPLE_RATE,
                     #'samples': 1536},
                    #{'mimeType': 'video/x-raw',
                     #'width': 640,
                     #'height': 480,
                     #'bpp': 2,
                     #'fps': 30000 / 1001})
    decoder = Decoder(stream.readAudioPacket,
                      stream.readVideoPacket)
    sync = Sync(decoder.readAudioFrame,
                decoder.readVideoFrame)
    output = Output(SAMPLE_RATE, sync)
    logger = Logger(stream, sync)

    stream.start()
    decoder.start()
    sync.start()
    output.start()
