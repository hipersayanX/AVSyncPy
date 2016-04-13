#!/usr/bin/env python
# -*- coding: UTF-8 -*-
#
# AVSyncPy, Lip synchronization algorith implemented in Python.
# Copyright (C) 2013-2016  Gonzalo Exequiel Pedone
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
PLAIN_LOG = False

# Max package queue size in bytes.
MAX_QUEUE_SIZE = 15 * 1024 * 1024

# no AV sync correction is done if below the minimum AV sync threshold.
AV_SYNC_THRESHOLD_MIN = 0.04

# AV sync correction is done if above the maximum AV sync threshold.
AV_SYNC_THRESHOLD_MAX = 0.1

# If a frame duration is longer than this, it will not be duplicated to compensate AV sync
AV_SYNC_FRAMEDUP_THRESHOLD = 0.1

# no AV correction is done if too big error.
AV_NOSYNC_THRESHOLD = 10.0

# maximum audio speed change to get correct sync.
SAMPLE_CORRECTION_PERCENT_MAX = 10

# we use about AUDIO_DIFF_AVG_NB A-V differences to make the average.
AUDIO_DIFF_AVG_NB = 20

# Max number of video frames.
VIDEO_PICTURE_QUEUE_SIZE = 3

# Max number of samples.
SAMPLE_QUEUE_SIZE = 9

class Stream:
    def __init__(self, aPacket, vPacket):
        self.aPacket = aPacket
        self.vPacket = vPacket

        # Initialize AV frames.
        self.aN = 0 # Number of audio frame

        if self.aPacket == {}:
            # No audio stream.
            self.aDuration = 0
            self.aRawSize = 0
            self.aCompressedSize = 0
        else:
            self.aDuration = aPacket['samples'] / aPacket['rate']
            self.aRawSize = aPacket['bps'] * aPacket['channels'] * aPacket['samples'] / 8
            self.aCompressedSize = self.aDuration * aPacket['bitrate'] / 8

        self.vN = 0 # Number of video frame

        if self.vPacket == {}:
            # No video stream.
            self.vDuration = 0
            self.vRawSize = 0
            self.vCompressedSize = 0
        else:
            self.vDuration = 1 / vPacket['fps']
            self.vRawSize = vPacket['width'] * vPacket['height'] * vPacket['bpp'] / 8
            self.vCompressedSize = self.vDuration * vPacket['bitrate'] / 8

        # Packet queue.
        self.queueSize = 0      # Size of packet queue in bytes.
        self.audioQueueSize = 0 # Size of audio queue in bytes.
        self.videoQueueSize = 0 # Size of video queue in bytes.

        self.audioPacketQueue = []
        self.videoPacketQueue = []

        # Setup threads.
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

            self.queueMutex.acquire()

            # Wait until there's enough space to store a new packet.
            if self.queueSize >= MAX_QUEUE_SIZE:
                self.queueNotFull.wait()

            self.queueMutex.release()

            # Get an audio frame.
            if (aPts <= vPts and self.aPacket != {}) or self.vPacket == {}:
                    packet = {'pts': aPts,
                              'duration': self.aDuration,
                              'rawSize': self.aRawSize,
                              'compressedSize': self.aCompressedSize}
                    packet.update(self.aPacket)
                    self.aN += 1

                    self.audioMutex.acquire()
                    self.audioPacketQueue.append(packet)
                    self.audioQueueSize += packet['compressedSize']
                    self.aQueueNotEmpty.notifyAll()
                    self.audioMutex.release()

                    self.queueMutex.acquire()
                    self.queueSize += packet["compressedSize"]
                    self.queueMutex.release()
            # Get an video frame.
            else:
                if self.vPacket != {}:
                    packet = {'pts': vPts,
                              'duration': self.vDuration,
                              'rawSize': self.vRawSize,
                              'compressedSize': self.vCompressedSize}
                    packet.update(self.vPacket)
                    self.vN += 1

                    self.videoMutex.acquire()
                    self.videoPacketQueue.append(packet)
                    self.videoQueueSize += packet['compressedSize']
                    self.vQueueNotEmpty.notifyAll()
                    self.videoMutex.release()

                    self.queueMutex.acquire()
                    self.queueSize += packet["compressedSize"]
                    self.queueMutex.release()

    def readAudioPacket(self):
        self.audioMutex.acquire()

        # Wait until there are at least one audio packet.
        if len(self.audioPacketQueue) < 1:
            if not self.aQueueNotEmpty.wait(1):
                # If timeout, return a null packet.
                self.audioMutex.release()

                return {}

        packet = self.audioPacketQueue.pop(0)
        self.audioQueueSize -= packet['compressedSize']
        self.audioMutex.release()

        self.queueMutex.acquire()
        self.queueSize -= packet["compressedSize"]

        if self.queueSize < MAX_QUEUE_SIZE:
            self.queueNotFull.notifyAll()

        self.queueMutex.release()

        return packet

    def readVideoPacket(self):
        self.videoMutex.acquire()

        # Wait until there are at least one video packet.
        if len(self.videoPacketQueue) < 1:
            if not self.vQueueNotEmpty.wait(1):
                # If timeout, return a null packet.
                self.videoMutex.release()

                return {}

        packet = self.videoPacketQueue.pop(0)
        self.videoQueueSize -= packet['compressedSize']
        self.videoMutex.release()

        self.queueMutex.acquire()
        self.queueSize -= packet["compressedSize"]

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
        self.audioFrames = 0 # Number of frames in the audio queue.
        self.videoFrames = 0 # Number of frames in the video queue.

        self.audioFrameQueue = []
        self.videoFrameQueue = []

        # Setup threads.
        self.audioMutex = threading.Lock()
        self.videoMutex = threading.Lock()
        self.audioQueueNotFull = threading.Condition(self.audioMutex)
        self.videoQueueNotFull = threading.Condition(self.videoMutex)
        self.aQueueNotEmpty = threading.Condition(self.audioMutex)
        self.vQueueNotEmpty = threading.Condition(self.videoMutex)

    def decodeAudio(self):
        while True:
            # Read an audio packet.
            packet = self.audioReadFunc()

            if packet == {}:
                continue

            # Simulate frame decoding
            time.sleep(random.uniform(0.0, 0.25) * packet['duration'])

            # NOTE: One audio packet isn't necessarily equal to one audio frame.
            # One audio packet can contain more than one audio frame, but it's
            # ok for our purposes.
            # If in the decoding process you get more than one audio frame, join
            # the frames into an audio buffer, and send frames in chunks
            # (preferably) not bigger than 1024 samples.

            self.audioMutex.acquire()

            if self.audioFrames >= SAMPLE_QUEUE_SIZE:
                self.audioQueueNotFull.wait()

            # Append decoded frame to the audio queue.
            self.audioFrameQueue.append(packet)
            self.audioFrames += 1
            self.aQueueNotEmpty.notifyAll()
            self.audioMutex.release()

    def decodeVideo(self):
        while True:
            # Read a video packet.
            packet = self.videoReadFunc()

            if packet == {}:
                continue

            # Simulate frame decoding
            time.sleep(random.uniform(0.0, 0.25) * packet['duration'])

            self.videoMutex.acquire()

            if self.videoFrames >= VIDEO_PICTURE_QUEUE_SIZE:
                self.videoQueueNotFull.wait()

            # Append decoded frame to the video queue.
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
        self.audioFrames -= 1

        if self.audioFrames < SAMPLE_QUEUE_SIZE:
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

        if self.videoFrames < VIDEO_PICTURE_QUEUE_SIZE:
            self.videoQueueNotFull.notifyAll()

        self.videoMutex.release()

        return frame

    def start(self):
        threading.Thread(target=self.decodeAudio, name='AudioDecoding').start()
        threading.Thread(target=self.decodeVideo, name='VideoDecoding').start()

class Output:
    def __init__(self, syncObject):
        syncObject.playFrame = self.playFrame
        syncObject.drawFrame = self.drawFrame

    # Simulate audio playback.
    def playFrame(self, frame):
        time.sleep(frame['samples'] / frame['rate'])

    # Simulate frame drawing in the screen.
    def drawFrame(self, frame):
        # A frame can take longer to draw depending on it's resolution,
        # but i don't know how much.
        time.sleep(random.uniform(0.0, 0.25) * frame['duration'])

# I use global clock for syncing.
class Clock:
    def __init__(self):
        self.timeDrift = 0
        self.mutex = threading.Lock()

    def clock(self):
        self.mutex.acquire()
        clock = time.time() - self.timeDrift
        self.mutex.release()

        return clock

    def setClock(self, clock):
        self.mutex.acquire()
        self.timeDrift = time.time() - clock
        self.mutex.release()

class Sync:
    def __init__(self, readAudioFunc, readVideoFunc):
        self.readAudioFunc = readAudioFunc
        self.readVideoFunc = readVideoFunc

        self.globalClock = Clock()

        # Audio Parameters
        self.lastAudioPts = 0
        self.audioDiffAvgCoef = math.exp(math.log(0.01) / AUDIO_DIFF_AVG_NB)
        self.audioDiffCum = 0
        self.audioDiffAvgCount = 0

        # Video Parameters
        self.lastVideoPts = 0
        self.curFrame = {}
        self.framesDropped = 0

    def processAudioFrame(self):
        while True:
            # Read an audio frame.
            frame = self.readAudioFunc()

            if frame == {}:
                continue

            pts = frame['pts']
            diff = pts - self.globalClock.clock()
            outputSamples = frame['samples']

            if not math.isnan(diff) and abs(diff) < AV_NOSYNC_THRESHOLD:
                self.audioDiffCum = diff + self.audioDiffAvgCoef * self.audioDiffCum

                if self.audioDiffAvgCount < AUDIO_DIFF_AVG_NB:
                    # not enough measures to have a correct estimate
                    self.audioDiffAvgCount += 1
                else:
                    # estimate the A-V difference
                    avgDiff = self.audioDiffCum * (1.0 - self.audioDiffAvgCoef)
                    audioDiffThreshold = frame['samples'] / frame['rate']

                    if abs(avgDiff) >= audioDiffThreshold:
                        outputSamples = frame['samples'] + diff * frame['rate']
                        minSamples = frame['samples'] * (1.0 - SAMPLE_CORRECTION_PERCENT_MAX / 100)
                        maxSamples = frame['samples'] * (1.0 + SAMPLE_CORRECTION_PERCENT_MAX / 100)
                        outputSamples = int(max(minSamples, min(outputSamples, maxSamples)))
            else:
                # too big difference: may be initial PTS errors, so
                # reset A-V filter
                self.audioDiffAvgCount = 0
                self.audioDiffCum = 0

            # Compensate audio.
            frame['samples'] = outputSamples

            if abs(diff) >= AV_NOSYNC_THRESHOLD:
                self.globalClock.setClock(pts)

            self.lastAudioPts = pts
            self.logger.log()
            self.playFrame(frame)

    def processVideoFrame(self):
        while True:
            # If there's not a current frame to process, read an new video frame.
            if self.curFrame == {}:
                self.curFrame = self.readVideoFunc()

            if self.curFrame == {}:
                continue

            pts = self.curFrame['pts']
            diff = pts - self.globalClock.clock()
            delay = pts - self.lastVideoPts

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
                    self.lastVideoPts = pts
                    self.framesDropped += 1

                    continue
                elif diff > syncThreshold:
                    # video is ahead the external clock.
                    time.sleep(diff - syncThreshold)

                    continue
            else:
                # Resync video.
                self.globalClock.setClock(pts)

            self.lastVideoPts = pts
            self.logger.log()
            self.drawFrame(self.curFrame)
            self.curFrame = {}

    def start(self):
        threading.Thread(target=self.processAudioFrame, name='SyncAudio').start()
        threading.Thread(target=self.processVideoFrame, name='SyncVideo').start()

class Logger:
    def __init__(self, stream, sync):
        self.stream = stream
        self.sync = sync
        self.sync.logger = self

    def streamLog(self):
        print('Streams:')

        vInfo = '    Video: {} bpp, {}x{}, {} kb/s, {:.2f} fps'

        print(vInfo.format(self.stream.vPacket['bpp'],
                           self.stream.vPacket['width'],
                           self.stream.vPacket['height'],
                           int(self.stream.vPacket['bitrate'] / 1000),
                           self.stream.vPacket['fps']))

        aInfo = '    Audio: {} Hz, {} ch, {} bps, {} kb/s'

        print(aInfo.format(self.stream.aPacket['rate'],
                           self.stream.aPacket['channels'],
                           self.stream.aPacket['bps'],
                           int(self.stream.aPacket['bitrate'] / 1000)))

        print()

    def log(self):
        if LOG:
            streamType = ''
            diff = 0

            if self.stream.aPacket != {} and self.stream.vPacket != {}:
                streamType = 'A-V'
                diff = self.sync.lastAudioPts - self.sync.lastVideoPts
            elif self.stream.aPacket != {}:
                streamType = 'M-A'
                diff = self.sync.globalClock.clock() - self.sync.lastAudioPts
            elif self.stream.vPacket != {}:
                streamType = 'M-V'
                diff = self.sync.globalClock.clock() - self.sync.lastVideoPts
            else:
                return

            logFmt = '\r {:7.2f} {}: {:7.3f} fd={:5d} aq={:5d}KB vq={:5d}KB'

            if PLAIN_LOG:
                logFmt += '\n'

            log = logFmt.format(self.sync.globalClock.clock(),
                                streamType,
                                diff,
                                self.sync.framesDropped,
                                int(self.stream.audioQueueSize / 1024),
                                int(self.stream.videoQueueSize / 1024))

            sys.stdout.write(log)


if __name__== "__main__":
    #stream = Stream({'mimeType': 'audio/x-raw',
                     #'channels': 2,
                     #'bps': 2,
                     #'rate': 44100,
                     #'samples': 1024,
                     #'bitrate': 128000},
                    #{})
    #stream = Stream({},
                    #{'mimeType': 'video/x-raw',
                     #'width': 640,
                     #'height': 480,
                     #'bpp': 16,
                     #'fps': 30000 / 1001,
                     #'bitrate': 200000})
    stream = Stream({'mimeType': 'audio/x-raw',
                     'channels': 2,
                     'bps': 16,
                     'rate': 44100,
                     'samples': 1024,
                     'bitrate': 128000},
                    {'mimeType': 'video/x-raw',
                     'width': 640,
                     'height': 480,
                     'bpp': 16,
                     'fps': 30000 / 1001,
                     'bitrate': 200000})
    decoder = Decoder(stream.readAudioPacket,
                      stream.readVideoPacket)
    sync = Sync(decoder.readAudioFrame,
                decoder.readVideoFrame)
    output = Output(sync)
    logger = Logger(stream, sync)

    logger.streamLog()
    stream.start()
    decoder.start()
    sync.start()
