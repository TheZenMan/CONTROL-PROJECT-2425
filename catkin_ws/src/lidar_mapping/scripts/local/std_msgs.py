#!/usr/bin/env python3


class Header:
    def __init__(self, seq=0, stamp=0, frame_id=""):
        self.seq = seq
        self.stamp = stamp
        self.frame_id = frame_id

    def __repr__(self):
        return "%s, %s, %s" % (self.seq, self.stamp, self.frame_id)

    def __str__(self):
        return "%s, %s, %s" % (self.seq, self.stamp, self.frame_id)
