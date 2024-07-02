#!/usr/bin/env python

from sort import Sort

class ObjectTracker:
    def __init__(self):        
        self.tracker = Sort()
        
    def __call__(self, detected_objects):
        tracked_objects = self.track_objects(detected_objects)

        return tracked_objects
    
    def track_objects(self, detected_objects):
        tracked_objects = self.tracker.update(detected_objects)

        return tracked_objects