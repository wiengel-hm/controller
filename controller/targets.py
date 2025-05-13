from collections import deque
import numpy as np

class Target():
    def __init__(self, id, label, threshold=0.3, len_history=10, min_distance=1.5):
        self.id = id          # class ID
        self.label = label    # Descriptive label for the target (e.g., 'stop_sign')
        self.threshold = threshold  # Minimum ratio of positive detections to consider the target visible
        self.len_history = len_history  # Number of recent detection results to track
        self.history = deque([False] * self.len_history, maxlen=self.len_history)  # Detection history buffer
        self.has_reacted = False  # Flag to track if the system has already reacted to this target
        self.min_distance = min_distance  # Distance threshold to trigger a reaction (e.g., braking)
        self.position = None # Latest detected position of the target 

    @property
    def visible(self):
        # Compute visibility based on the average detection history
        return np.mean(self.history) > self.threshold

    @property
    def in_range(self):
        # Return True if position is known and x-distance is within the threshold
        return self.position is not None and self.position[0] <= self.min_distance


    def update(self, position, node):
        self.position = position

        # Update detection history based on current observation
        if np.isnan(position).any():
            self.history.append(False)
        else:
            self.history.append(True)

        # Trigger action if target is visible and in range and not yet reacted
        if self.visible and not self.has_reacted:
            if self.in_range:
                self.has_reacted = self.react(node)  # Pass node here

        # Reset if target is no longer visible
        elif not self.visible and self.has_reacted:
            self.has_reacted = False

    def react(self, node):
        # Override this method in subclasses
        return False

