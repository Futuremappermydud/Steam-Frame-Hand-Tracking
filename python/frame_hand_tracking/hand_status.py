import math

STAGE_STATUS = {2: 'not_a_hand', 3: 'lost', 4: 'too_far', 6: 'leaving_view', 7: 'starting', 8: 'too_close'}
UNSURE_HAND_CONFIDENCE = 0.1


def frame_status(hand, quality, unsure_below=UNSURE_HAND_CONFIDENCE):
    if hand is not None:
        confidence = hand.get('confidence')
        unsure = confidence is not None and math.isfinite(confidence) and confidence < unsure_below
        return 'unsure' if unsure else 'tracked'
    return STAGE_STATUS.get((quality or {}).get('rejection_stage', 0), 'not_seen')


class HandStatus:
    def __init__(self, settle_s=.15):
        self.settle_s = settle_s
        self.shown, self.pending, self.since = 'not_seen', None, 0.

    def update(self, status, t_s):
        if status == self.shown:
            self.pending = None
        elif status != self.pending:
            self.pending, self.since = status, t_s
        if self.pending is not None and t_s - self.since >= self.settle_s:
            self.shown, self.pending = self.pending, None
        return self.shown
