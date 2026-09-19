"""The measured output-shaft error, as something that can be applied.

gear_error.py measures what the AS5600 cannot see; this turns that
measurement into a correction and lets a run pre-compensate for it. The
rotator's own closed loop drives the *sensor* to the commanded angle, so
the output ends up at

    true = commanded + g(commanded)

and asking for `commanded - g(commanded)` instead lands on `true` to first
order - which is enough, because g is at most 0.08 degrees and its own
slope contributes below a tenth of a mdeg.

The model is only valid while the machine's mechanical zero is where it was
when the measurement was taken, since its phase is referenced to that.
"""

import json
import math


class GearModel:
    """Harmonics of the output-shaft error against the machine's MECHANICAL
    angle, plus the offset from the Alpaca scale to that angle.

    The two scales are not the same: measured live on 2026-09-18, Alpaca
    Position sat 20.2539 degrees from the angle the step counter counts, and
    the model's phase is tied to the latter. Evaluating it at an Alpaca
    target without converting would shift a 20th-order harmonic, whose
    period is 18 degrees, by more than a whole cycle - the correction would
    then add error rather than remove it."""

    def __init__(self, orders, constant, cos, sin, alpaca_offset=0.0):
        self.orders, self.constant, self.cos, self.sin = orders, constant, cos, sin
        self.alpaca_offset = alpaca_offset

    @classmethod
    def from_json(cls, path):
        with open(path) as handle:
            data = json.load(handle)
        model = data["model"]
        return cls(model["orders"], model["constant"], model["cos"], model["sin"],
                   model.get("alpacaToMechanicalDeg", 0.0))

    @classmethod
    def average(cls, paths):
        """Mean of several measurements of the same machine - the campaign
        took two, at different step spacings, precisely so neither run's own
        aliasing ends up baked into the correction."""
        models = [cls.from_json(p) for p in paths]
        orders = models[0].orders
        for model in models[1:]:
            if model.orders != orders:
                raise ValueError("models were fitted on different orders")
        count = len(models)
        offsets = {round(m.alpaca_offset, 6) for m in models}
        if len(offsets) > 1:
            raise ValueError(f"models carry different Alpaca offsets: {offsets}")
        return cls(orders,
                   sum(m.constant for m in models) / count,
                   [sum(m.cos[i] for m in models) / count for i in range(len(orders))],
                   [sum(m.sin[i] for m in models) / count for i in range(len(orders))],
                   models[0].alpaca_offset)

    def at_mechanical(self, angle_deg):
        phase = math.radians(angle_deg)
        value = self.constant
        for index, k in enumerate(self.orders):
            value += (self.cos[index] * math.cos(k * phase)
                      + self.sin[index] * math.sin(k * phase))
        return value

    def __call__(self, alpaca_deg):
        """Error at an angle given on the Alpaca scale."""
        return self.at_mechanical(alpaca_deg + self.alpaca_offset)

    def precompensate(self, alpaca_target_deg):
        return alpaca_target_deg - self(alpaca_target_deg)
