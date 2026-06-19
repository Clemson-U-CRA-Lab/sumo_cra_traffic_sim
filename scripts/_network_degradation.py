import csv
import heapq
import itertools
import random
from pathlib import Path


class NetworkDegradationLayer:
    """
    Simple attack/degradation layer for SUMO controller messages.

    It takes clean messages and can:
    - deliver normally outside the attack window
    - delay messages during the attack window
    - add jitter during the attack window
    - randomly drop messages during the attack window
    - leave the controller using the latest delivered/stale message
    """

    def __init__(
        self,
        enabled=True,
        attack_start=0.0,
        attack_end=0.0,
        base_delay_seconds=0.0,
        attack_delay_seconds=0.0,
        attack_jitter_seconds=0.0,
        attack_drop_probability=0.0,
        trace_path="networksim/results/network_degradation_trace.csv",
        seed=1,
    ):
        self.enabled = enabled
        self.attack_start = float(attack_start)
        self.attack_end = float(attack_end)
        self.base_delay_seconds = float(base_delay_seconds)
        self.attack_delay_seconds = float(attack_delay_seconds)
        self.attack_jitter_seconds = float(attack_jitter_seconds)
        self.attack_drop_probability = float(attack_drop_probability)

        self.random = random.Random(seed)
        self.queue = []
        self.counter = itertools.count()

        self.trace_path = Path(trace_path)
        self.trace_path.parent.mkdir(parents=True, exist_ok=True)

        self.trace_file = open(self.trace_path, "w", newline="")
        self.trace_writer = csv.writer(self.trace_file)

        self.trace_writer.writerow([
            "message_id",
            "send_time",
            "delivery_time",
            "controller_receive_time",
            "status",
            "delay_seconds",
            "message_age_seconds",
            "sender_id",
            "receiver_id",
            "reason",
        ])

    def _attack_active(self, sim_time):
        return self.attack_start <= sim_time <= self.attack_end

    def send(self, msg, sim_time):
        """
        Accept a clean SUMO message and either drop it or schedule delayed delivery.
        """

        sim_time = float(sim_time)
        message_id = msg.get("message_id", -1)

        if not self.enabled:
            delay = 0.0
            delivery_time = sim_time
            heapq.heappush(self.queue, (delivery_time, next(self.counter), msg))
            return

        if self._attack_active(sim_time):
            if self.random.random() < self.attack_drop_probability:
                self.trace_writer.writerow([
                    message_id,
                    sim_time,
                    "",
                    "",
                    "dropped",
                    "",
                    "",
                    msg.get("sender_id", ""),
                    msg.get("receiver_id", ""),
                    "attack_drop",
                ])
                return

            jitter = 0.0
            if self.attack_jitter_seconds > 0:
                jitter = self.random.uniform(
                    -self.attack_jitter_seconds,
                    self.attack_jitter_seconds
                )

            delay = max(0.0, self.attack_delay_seconds + jitter)
            reason = "attack_delay"
        else:
            delay = max(0.0, self.base_delay_seconds)
            reason = "normal_delivery"

        delivery_time = sim_time + delay
        heapq.heappush(self.queue, (delivery_time, next(self.counter), msg))

        self.trace_writer.writerow([
            message_id,
            sim_time,
            delivery_time,
            "",
            "scheduled",
            delay,
            "",
            msg.get("sender_id", ""),
            msg.get("receiver_id", ""),
            reason,
        ])

    def receive_available(self, sim_time):
        """
        Return all messages whose delivery time has arrived.
        """

        sim_time = float(sim_time)
        delivered = []

        while self.queue and self.queue[0][0] <= sim_time:
            delivery_time, _, msg = heapq.heappop(self.queue)

            out_msg = dict(msg)
            out_msg["delivery_time"] = delivery_time
            out_msg["controller_receive_time"] = sim_time
            out_msg["message_age_seconds"] = sim_time - float(msg["send_time"])

            delivered.append(out_msg)

            self.trace_writer.writerow([
                msg.get("message_id", -1),
                msg.get("send_time", ""),
                delivery_time,
                sim_time,
                "delivered",
                delivery_time - float(msg["send_time"]),
                sim_time - float(msg["send_time"]),
                msg.get("sender_id", ""),
                msg.get("receiver_id", ""),
                "controller_received",
            ])

        return delivered

    def close(self):
        self.trace_file.close()
