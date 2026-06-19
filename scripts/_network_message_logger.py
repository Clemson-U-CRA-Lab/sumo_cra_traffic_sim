import csv
from pathlib import Path


class NetworkMessageLogger:
    """
    Logs SUMO/TraCI vehicle state as network-style messages.

    First baseline coupling step:
    SUMO vehicle state -> message schedule CSV.
    """

    def __init__(self, output_path):
        self.output_path = Path(output_path)
        self.output_path.parent.mkdir(parents=True, exist_ok=True)

        self.file = open(self.output_path, "w", newline="")
        self.writer = csv.writer(self.file)

        self.writer.writerow([
            "message_id",
            "send_time",
            "sender_id",
            "receiver_id",
            "message_type",
            "packet_size_bytes",
            "position_x",
            "position_y",
            "speed",
            "acceleration",
        ])

        self.message_id = 0

    def log_status_message(
        self,
        send_time,
        sender_id,
        receiver_id,
        position_x,
        position_y,
        speed,
        acceleration,
        packet_size_bytes=300,
    ):
        self.writer.writerow([
            self.message_id,
            float(send_time),
            str(sender_id),
            str(receiver_id),
            "status",
            int(packet_size_bytes),
            float(position_x),
            float(position_y),
            float(speed),
            float(acceleration),
        ])

        self.message_id += 1

    def close(self):
        self.file.close()
