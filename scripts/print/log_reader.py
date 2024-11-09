import json
import datetime


class LogReader:
    """日志加载读取器，读取日志文件并解析"""

    def __init__(self, log_file):
        self.log_file = log_file
        self.start_time = 0

    def read(self) -> list[dict]:
        logs = []
        with open(self.log_file, "r", encoding="utf8") as f:
            for line in f:
                try:
                    logs.append(json.loads(line))
                except json.JSONDecodeError as e:
                    print(f"Skipping invalid JSON line: {line.strip()}")
                    print(f"Error: {e}")
        self.start_time = datetime.datetime.fromisoformat(logs[0]["timestamp"])
        return logs

    def read_by_level(self, level: str) -> list[dict]:
        return [log for log in self.read() if log["level"] == level]

    def read_by_node(self, node: str) -> list[dict]:
        return [log for log in self.read() if log["fields"]["node"] == node]

    def read_timestamps_by_keys(
        self, node: str, keys: list[str]
    ) -> list[datetime.datetime]:
        node_log = self.read_by_node(node)
        timestamps = [
            (
                datetime.datetime.fromisoformat(log["timestamp"]) - self.start_time
            ).total_seconds()
            for log in node_log
            if all(key in log["fields"] for key in keys)
        ]
        return timestamps

    def read_timestamp_value(self, node, key) -> tuple[list, list]:
        node_log = self.read_by_node(node)
        timestamps = [
            (
                datetime.datetime.fromisoformat(log["timestamp"]) - self.start_time
            ).total_seconds()
            for log in node_log
            if key in log["fields"]
        ]
        values = []
        for log in node_log:
            if key in log["fields"]:
                try:
                    value = json.loads(log["fields"][key])
                    values.append(value)
                except json.JSONDecodeError as e:
                    print(f"Skipping invalid JSON value: {log['fields'][key]}")
                    print(f"Error: {e}")
        return timestamps, values

    def read_usage_intervals(
        self, node: str
    ) -> tuple[list[datetime.datetime], list[datetime.datetime]]:
        begin = self.read_timestamps_by_keys(node, ["begin"])
        end = self.read_timestamps_by_keys(node, ["end"])
        return (begin, end)
