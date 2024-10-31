import re
import datetime
import itertools
import matplotlib.pyplot as plt


class LogPlotter:
    """LogPlotter class to parse and plot log data"""

    def __init__(self, log_file_path):
        self.log_file_path = log_file_path
        self.logs = []
        self.pattern = re.compile(
            (
                r"(?P<timestamp>\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}\.\d+Z)\s+INFO\s+(?P<module>\S+):\s+"
                r'node="(?P<node>[^"]+)"\s+input=\[(?P<input>[^\]]+)\](?:\s+current=\[(?P<current>[^\]]+)\])?'
            )
        )

    def parse_logs(self):
        with open(self.log_file_path, "r", encoding="utf8") as file:
            for line in file:
                match = self.pattern.search(line)
                if match:
                    log_dict = {
                        "timestamp": datetime.datetime.fromisoformat(
                            match.group("timestamp").replace("Z", "+00:00")
                        ),
                        "module": match.group("module"),
                        "node": match.group("node"),
                        "input": [float(x) for x in match.group("input").split(",")],
                    }
                    self.logs.append(log_dict)

    def filter_logs(self, node_key_pairs):
        # 按照node_key_pairs筛选日志数据
        filtered_data = {}
        for node, key in node_key_pairs:
            filtered_logs = [
                log for log in self.logs if log["node"] == node and key in log
            ]
            if filtered_logs:
                timestamps = [log["timestamp"] for log in filtered_logs]
                data_values = [log[key] for log in filtered_logs]
                filtered_data[(node, key)] = (timestamps, data_values)
        return filtered_data

    def plot(self, node_key_pairs):
        filtered_data = self.filter_logs(node_key_pairs)

        plt.figure(figsize=(12, 8))

        color_cycle = plt.get_cmap("tab10").colors  # 使用颜色循环
        markers = itertools.cycle(["o", "s", "D", "^", "v", "<", ">"])

        for idx, ((node, key), (timestamps, data_values)) in enumerate(
            filtered_data.items()
        ):
            data_by_dimension = list(zip(*data_values))  # 转置数据按维度绘制
            for dim_idx, dimension_data in enumerate(data_by_dimension):
                plt.plot(
                    timestamps,
                    dimension_data,
                    label=f"{node} - {key} - Dimension {dim_idx+1}",
                    color=color_cycle[idx % len(color_cycle)],
                    marker=next(markers),
                    markersize=5,
                    linestyle="-",
                )

        plt.xlabel("Time")
        plt.ylabel("Data Values")
        plt.title("Node Data Over Time")
        # plt.legend()
        plt.xticks(rotation=45)
        plt.tight_layout()
        plt.show()


# 使用示例
LOG_PATH = "../../logs/test.log"
plotter = LogPlotter(LOG_PATH)
plotter.parse_logs()

# 输入需要绘制的 (node, key) 组合
node_key_pairs = [
    ("cfs:panda_1", "input"),
    ("interp:panda_1", "input"),
    ("position:panda_1", "input"),
    ("bullet:panda_1", "input"),
]

plotter.plot(node_key_pairs)
