import itertools
import log_reader
import matplotlib.pyplot as plt


class LogPlotter:
    """日志文件绘制器，取出日志文件中的字段绘制为图像"""

    def __init__(self, log_file):
        self.log_reader = log_reader.LogReader(log_file)

    def plot_timestamp_value(self, ax: plt.Axes, node_key_pairs: list[tuple[str, str]]):
        """绘制时间戳和值的关系图"""

        color_cycle = plt.get_cmap("tab10").colors
        markers = itertools.cycle(["o", "s", "D", "^", "v", "<", ">"])

        for idx, (node, key) in enumerate(node_key_pairs):
            timestamps, values = self.log_reader.read_timestamp_value(node, key)
            data_by_dimension = list(zip(*values))  # 转置数据按维度绘制
            for dim_idx, dimension_data in enumerate(data_by_dimension):
                ax.plot(
                    timestamps,
                    dimension_data,
                    label=f"{node}:{key} dim={dim_idx}",
                    color=color_cycle[idx % len(color_cycle)],
                    marker=next(markers),
                    markersize=5,
                    linestyle="--",
                )
            print(f"idx: {idx}, node: {node}, key: {key}")

        ax.set_xlabel("Time")
        ax.set_ylabel("Values")
        ax.get_legend()
        # ax.set_xticks(rotation=45)

    def plot_node_usage(self, ax: plt.Axes, nodes: list[str]):
        """绘制节点使用情况图"""

        color_cycle = plt.get_cmap("tab10").colors

        for idx, node in enumerate(nodes):
            intervals = self.log_reader.read_usage_intervals(node)
            # 绘制空白时间段
            ax.barh(
                idx,
                intervals[1][-1] - intervals[0][0],  # 使用时间跨度
                left=intervals[0][0],
                height=0.05,
                color=color_cycle[idx % len(color_cycle)],
                alpha=0.3,
            )
            # 绘制使用中时间段
            for start, end in zip(intervals[0], intervals[1]):
                ax.barh(
                    idx,
                    end - start,
                    left=start,
                    height=0.05,
                    color=color_cycle[idx % len(color_cycle)],
                    alpha=0.8,
                )
                # 绘制开始和结束虚线
                ax.plot(
                    [start, start],
                    [idx - 1, idx],
                    color=color_cycle[idx % len(color_cycle)],
                    linestyle="--",
                )
                ax.plot(
                    [end, end],
                    [idx - 1, idx],
                    color=color_cycle[idx % len(color_cycle)],
                    linestyle="--",
                )
            print(f"idx: {idx}, node: {node}")

        ax.set_xlabel("Time")
        ax.set_ylabel("Node Usage")
        # ax.set_xticks(rotation=45)
        ax.set_yticks(range(len(nodes)), nodes)
        ax.set_ylim(-0.2, None)
