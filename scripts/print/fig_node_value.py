import log_plotter
import matplotlib.pyplot as plt

LOG_PATH = "../../logs/info.json"
plotter = log_plotter.LogPlotter(LOG_PATH)

fig, (ax0, ax1) = plt.subplots(2, 1, figsize=(10, 8))

# 输入需要绘制的 (node, key) 组合
node_key_pairs = [
    ("bullet:panda_1", "input"),
    ("position:panda_1", "input"),
    ("interp:panda_1", "input"),
]

plotter.plot_timestamp_value(ax0, node_key_pairs)

node_key_pairs = [
    ("bullet:panda_2", "input"),
    ("position:panda_2", "input"),
    ("interp:panda_2", "input"),
    ("cfs:panda_2", "input"),
]

plotter.plot_timestamp_value(ax1, node_key_pairs)

plt.tight_layout()
plt.show()
