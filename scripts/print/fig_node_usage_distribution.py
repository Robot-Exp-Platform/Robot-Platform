import log_plotter
import matplotlib.pyplot as plt

LOG_PATH = "../../logs/info.json"

plotter = log_plotter.LogPlotter(LOG_PATH)

fig, (ax0, ax1) = plt.subplots(2, 1, figsize=(10, 8))

nodes = ["position:panda_1", "interp:panda_1", "cfs_end_pose:panda_1"]
plotter.plot_node_usage(ax0, nodes)

nodes = ["position:panda_2", "interp:panda_2", "cfs:panda_2"]
plotter.plot_node_usage(ax1, nodes)

plt.tight_layout()
plt.show()
