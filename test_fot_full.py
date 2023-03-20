import argparse
import time
from pathlib import Path

import matplotlib.patches as patch
import matplotlib.pyplot as plt
import numpy as np
from frenet_optimal_trajectory_planner.FrenetOptimalTrajectory import (
    fot_wrapper,
)


# Run fot planner
def fot(show_animation=False, show_info=False, num_threads=0, save_frame=False):
    conds = {
        "ps": 30,
        "target_speed": 30,
        "pos": np.array([983.43024, 5472.8867]),
        "vel": np.array([-0.0, 0.0]),
        "wp": np.array(
            [
                [983.43024, 5472.8867],
                [983.432, 5471.8867],
                [983.4338, 5470.8867],
                [983.4356, 5469.8867],
                [983.4374, 5468.8867],
                [983.43915, 5467.8867],
                [983.4409, 5466.8867],
                [983.44275, 5465.8867],
                [983.4445, 5464.8867],
                [983.4463, 5463.8867],
                [983.44806, 5462.8867],
                [983.4499, 5461.8867],
                [983.45166, 5460.8867],
                [983.4534, 5459.8867],
                [983.4552, 5458.8867],
                [983.45703, 5457.8867],
                [983.4588, 5456.8867],
                [983.4606, 5455.8867],
                [983.4624, 5454.8867],
                [983.4642, 5453.8867],
                [983.46594, 5452.887],
                [983.4677, 5451.887],
                [983.46954, 5450.887],
                [983.4713, 5449.887],
                [983.4731, 5448.887],
                [983.47485, 5447.887],
                [983.4767, 5446.887],
                [983.47845, 5445.887],
                [983.4802, 5444.887],
                [983.482, 5443.887],
                [983.4838, 5442.887],
                [983.4856, 5441.887],
                [983.48737, 5440.887],
                [983.48914, 5439.887],
                [983.49097, 5438.887],
                [983.49274, 5437.887],
                [983.4945, 5436.887],
                [983.4963, 5435.887],
                [983.4981, 5434.887],
                [983.4999, 5433.887],
                [983.50165, 5432.887],
                [983.5035, 5431.887],
                [983.50525, 5430.887],
                [983.507, 5429.887],
                [983.5088, 5428.887],
                [983.5106, 5427.887],
                [983.5124, 5426.887],
                [983.51416, 5425.887],
                [983.5159, 5424.887],
                [983.51776, 5423.887],
                [983.51953, 5422.887],
                [983.5213, 5421.887],
                [983.5231, 5420.887],
                [983.5249, 5419.887],
                [983.5267, 5418.887],
                [983.52844, 5417.887],
                [983.5302, 5416.887],
                [983.53204, 5415.887],
                [983.5338, 5414.887],
                [983.5356, 5413.887],
                [983.53735, 5412.887],
                [983.5392, 5411.887],
                [983.54095, 5410.887],
                [983.5427, 5409.887],
                [983.54456, 5408.887],
                [983.5463, 5407.887],
                [983.5481, 5406.887],
                [983.54987, 5405.887],
                [983.5517, 5404.887],
                [983.55347, 5403.887],
                [983.55524, 5402.887],
                [983.557, 5401.887],
                [983.55884, 5400.887],
                [983.5606, 5399.887],
                [983.5624, 5398.887],
                [983.56415, 5397.887],
                [983.566, 5396.887],
                [983.56775, 5395.887],
                [983.5695, 5394.887],
                [983.5713, 5393.887],
                [983.5731, 5392.887],
                [983.5749, 5391.887],
                [983.57666, 5390.887],
                [983.5784, 5389.887],
                [983.58026, 5388.887],
                [983.58203, 5387.887],
                [983.5838, 5386.887],
                [983.58563, 5385.887],
                [983.5874, 5384.887],
                [983.5892, 5383.887],
            ],
        ),
        "obs": np.array([[973.5106, 5427.887, 993.5106, 5437.887]]),
    }
    # paste output from debug log

    initial_conditions = conds

    hyperparameters = {
        "max_speed": 25.0,
        "max_accel": 15.0,
        "max_curvature": 15.0,
        "max_road_width_l": 0.5,
        "max_road_width_r": 0.5,
        "d_road_w": 0.5,
        "dt": 0.1,
        "maxt": 5.0,
        "mint": 0.5,
        "d_t_s": 5,
        "n_s_sample": 2.0,
        "obstacle_clearance": 0.1,
        "kd": 1.0,
        "kv": 0.1,
        "ka": 0.1,
        "kj": 0.1,
        "kt": 0.1,
        "ko": 0.1,
        "klat": 1.0,
        "klon": 1.0,
        "num_threads": 0,  # set 0 to avoid using threaded algorithm
    }

    # static elements of planner
    wx = initial_conditions["wp"][:, 0]
    wy = initial_conditions["wp"][:, 1]
    obs = np.array(conds["obs"])

    # simulation config
    sim_loop = 200
    area = 10
    total_time = 0
    time_list = []
    for i in range(sim_loop):
        # run FOT and keep time
        print("Iteration: {}".format(i))
        start_time = time.time()
        (
            result_x,
            result_y,
            speeds,
            ix,
            iy,
            iyaw,
            d,
            s,
            speeds_x,
            speeds_y,
            misc,
            costs,
            success,
        ) = fot_wrapper.run_fot(initial_conditions, hyperparameters)
        end_time = time.time() - start_time
        print("Time taken: {}".format(end_time))
        total_time += end_time
        time_list.append(end_time)

        # reconstruct initial_conditions
        if success:
            initial_conditions["pos"] = np.array([result_x[1], result_y[1]])
            initial_conditions["vel"] = np.array([speeds_x[1], speeds_y[1]])
            initial_conditions["ps"] = misc["s"]
            if show_info:
                print(costs)
        else:
            print("Failed unexpectedly")

        # break if near goal
        if np.hypot(result_x[1] - wx[-1], result_y[1] - wy[-1]) <= 1.0:
            print("Goal")

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                "key_release_event",
                lambda event: [exit(0) if event.key == "escape" else None],
            )
            plt.plot(wx, wy)
            if obs.shape[0] == 0:
                obs = np.empty((0, 4))
            ax = plt.gca()
            for o in obs:
                rect = patch.Rectangle((o[0], o[1]), o[2] - o[0], o[3] - o[1])
                ax.add_patch(rect)
            plt.plot(result_x[1:], result_y[1:], "-or")
            plt.plot(result_x[1], result_y[1], "vc")
            plt.xlim(result_x[1] - area, result_x[1] + area)
            plt.ylim(result_y[1] - area, result_y[1] + area)
            plt.xlabel("X axis")
            plt.ylabel("Y axis")
            plt.title(
                "v[m/s]:" + str(np.linalg.norm(initial_conditions["vel"]))[0:4]
            )
            plt.grid(True)
            if save_frame:
                Path("img/frames").mkdir(parents=True, exist_ok=True)
                plt.savefig("img/frames/{}.jpg".format(i))
            plt.pause(0.05)

    print("Finish")

    print("======================= SUMMARY ========================")
    print("Total time for {} iterations taken: {}".format(i, total_time))
    print("Max time per iteration: {}".format(max(time_list)))

    return time_list


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-d",
        "--display",
        action="store_true",
        help="show animation, ensure you have X11 forwarding server open",
    )
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="verbose mode, show all state info",
    )
    parser.add_argument(
        "-s",
        "--save",
        action="store_true",
        help="save each frame of simulation",
    )
    parser.add_argument(
        "-t",
        "--thread",
        type=int,
        default=0,
        help="set number of threads to run with",
    )
    args = parser.parse_args()

    # run planner with args passed in
    fot(args.display, args.verbose, args.thread, args.save)
