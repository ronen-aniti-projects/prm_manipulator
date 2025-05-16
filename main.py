import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from astar import *
from obstacles import *
from prm import *
from robot import *


PRM_NODES = 20

def main():

    global PRM_NODES

    # Initialize pertinent objects
    robot = RobotArm()
    ob = Obstacles()
    prm = PRM(robot, ob.obstacles, N=PRM_NODES, K=5)
    
    # Set the query config
    c_start = np.array([0, 0, 0, 0, 0, 0], dtype=float)
    c_goal = c_start + np.pi

    print(f"c_start: {c_start}")
    print(f"c_goal: {c_goal}")
    astr = AStar(prm, c_start, c_goal)
    nodes = prm.nodes

    # for q in prm.nodes:
    # print(robot.generate_interpolated_robot_points(q))

    start_idx = astr.start
    goal_idx = astr.goal
    ee_start = robot.generate_link_end_points(prm.nodes[start_idx])[-1]
    ee_goal = robot.generate_link_end_points(prm.nodes[goal_idx])[-1]

    #    ee_positions
    ee_positions = np.array(
        [
            robot.generate_link_end_points(q)[-1]  # p_L is the last row
            for q in nodes
        ]
    )

    if True:
        try:
            fig = plt.figure(figsize=(14, 6))

            ax1 = fig.add_subplot(121, projection="3d")
            ax1.scatter(
                ee_positions[:, 0],
                ee_positions[:, 1],
                ee_positions[:, 2],
                c="blue",
                s=20,
                alpha=0.6,
                label="End Effector Positions",
            )
            for i, nbrs in prm.connections.items():
                p0 = ee_positions[i]
                for j, _ in nbrs:
                    p1 = ee_positions[j]
                    ax1.plot(
                        [p0[0], p1[0]],
                        [p0[1], p1[1]],
                        [p0[2], p1[2]],
                        c="gray",
                        linewidth=0.5,
                        alpha=0.5,
                    )

            # Start and goal end-effector positions
            ax1.scatter(
                ee_start[0],
                ee_start[1],
                ee_start[2],
                c="green",
                s=100,
                label="Start EE",
                marker="^",
            )
            ax1.scatter(
                ee_goal[0],
                ee_goal[1],
                ee_goal[2],
                c="red",
                s=100,
                label="Goal EE",
                marker="X",
            )

            ax1.set_title("PRM Roadmap: End Effector Positions")
            ax1.set_xlabel("X [m]")
            ax1.set_ylabel("Y [m]")
            ax1.set_zlabel("Z [m]")
            ax1.legend()
            ax1.set_box_aspect([1, 1, 1])

            ax2 = fig.add_subplot(122, projection="3d")
            # sample_indices = np.random.choice(
            #     len(prm.nodes), size=min(5, len(prm.nodes)), replace=False
            # )
            # for idx in sample_indices:
            # for idx in prm.nodes:

            for q in prm.nodes:
                # q = prm.nodes[idx]

                links = robot.generate_link_end_points(q)
                ax2.plot(links[:, 0], links[:, 1], links[:, 2], marker="o",c="blue")
                

            # Plot START robot configuration in green
            start_links = robot.generate_link_end_points(prm.nodes[start_idx])
            ax2.plot(
                start_links[:, 0],
                start_links[:, 1],
                start_links[:, 2],
                c="green",
                marker="^",
                linewidth=2,
                label="Start Config",
            )

            # Plot GOAL robot configuration in red
            goal_links = robot.generate_link_end_points(prm.nodes[goal_idx])
            ax2.plot(
                goal_links[:, 0],
                goal_links[:, 1],
                goal_links[:, 2],
                c="red",
                marker="X",
                linewidth=2,
                label="Goal Config",
            )
            ax2.set_title("Sample Robot Arm Configurations")
            ax2.set_xlabel("X [m]")
            ax2.set_ylabel("Y [m]")
            ax2.set_zlabel("Z [m]")
            ax2.set_box_aspect([1, 1, 1])
            ax2.legend()

            # Plot obstacles as translucent red boxes on right plot
            for xmin, ymin, zmin, xmax, ymax, zmax in ob.obstacles:
                c = [
                    [xmin, ymin, zmin],
                    [xmax, ymin, zmin],
                    [xmax, ymax, zmin],
                    [xmin, ymax, zmin],
                    [xmin, ymin, zmax],
                    [xmax, ymin, zmax],
                    [xmax, ymax, zmax],
                    [xmin, ymax, zmax],
                ]
                faces = [
                    [c[i] for i in f]
                    for f in [
                        [0, 1, 2, 3],
                        [4, 5, 6, 7],
                        [0, 1, 5, 4],
                        [2, 3, 7, 6],
                        [1, 2, 6, 5],
                        [3, 0, 4, 7],
                    ]
                ]
                ax2.add_collection3d(
                    Poly3DCollection(faces, facecolors="red", alpha=0.3, edgecolors="k")
                )

            plt.tight_layout()
            plt.show()

            ax = fig.add_subplot(111, projection="3d")

            ax.scatter(
                ee_positions[:, 0],  # x
                ee_positions[:, 1],  # y
                ee_positions[:, 2],  # z
                c="blue",
                s=20,
                alpha=0.6,
                label="End Effector Positions",
            )

            for i, nbrs in prm.connections.items():
                p0 = ee_positions[i]
                for j, _ in nbrs:
                    p1 = ee_positions[j]
                    ax.plot(
                        [p0[0], p1[0]],
                        [p0[1], p1[1]],
                        [p0[2], p1[2]],
                        c="gray",
                        linewidth=0.5,
                        alpha=0.5,
                    )

            ax.set_xlabel("X [m]")
            ax.set_ylabel("Y [m]")
            ax.set_zlabel("Z [m]")
            ax.set_title("PRM Roadmap for Barista Manipulator")
            ax.legend()
            ax.set_box_aspect([1, 1, 1])
            plt.tight_layout()
            plt.show()
        except Exception as e:
            print(f"Error: {e}")


if __name__ == "__main__":
    main()
