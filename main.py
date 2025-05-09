from astar import *
from obstacles import *
from prm import *
from robot import *

def main():

    # Copy and paste chatGPT code to run and visualize quickly: 

    # 1) Instantiate robot & PRM with a small sample size for quick visualization
    robot     = RobotArm()
    ob = Obstacles()
    prm       = PRM(robot, ob.obstacles, N=20, K=5)
    c_start = np.array([0,0,0,0,0,0], dtype=float)
    c_goal = c_start + np.pi/4
    astr = AStar(prm, c_start, c_goal)
    
    

    # 2) For each PRM node, compute the end-effector (last link end) in world coords
    #    nodes: shape (100,6)
    nodes     = prm.nodes
    
    start_idx = astr.start 
    goal_idx = astr.goal
    ee_start = robot.generate_link_end_points(prm.nodes[start_idx])[-1]
    ee_goal = robot.generate_link_end_points(prm.nodes[goal_idx])[-1]

    #    ee_positions: shape (100,3)
    ee_positions = np.array([
        robot.generate_link_end_points(q)[-1]   # p_L is the last row
        for q in nodes
    ])

    # 3) Start a 1x2 subplot figure
    fig = plt.figure(figsize=(14,6))

    # --- Left plot: PRM end-effector roadmap ---
    ax1 = fig.add_subplot(121, projection='3d')
    ax1.scatter(
        ee_positions[:,0], ee_positions[:,1], ee_positions[:,2],
        c='blue', s=20, alpha=0.6, label='End Effector Positions'
    )
    for i, nbrs in prm.connections.items():
        p0 = ee_positions[i]
        for j, _ in nbrs:
            p1 = ee_positions[j]
            ax1.plot(
                [p0[0], p1[0]],
                [p0[1], p1[1]],
                [p0[2], p1[2]],
                c='gray', linewidth=0.5, alpha=0.5
            )

    # Start and goal end-effector positions
    ax1.scatter(
        ee_start[0], ee_start[1], ee_start[2],
        c='green', s=100, label='Start EE', marker='^'
    )
    ax1.scatter(
        ee_goal[0], ee_goal[1], ee_goal[2],
        c='red', s=100, label='Goal EE', marker='X'
    )

    ax1.set_title('PRM Roadmap: End Effector Positions')
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.set_zlabel('Z [m]')
    ax1.legend()
    ax1.set_box_aspect([1,1,1])
    
    # --- Right plot: full link visualizations for a few PRM nodes ---
    ax2 = fig.add_subplot(122, projection='3d')
    sample_indices = np.random.choice(len(prm.nodes), size=min(5, len(prm.nodes)), replace=False)
    for idx in sample_indices:
        q = prm.nodes[idx]
        links = robot.generate_link_end_points(q)
        ax2.plot(links[:,0], links[:,1], links[:,2], marker='o')

    # Plot START robot configuration in green
    start_links = robot.generate_link_end_points(prm.nodes[start_idx])
    ax2.plot(
        start_links[:,0], start_links[:,1], start_links[:,2],
        c='green', marker='^', linewidth=2, label='Start Config'
    )

    # Plot GOAL robot configuration in red
    goal_links = robot.generate_link_end_points(prm.nodes[goal_idx])
    ax2.plot(
        goal_links[:,0], goal_links[:,1], goal_links[:,2],
        c='red', marker='X', linewidth=2, label='Goal Config'
    )
    ax2.set_title('Sample Robot Arm Configurations')
    ax2.set_xlabel('X [m]')
    ax2.set_ylabel('Y [m]')
    ax2.set_zlabel('Z [m]')
    ax2.set_box_aspect([1,1,1])
    ax2.legend()

    """
    # Plot obstacles on the right plot too
    for obstacle in ob.obstacles:
        xmin, ymin, zmin, xmax, ymax, zmax = obstacle
        ax2.scatter([xmin, xmax], [ymin, ymax], [zmin, zmax], c='red', s=10)

    ax2.set_title('Sample Robot Arm Configurations')
    ax2.set_xlabel('X [m]')
    ax2.set_ylabel('Y [m]')
    ax2.set_zlabel('Z [m]')
    ax2.set_box_aspect([1,1,1])
    """
    plt.tight_layout()
    plt.show()

    ax  = fig.add_subplot(111, projection='3d')
    
    # 4) Plot PRM nodes (end-effector positions)
    ax.scatter(
        ee_positions[:,0],   # x
        ee_positions[:,1],   # y
        ee_positions[:,2],   # z
        c='blue', s=20, alpha=0.6, label='End Effector Positions'
    )

    # 5) Plot edges: for each connection i→j, draw a thin line between EE_i and EE_j
    for i, nbrs in prm.connections.items():
        p0 = ee_positions[i]
        for j, _ in nbrs:
            p1 = ee_positions[j]
            ax.plot(
                [p0[0], p1[0]],
                [p0[1], p1[1]],
                [p0[2], p1[2]],
                c='gray', linewidth=0.5, alpha=0.5
            )

    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.set_title('PRM Roadmap for Barista Manipulator')
    ax.legend()
    ax.set_box_aspect([1,1,1])
    plt.tight_layout()
    plt.show()
    pdb.set_trace()
    
if __name__ == "__main__":
    main()

