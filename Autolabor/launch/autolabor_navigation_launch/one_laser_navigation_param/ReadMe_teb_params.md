参数	类型	含义	最小	默认	最大
teb_autosize	bool	优化期间允许改变轨迹的时域长度；Enable the automatic resizing of the trajectory during optimization (based on the temporal resolution of the trajectory, recommended)	False	True	True
dt_ref	double	局部路径规划的解析度; Temporal resolution of the planned trajectory (usually it is set to the magnitude of the 1/control_rate)	0.01	0.3	1.0
dt_hysteresis	double	允许改变的时域解析度的浮动范围， 一般为 dt_ref 的 10% 左右; Hysteresis that is utilized for automatic resizing depending on the current temporal resolution (dt): usually 10% of dt_ref	0.002	0.1	0.5
global_plan_overwrite_orientation	bool	覆盖全局路径中局部路径点的朝向，Some global planners are not considering the orientation at local subgoals between start and global goal, therefore determine it automatically	False	True	True
allow_init_with_backwards_motion	bool	允许在开始时想后退来执行轨迹，If true, the underlying trajectories might be initialized with backwards motions in case the goal is behind the start within the local costmap (this is only recommended if the robot is equipped with rear sensors)	False	False	True
max_global_plan_lookahead_dist	double	考虑优化的全局计划子集的最大长度（累积欧几里得距离）（如果为0或负数：禁用；长度也受本地Costmap大小的限制）， Specify maximum length (cumulative Euclidean distances) of the subset of the global plan taken into account for optimization [if 0 or negative: disabled; the length is also bounded by the local costmap size ]	0.0	3.0	50.0
force_reinit_new_goal_dist	double	如果上一个目标的间隔超过指定的米数（跳过热启动），则强制规划器重新初始化轨迹，Force the planner to reinitialize the trajectory if a previous goal is updated with a seperation of more than the specified value in meters (skip hot-starting)	0.0	1.0	10.0
feasibility_check_no_poses	int	检测位姿可到达的时间间隔，Specify up to which pose on the predicted plan the feasibility should be checked each sampling interval	0	5	50
exact_arc_length	bool	如果为真，规划器在速度、加速度和转弯率计算中使用精确的弧长[->增加的CPU时间]，否则使用欧几里德近似。If true, the planner uses the exact arc length in velocity, acceleration and turning rate computations [-> increased cpu time], otherwise the euclidean approximation is used.	False	False	True
publish_feedback	bool	发布包含完整轨迹和活动障碍物列表的规划器反馈，Publish planner feedback containing the full trajectory and a list of active obstacles (should be enabled only for evaluation or debugging purposes)	False	False	True
visualize_with_time_as_z_axis_scale	double	如果该值大于0，则使用该值缩放的Z轴的时间在3D中可视化轨迹和障碍物。最适用于动态障碍。 If this value is bigger than 0, the trajectory and obstacles are visualized in 3d using the time as the z-axis scaled by this value. Most useful for dynamic obstacles.	0.0	0.0	1.0
global_plan_viapoint_sep	double	从全局计划中提取的每两个连续通过点之间的最小间隔[如果为负：禁用]， Min. separation between each two consecutive via-points extracted from the global plan [if negative: disabled]	-0.1	-0.1	5.0
via_points_ordered	bool	如果为真，规划器遵循存储容器中通过点的顺序。If true, the planner adheres to the order of via-points in the storage container	False	False	True
max_vel_x	double	最大x前向速度，Maximum translational velocity of the robot	0.01	0.4	100.0
max_vel_x_backwards	double	最大x后退速度，Maximum translational velocity of the robot for driving backwards	0.01	0.2	100.0
max_vel_theta	double	最大转向叫速度 Maximum angular velocity of the robot	0.01	0.3	100.0
acc_lim_x	double	最大x加速度，Maximum translational acceleration of the robot	0.01	0.5	100.0
acc_lim_theta	double	最大角速度，Maximum angular acceleration of the robot	0.01	0.5	100.0
is_footprint_dynamic	bool	是否footprint 为动态的，If true, updated the footprint before checking trajectory feasibility	False	False	True
min_turning_radius	double	车类机器人的最小转弯半径，Minimum turning radius of a carlike robot (diff-drive robot: zero)	0.0	0.0	50.0
wheelbase	double	驱动轴和转向轴之间的距离（仅适用于启用了“Cmd_angle_而不是_rotvel”的Carlike机器人）；对于后轮式机器人，该值可能为负值！ The distance between the drive shaft and steering axle (only required for a carlike robot with ‘cmd_angle_instead_rotvel’ enabled); The value might be negative for back-wheeled robots!	-10.0	1.0	10.0
cmd_angle_instead_rotvel	bool	将收到的角速度消息转换为 操作上的角度变化。 Substitute the rotational velocity in the commanded velocity message by the corresponding steering angle (check ‘axles_distance’)	False	False	True
max_vel_y	double	最大y方向速度， Maximum strafing velocity of the robot (should be zero for non-holonomic robots!)	0.0	0.0	100.0
acc_lim_y	double	最大y向加速度， Maximum strafing acceleration of the robot	0.01	0.5	100.0
xy_goal_tolerance	double	目标 xy 偏移容忍度，Allowed final euclidean distance to the goal position	0.001	0.2	10.0
yaw_goal_tolerance	double	目标 角度 偏移容忍度， Allowed final orientation error to the goal orientation	0.001	0.1	3.2
free_goal_vel	bool	允许机器人以最大速度驶向目的地， Allow the robot’s velocity to be nonzero for planning purposes (the robot can arrive at the goal with max speed)	False	False	True
min_obstacle_dist	double	和障碍物最小距离， Minimum desired separation from obstacles	0.0	0.5	10.0
inflation_dist	double	障碍物膨胀距离， Buffer zone around obstacles with non-zero penalty costs (should be larger than min_obstacle_dist in order to take effect)	0.0	0.6	15.0
dynamic_obstacle_inflation_dist	double	动态障碍物的膨胀范围， Buffer zone around predicted locations of dynamic obstacles with non-zero penalty costs (should be larger than min_obstacle_dist in order to take effect)	0.0	0.6	15.0
include_dynamic_obstacles	bool	是否将动态障碍物预测为速度模型， Specify whether the movement of dynamic obstacles should be predicted by a constant velocity model (this also changes the homotopy class search). If false, all obstacles are considered to be static.	False	False	True
include_costmap_obstacles	bool	costmap 中的障碍物是否被直接考虑， Specify whether the obstacles in the costmap should be taken into account directly (this is necessary if no seperate clustering and detection is implemented)	False	True	True
legacy_obstacle_association	bool	是否严格遵循局部规划出来的路径， If true, the old association strategy is used (for each obstacle, find the nearest TEB pose), otherwise the new one (for each teb pose, find only ‘relevant’ obstacles).	False	False	True
obstacle_association_force_inclusion_factor	double	The non-legacy obstacle association technique tries to connect only relevant obstacles with the discretized trajectory during optimization, all obstacles within a specifed distance are forced to be included (as a multiple of min_obstacle_dist), e.g. choose 2.0 in order to consider obstacles within a radius of 2.0*min_obstacle_dist.	0.0	1.5	100.0
obstacle_association_cutoff_factor	double	See obstacle_association_force_inclusion_factor, but beyond a multiple of [value]*min_obstacle_dist all obstacles are ignored during optimization. obstacle_association_force_inclusion_factor is processed first.	1.0	5.0	100.0
costmap_obstacles_behind_robot_dist	double	Limit the occupied local costmap obstacles taken into account for planning behind the robot (specify distance in meters)	0.0	1.5	20.0
obstacle_poses_affected	int	The obstacle position is attached to the closest pose on the trajectory to reduce computational effort, but take a number of neighbors into account as well	0	30	200
no_inner_iterations	int	被外循环调用后内循环执行优化次数， Number of solver iterations called in each outerloop iteration	1	5	100
no_outer_iterations	int	执行的外循环的优化次数， Each outerloop iteration automatically resizes the trajectory and invokes the internal optimizer with no_inner_iterations	1	4	100
optimization_activate	bool	激活优化， Activate the optimization	False	True	True
optimization_verbose	bool	打印优化过程详情， Print verbose information	False	False	True
penalty_epsilon	double	对于硬约束近似，在惩罚函数中添加安全范围， Add a small safty margin to penalty functions for hard-constraint approximations	0.0	0.1	1.0
weight_max_vel_x	double	最大x速度权重， Optimization weight for satisfying the maximum allowed translational velocity	0.0	2.0	1000.0
weight_max_vel_y	double	最大y速度权重，Optimization weight for satisfying the maximum allowed strafing velocity (in use only for holonomic robots)	0.0	2.0	1000.0
weight_max_vel_theta	double	最大叫速度权重， Optimization weight for satisfying the maximum allowed angular velocity	0.0	1.0	1000.0
weight_acc_lim_x	double	最大x 加速度权重，Optimization weight for satisfying the maximum allowed translational acceleration	0.0	1.0	1000.0
weight_acc_lim_y	double	最大y 加速度权重，Optimization weight for satisfying the maximum allowed strafing acceleration (in use only for holonomic robots)	0.0	1.0	1000.0
weight_acc_lim_theta	double	最大角速度权重，Optimization weight for satisfying the maximum allowed angular acceleration	0.0	1.0	1000.0
weight_kinematics_nh	double	Optimization weight for satisfying the non-holonomic kinematics	0.0	1000.0	10000.0
weight_kinematics_forward_drive	double	优化过程中，迫使机器人只选择前进方向，差速轮适用，Optimization weight for forcing the robot to choose only forward directions (positive transl. velocities, only diffdrive robot)	0.0	1.0	1000.0
weight_kinematics_turning_radius	double	优化过程中，车型机器人的最小转弯半径的权重。 Optimization weight for enforcing a minimum turning radius (carlike robots)	0.0	1.0	1000.0
weight_optimaltime	double	优化过程中，基于轨迹的时间上的权重， Optimization weight for contracting the trajectory w.r.t transition time	0.0	1.0	1000.0
weight_obstacle	double	优化过程中，和障碍物最小距离的权重，Optimization weight for satisfying a minimum seperation from obstacles	0.0	50.0	1000.0
weight_inflation	double	优化过程中， 膨胀区的权重，Optimization weight for the inflation penalty (should be small)	0.0	0.1	10.0
weight_dynamic_obstacle	double	优化过程中，和动态障碍物最小距离的权重，Optimization weight for satisfying a minimum seperation from dynamic obstacles	0.0	50.0	1000.0
weight_dynamic_obstacle_inflation	double	优化过程中，和动态障碍物膨胀区的权重，Optimization weight for the inflation penalty of dynamic obstacles (should be small)	0.0	0.1	10.0
weight_viapoint	double	优化过程中，和全局路径采样点距离的权重， Optimization weight for minimizing the distance to via-points	0.0	1.0	1000.0
weight_adapt_factor	double	Some special weights (currently ‘weight_obstacle’) are repeatedly scaled by this factor in each outer TEB iteration (weight_new: weight_old * factor); Increasing weights iteratively instead of setting a huge value a-priori leads to better numerical conditions of the underlying optimization problem.	1.0	2.0	100.0
enable_multithreading	bool	允许多线程并行处理， Activate multiple threading for planning multiple trajectories in parallel	False	True	True
max_number_classes	int	允许的线程数， Specify the maximum number of allowed alternative homotopy classes (limits computational effort)	1	5	100
selection_cost_hysteresis	double	Specify how much trajectory cost must a new candidate have w.r.t. a previously selected trajectory in order to be selected (selection if new_cost < old_cost*factor)	0.0	1.0	2.0
selection_prefer_initial_plan	double	Specify a cost reduction in the interval (0,1) for the trajectory in the equivalence class of the initial plan.)	0.0	0.95	1.0
selection_obst_cost_scale	double	Extra scaling of obstacle cost terms just for selecting the ‘best’ candidate (new_obst_cost: obst_cost*factor)	0.0	100.0	1000.0
selection_viapoint_cost_scale	double	Extra scaling of via-point cost terms just for selecting the ‘best’ candidate. (new_viapt_cost: viapt_cost*factor)	0.0	1.0	100.0
selection_alternative_time_cost	bool	If true, time cost is replaced by the total transition time.	False	False	True
switching_blocking_period	double	Specify a time duration in seconds that needs to be expired before a switch to new equivalence class is allowed	0.0	0.0	60.0
roadmap_graph_no_samples	int	Specify the number of samples generated for creating the roadmap graph, if simple_exploration is turend off	1	15	100
roadmap_graph_area_width	double	Specify the width of the area in which sampled will be generated between start and goal [m ] (the height equals the start-goal distance)	0.1	5.0	20.0
roadmap_graph_area_length_scale	double	The length of the rectangular region is determined by the distance between start and goal. This parameter further scales the distance such that the geometric center remains equal!)	0.5	1.0	2.0
h_signature_prescaler	double	Scale number of obstacle value in order to allow huge number of obstacles. Do not choose it extremly low, otherwise obstacles cannot be distinguished from each other (0.2<H<=1)	0.2	1.0	1.0
h_signature_threshold	double	Two h-signuteres are assumed to be equal, if both the difference of real parts and complex parts are below the specified threshold	0.0	0.1	1.0
obstacle_heading_threshold	double	Specify the value of the normalized scalar product between obstacle heading and goal heading in order to take them (obstacles) into account for exploration)	0.0	0.45	1.0
viapoints_all_candidates	bool	If true, all trajectories of different topologies are attached to the set of via-points, otherwise only the trajectory sharing the same one as the initial/global plan is attached (no effect in test_optim_node).	False	True	True
visualize_hc_graph	bool	Visualize the graph that is created for exploring new homotopy classes	False	False	True
shrink_horizon_backup	bool	当规划器检测到系统异常，允许缩小时域规划范围。Allows the planner to shrink the horizon temporary (50%) in case of automatically detected issues.	False	True	True
oscillation_recovery	bool	尝试检测和解决振荡，Try to detect and resolve oscillations between multiple solutions in the same equivalence class (robot frequently switches between left/right/forward/backwards).	False	True	True
重要参数总结
序号	参数	功能
1	max_global_plan_lookahead_dist	考虑优化的全局计划子集的最大长度（累积欧几里得距离）
2	min_obstacle_dist	避障距离(障碍物膨胀半径)
3	dt_ref	轨迹的时间分辨率, TEB 时间最优策略, 分辨率越高更好逼近真实
————————————————
版权声明：本文为CSDN博主「Dr. Qing」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/Fourier_Legend/article/details/89398485