def create_pose(pose, base_pose):
    newpose = deepcopy(base_pose)
    newpose.position.x = pose[0]
    newpose.position.y = pose[1]
    newpose.position.z = pose[2]
    newpose.orientation.x = 0.0
    newpose.orientation.y = 1.0
    newpose.orientation.z = 0.0
    newpose.orientation.w = 0.0
    return newpose

waypoints = []
group = MoveGroupCommander("right_arm")
wpose = group.get_current_pose().pose
for point in points:
    waypoints.append(create_pose(point, wpose))

(plan, fraction) = group.compute_cartesian_path(
    waypoints,
    0.003,
    0.0
)
group.execute(plan)
