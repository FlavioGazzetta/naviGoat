"""Laptop-only goal preview with dummy tracks and a stationary robot/camera.

Run python3 -B goal_demo.py. No ROS, action server or physical movement.
"""

from goal_generator import GoalGenerator, camera_point_on_map
from state_machine import FollowingState, FollowingStateMachine, PerceptionFrame, PersonObservation


def main():
    machine = FollowingStateMachine()
    planner = GoalGenerator()
    machine.enable()
    robot = (0.0, 0.0)
    camera_mount = (0.2, 0.0, 0.0, 0.0, 0.0, 1.0)
    # ID 7 starts nearer. ID 8 later approaches, without replacing the target.
    samples = [((7, 2, 0), (8, 3, 0))] * 3
    samples += [((7, 2.4, 0.3), (8, 1.2, 0)), (), ()]
    samples += [((7, 2.4, 0.3), (8, 1.2, 0)), ((7, 0.8, 0),)]
    samples += [()] * 8 + [((7, 2, 0),)]
    print("Dummy data; map goals only. No ROS or hardware. Camera is 0.2 m ahead of base.")
    print(" t(s)  State               ID  Navigation decision / goal (x, y, yaw)")
    for index, sample in enumerate(samples):
        now = index * 0.25
        people = tuple(PersonObservation(*entry) for entry in sample)
        output = machine.update(now, PerceptionFrame(now, people))
        if output.state == FollowingState.FOLLOWING:
            selected = next(p for p in people if p.track_id == output.target_id)
            target = camera_point_on_map(selected.forward_m, selected.lateral_m, camera_mount)
            action, goal = planner.update(robot, target, now)
            detail = action if goal is None else f"{action}: ({goal.x:.2f}, {goal.y:.2f}, {goal.yaw:.2f})"
        else:
            planner.reset()
            detail = "cancel / no new goal"
        print(f"{now:5.2f}  {output.state.value:<18} {str(output.target_id):>3}  {detail} {output.event or ''}")


if __name__ == "__main__":
    main()
