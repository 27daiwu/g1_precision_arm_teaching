import numpy as np
from g1_dual_arm_teaching.trajectory import SmoothstepTrajectory
from g1_dual_arm_teaching.waypoint import JointWaypoint, WaypointPlayer


def test_trajectory_includes_exact_endpoints():
    trajectory = SmoothstepTrajectory(np.zeros(14), np.full(14, 2.0), 2.0)
    np.testing.assert_array_equal(trajectory.sample(0).q, np.zeros(14))
    np.testing.assert_array_equal(trajectory.sample(2).q, np.full(14, 2.0))


def test_player_starts_at_measured_pose_and_ends_at_waypoint():
    player = WaypointPlayer([JointWaypoint("target", np.ones(14), 1.0)], np.zeros(14))
    np.testing.assert_array_equal(player.sample(0).q, np.zeros(14))
    np.testing.assert_array_equal(player.sample(player.duration).q, np.ones(14))
