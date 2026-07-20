# GP11 MoveIt Real-Robot Integration

This package contains the non-Gazebo GP11 planning stack used with the B29
Planner Adapter. It provides the anchored planning model, MoveIt/OMPL
configuration, real-robot anchor bridge, execution monitoring, and the
`ReachPoint` Action definition.

The expected command path is:

```text
MoveIt move_group -> FollowJointTrajectory Action -> b29_planner_adapter
-> B29 SMC PlannerControl -> CommandDispatcher -> hardware interface
```

`gp11_moveit_real.launch` is intentionally independent from
`b29_control/start.launch`. It must not be used to start B29 hardware or the
Planner Adapter, and it does not start Gazebo.

For safety, `enable_real_execution` defaults to `false`. The default mode is
planning and observation only. Enable trajectory execution explicitly only
after the B29 controller, Planner Adapter, joint mapping, anchor side, and
physical safety conditions have been verified.

The package keeps the real-robot anchor provider only. Gazebo worlds, support
plugins, cable simulation, and simulation launch files are intentionally not
included in this migration.
