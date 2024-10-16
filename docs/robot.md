# MyRobot
The `MyRobot` class should inherit `commands2.TimedCommandRobot` and do the bare minimum to set `self.container` to an instance of `RobotContainer` (in the `robotInit` function) and schedule the auton command. This is done by declaring an attribute of type [`Optional[commands2.Command]`](https://robotpy.readthedocs.io/projects/commands-v2/en/latest/commands2/Command.html#commands2.Command) in `robotInit`. Then, in `autonomousInit()`, setting it to the result of `RobotContainer`'s `getAutonomousCommand()` function, and finally (after ensuring that it exists) running its `schedule()` function.

# RobotContainer
`RobotContainer` includes the majority of the robot code. This includes the functions `__init__()`, `configureButtonBindings()`, and `getAutonomousCommand()`.

`__init__()` declares the subsystems as instance of the classes in `subsystems/`, named identically save for capitalization conventions. Also upon initialization, a [`driverController` (and `operatorController`)](https://robotpy.readthedocs.io/projects/commands-v2/en/latest/commands2.button/CommandXboxController.html#commands2.button.CommandXboxController) are declared using port numbers taken from the constants file. Finally, call the drivetrain's `setDefaultCommand` to a command which runs the `drive()` function using the appropriate arguments.

`configureButtonBindings()` is called by `__init__()` and calls the controller's triggers to bind buttons. [Command decorators](https://robotpy.readthedocs.io/projects/commands-v2/en/latest/commands2.cmd/functions.html) may be chained to an arbitrary length to achieve the desired actions, but at some point a state machine may be necessary.

`getAutonomousCommand()` returns a command which contains the behavior for auton phase. This is traditionally done using a ShuffleBoard chooser.