# Initialization
Every subsystem is a class inheriting from `commands2.PIDSubsystem`. The `__init__()` dunder function is overridden to both contain the parent class' `__init__()` (with the PID values from `constants` passed) and the motor configuration for each motor.

## Motor configuration
Motors are stored as a class attribute of type [`TalonFX`](https://api.ctr-electronics.com/phoenix6/release/python/autoapi/phoenix6/hardware/core/core_talon_fx/index.html#phoenix6.hardware.core.core_talon_fx.CoreTalonFX) (with the constructor passed the motor's ID from `constants`). Motors are to be named based off of their location or function on the physical robot. This is the class used for the Kraken's API, and that is the only motor we should ever be using. The motors are configured using `phoenix6.configs.TalonFXConfiguration` objects, or rather their slot attributes (`slot0`, `slot1`, etc). The most important part of this process are the `slotN.k_p`, `slotN.k_i`, `slotN.k_d`, and `slotN.k_v` values, which are to contain the PID values (once again via `constants`) for the motor. The configuration represented by the `phoenix6.configs.TalonFXConfiguration` object is applied by passing it to the motor's `configurator.apply()` function.

# Making motors move
Motor movement is the fundamental principle of robots doing things. It is achieved by creating an object from the classes contained in the [`phoenix6.controls` module](https://api.ctr-electronics.com/phoenix6/release/python/autoapi/phoenix6/controls/index.html#phoenix6.controls). An instance of one of these classes is passed to the `set_control()` function of a motor. The differences between movement types is determined by the class and parameters used.

## Move at a speed
Moving the motor continually at a set speed is achieved using the `VelocityVoltage` class. This class accepts the speed (in rotations per second) as a parameter to the constructor.

## Turn to a specific angle
Moving the motor to a specific angle requires the `PositionDutyCycle` class. This class accepts the desired position of the motor (in "turns", meaning 1 is equal to a full rotation) as a parameter to the constructor.

##Turn some amount relative to the current position
This functionality is also implemented using `PositionDutyCycle`. However, the position would have to incorporate the motor's current position, which can be accessed via the object returned by motor's `get_position()` function, specifically the `_value` attribute. This, too, is in turns.

# Taking action in response to functions
Subsystem actions are stored as class functions of the subsystem class. The two main paradigms for actions are direct commands and state machines.

## Direct commands
Under the direct command paradigm, the class functions call code to move motors directly. This is preferable for simpler subsystems, particularly if there are no conditions to switch from one action to another (eg, a climber that operates purely by running motors). Please note that trying to force direct commands on a more complex subsystem can lead to severe mental anguish.

## State machines
A state machine requires that the subsystem class contain two additional attributes: an enum type (which should be named `states`) and an enum instance (which should be named `state`). The enum type is declared with one option per state (in uppercase). The enum instance is set to one of the values of the enum type. Under this paradigm, all externally referenced class functions only change the value of `state` to a different option of `states` (depending if a condition is met).
Action is taken in the `periodic()` class function's match statement comparing `state` to all of `states`' options. The branches all assign the same set of variables to different `controlls.*` objects, which are passed to motors outside of the match statement. It is to the massive benefit of code quality that the only thing determined by the state in periodic is the controls used for each motor. If you want to initiate automatic state changes, you may only do this by calling one of the externally-accessible functions in the appropriate case statement.

# Use of periodic
The `periodic()` function is used to call `super().periodic()`, house the state machine logic (mach/case and `set_control()`) and call `wpilib.SmartDashboard`'s put functions.