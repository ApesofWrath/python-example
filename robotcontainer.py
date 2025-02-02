# project imports
import constants
from subsystems.drivetrain import CommandSwerveDrivetrain as Drivetrain
from subsystems.limelight import Limelight
from subsystems.spinner import Spinner
from subsystems.turntable import Turntable

# commands imports
import commands2
import commands2.button
import commands2.cmd
from commands2.sysid import SysIdRoutine

# wpi imports
from wpilib import SmartDashboard
import wpimath
from wpimath.geometry import Rotation2d
from phoenix6 import swerve
from pathplannerlib.auto import AutoBuilder, NamedCommands

from telemetry import Telemetry

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.

    """

    def __init__(self) -> None:
        """The container for the robot. Contains subsystems, OI devices, and commands."""
        # The robot's subsystems
        self.robotDrive = constants.TunerConstants.create_drivetrain()
        self.turntable = Turntable()
        self.spinner = Spinner()
        self.limelight = Limelight(self.robotDrive)

        # The robot's commands
        NamedCommands.registerCommand("spinner.off", self.spinner.off())
        NamedCommands.registerCommand("spinner.on", self.spinner.on())

        # The driver's controller
        self.driverController = commands2.button.CommandXboxController(constants.Global.kDriverControllerPort)

		# Setting up bindings for necessary control of the swerve drive platform
        self.drive = (
            swerve.requests.FieldCentric()
            .with_deadband(constants.Global.max_speed * 0.1)
            .with_rotational_deadband(
                constants.Global.max_angular_rate * 0.1
            )  # Add a 10% deadband
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )

        # Configure the button bindings
        self.configureButtonBindings()

        # Build an auto chooser. This will use Commands.none() as the default option.
        self.autoChooser = AutoBuilder.buildAutoChooser()
        SmartDashboard.putData("Auto Chooser", self.autoChooser)

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created via the button
        factories on commands2.button.CommandGenericHID or one of its
        subclasses (commands2.button.CommandJoystick or command2.button.CommandXboxController).
        """
        # Drive
        self.robotDrive.setDefaultCommand(
            # Drivetrain will execute this command periodically
            self.robotDrive.apply_request(
                lambda: (
                    self.drive.with_velocity_x(
                        -self.driverController.getLeftY()
                        * constants.Global.max_speed
                        * max((self.driverController.leftBumper() | self.driverController.rightBumper()).negate().getAsBoolean(),constants.Global.break_speed_mul)
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(0
                        #-self.driverController.getLeftX()
                        #* constants.Global.max_speed
                        #* max((self.driverController.leftBumper() | self.driverController.rightBumper()).negate().getAsBoolean(),constants.Global.break_speed_mul)
                    )  # Drive left with negative X (left)
                    .with_rotational_rate(
                        self.driverController.getRightX()
                        * constants.Global.max_angular_rate
                    )  # Drive counterclockwise with X (right)
                )
            )
        )

        (self.driverController.leftTrigger() | self.driverController.rightTrigger()).whileTrue(self.robotDrive.apply_request(lambda: swerve.requests.SwerveDriveBrake()))

        # Run SysId routines when holding back and face buttons.
        # Note that each routine should be run exactly once in a single log.
        (self.driverController.back() & self.driverController.a()).whileTrue(
            self.robotDrive.sys_id_dynamic(SysIdRoutine.Direction.kForward)
        )
        (self.driverController.back() & self.driverController.b()).whileTrue(
            self.robotDrive.sys_id_dynamic(SysIdRoutine.Direction.kReverse)
        )
        (self.driverController.back() & self.driverController.x()).whileTrue(
            self.robotDrive.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
        )
        (self.driverController.back() & self.driverController.y()).whileTrue(
            self.robotDrive.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)
        )

        # reset the field-centric heading on start press
        self.driverController.start().onTrue(
            self.limelight.runOnce(lambda: self.limelight.gyro.set_yaw(0))
        )

        self.logger = Telemetry(constants.Global.max_speed)
        self.robotDrive.register_telemetry(
            lambda state: self.logger.telemeterize(state)
        )

		# Subsystems
        self.driverController.a().onTrue(self.turntable.freespin(2)).onFalse(self.turntable.freespin(0))
        self.driverController.b().onTrue(self.turntable.freespin(-2)).onFalse(self.turntable.freespin(0))
        self.driverController.x().onTrue(self.turntable.turndeg(90))
        self.driverController.y().onTrue(self.turntable.turnto(0))
        self.driverController.povUp().onTrue(self.spinner.on())
        self.driverController.povDown().onTrue(self.spinner.off())

    def getAutonomousCommand(self) -> commands2.Command:
        """
        Use this to pass the autonomous command to the main :class:`.Robot` class.

        :returns: the command to run in autonomous
        """
        return self.autoChooser.getSelected()