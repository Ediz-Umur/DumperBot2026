// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AlignDrivetrain;
import frc.robot.commands.EmptyHopper;
import frc.robot.commands.Feed;
import frc.robot.commands.IntakeMove;
import frc.robot.commands.IntakeOscillate;
import frc.robot.commands.Shoot;
import frc.robot.commands.XLock;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    boolean isSysID = false;

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public static final CommandPS5Controller joystick = new CommandPS5Controller(0);

    public final CommandSwerveDrivetrain drivetrain = Constants.drivetrain;

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        //Turns on the SysID routines for the drivetrain (NOTE: THIS IS JUST FOR TESTING, DELETE AFTER ITS DONE) //TODO
        if(isSysID){
            joystick.circle().onTrue(Commands.runOnce(SignalLogger::stop));
            joystick.cross().onTrue(Commands.runOnce(SignalLogger::start));

            joystick.L1().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
            joystick.R1().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
            joystick.L2().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
            joystick.R2().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        } else {
            joystick.L1().whileTrue(new IntakeOscillate());

            joystick.L2().onTrue(new IntakeMove());

            joystick.R1().toggleOnTrue(new EmptyHopper());
            
            joystick.R2().onTrue(Commands.parallel(
                Commands.sequence(Commands.waitSeconds(1), new IntakeOscillate()), //Start the oscillation after the selected time 
                    Commands.parallel( //During that,
                            new Shoot(Constants.sShooter.kTeleopMaxShootTime), //Start the shoot command and
                            new AlignDrivetrain("hub").andThen(new XLock()) //Align the robot to the hub and XLock 
                            .until(() -> //Until 
                            //The joystick has moved enough away from the center position
                            Math.sqrt( Math.pow(joystick.getLeftX(), 2) + Math.pow(joystick.getLeftY(), 2)) > Constants.sDrivetrain.kXLockDisableLimit)
                        )
                    )
                );

            joystick.square().onTrue(new Feed());

            //Stopping the roller
            joystick.triangle().onTrue(new InstantCommand(() -> Constants.intake.RollerStop()));

            joystick.cross().onTrue(new InstantCommand(() -> Constants.intake.RollerMove(Constants.sIntake.kRollerSpeed)));

            joystick.circle().onTrue(new XLock());

            //Resetting the Gyro
            joystick.options().onTrue(new InstantCommand(() -> Constants.drivetrain.resetRotation(new Rotation2d(0))));

        }

        // Reset the field-centric heading on left bumper press.
        joystick.options().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
