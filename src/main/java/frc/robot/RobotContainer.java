// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AlignDrivetrain;
import frc.robot.commands.DynamicFlywheelWarmup;
import frc.robot.commands.EmptyHopper;
import frc.robot.commands.Feed;
import frc.robot.commands.IntakeMove;
import frc.robot.commands.IntakeOscillate;
import frc.robot.commands.RollerToggle;
import frc.robot.commands.Shoot;
import frc.robot.commands.XLock;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {

    boolean isSysID = false;
    boolean isCompetition = false;

    public SendableChooser<Command> autoChooser;
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.sDrivetrain.kMaxSpeed * 0.1).withRotationalDeadband(Constants.sDrivetrain.kMaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(Constants.sDrivetrain.kMaxSpeed);

    public static final CommandPS5Controller joystick = new CommandPS5Controller(0);

    public final CommandSwerveDrivetrain drivetrain = Constants.drivetrain;

    public RobotContainer() {
        Constants.drivetrain.configureAutoBuilder();

        NamedCommands.registerCommand("Shoot", Commands.race(
                     /* The oscillating command  */     Commands.sequence(
                    /* Wait for the set wait time */        Commands.waitSeconds(Constants.sIntake.kOscillateWaitTime), 
                    /* Then start the oscillation */        new IntakeOscillate()),
                                                        new Shoot(Constants.sShooter.kAutoMaxShootTime)));   

        NamedCommands.registerCommand("IntakeMove", new IntakeMove("open"));
        NamedCommands.registerCommand("IntakeOscillate", new IntakeOscillate());
        NamedCommands.registerCommand("RollerStop", new InstantCommand(() -> Constants.intake.RollerStop()));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * Constants.sDrivetrain.kMaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * Constants.sDrivetrain.kMaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * Constants.sDrivetrain.kMaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        //Setting the warmup command as the shooters' default command
        //Constants.shooter.setDefaultCommand(new DynamicFlywheelWarmup());

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

            joystick.square().toggleOnTrue(new EmptyHopper());
            
            joystick.R2().toggleOnTrue( 
                        Commands.parallel(
                            new Shoot(Constants.sShooter.kTeleopMaxShootTime), //Start the shoot command and
                            new AlignDrivetrain("hub").andThen(new XLock()) //Align the robot to the hub and XLock 
                            .until(() -> //Until 
                            //The joystick has moved enough away from the center position
                            Math.sqrt( Math.pow(joystick.getLeftX(), 2) + Math.pow(joystick.getLeftY(), 2)) > Constants.sDrivetrain.kXLockDisableLimit)
                    )
                );

            joystick.R1().toggleOnTrue(new Feed());

            //Stopping the roller
            joystick.triangle().onTrue(new InstantCommand(() -> Constants.intake.RollerStop()));

            joystick.cross().onTrue(new InstantCommand(() -> Constants.intake.RollerMove(Constants.sIntake.kRollerSpeed)));

            joystick.circle().onTrue(new XLock());

            //Resetting the Gyro
            joystick.options().onTrue(new InstantCommand(() -> Constants.drivetrain.resetRotation(new Rotation2d(0))));

            //Test Inputs
            joystick.povUp().onTrue(new IntakeMove("zero"));

            joystick.povDown().onTrue(new AlignDrivetrain("hub"));

            joystick.povLeft().onTrue(new IntakeMove("open"));
        }

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
