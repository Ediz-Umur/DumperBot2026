// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Shoot;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;
    private Field2d field = new Field2d();

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        //Resets the intake encoder
        Constants.intake.PivotEncoderReset();

        VisionManager.OdometryInitialize();

        m_robotContainer = new RobotContainer();

        SmartDashboard.putData("Field", field);      
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    }

    @Override
    public void robotPeriodic() {
        double TargetAngle = Shoot.DistanceToAngle(Shoot.getDistance());
        TargetAngle = (90 - TargetAngle) - Constants.sShooter.kHoodMinAngle;
        SmartDashboard.putBoolean("Can Shoot", TargetAngle > 0 && TargetAngle < Constants.sShooter.kHoodMaxAngle - Constants.sShooter.kHoodMinAngle);

        field.setRobotPose(Constants.drivetrain.getState().Pose);
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 

        VisionManager.OdometryCycle();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
