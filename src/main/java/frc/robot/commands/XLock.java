package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants;
import frc.robot.RobotContainer;

public class XLock extends Command{
    @Override
    public void execute(){
        CommandScheduler.getInstance().schedule(new InstantCommand(() -> Constants.drivetrain.setControl(new SwerveRequest.SwerveDriveBrake())));
    }

    @Override
    public boolean isFinished(){
        double JoystickMoveAmount = Math.sqrt(Math.pow(RobotContainer.joystick.getLeftX(),2) + Math.pow(RobotContainer.joystick.getLeftX(), 2));
        return JoystickMoveAmount > Constants.sDrivetrain.kXLockDisableLimit;
    }

    @Override
    public void end(boolean Interrupted){
        CommandScheduler.getInstance().cancel(new InstantCommand(() -> Constants.drivetrain.setControl(new SwerveRequest.SwerveDriveBrake())));
    }
}
