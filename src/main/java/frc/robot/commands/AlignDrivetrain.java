package frc.robot.commands;

import java.lang.annotation.Target;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;

public class AlignDrivetrain extends Command{
    
    Timer timer = new Timer();
    PIDController PID = new PIDController(Constants.sDrivetrain.pHubAlignP, Constants.sDrivetrain.pHubAlignI, Constants.sDrivetrain.pHubAlignD);
    double counter = 0;
    private double NextOutput;

    Translation2d TargetPosition;
    String aimType;

    public AlignDrivetrain(String aimType){
        this.aimType = aimType;
    }

    @Override
    public void initialize(){
        if(Constants.blueAlliance){
            TargetPosition = Constants.sFieldElement.kBlueHubPosition;
        } else {
            TargetPosition = Constants.sFieldElement.kRedHubPosition;
        }
        timer.restart();
    }

    @Override
    public void execute(){
        //Getting the target and current angle
        Translation2d CurrentPosition = Constants.drivetrain.getState().Pose.getTranslation();
        Translation2d TargetVector = CurrentPosition.minus(TargetPosition);
        double TargetAngle = TargetVector.getAngle().getDegrees();
        double Rotation = Constants.drivetrain.getState().Pose.getRotation().getDegrees();

        //Moving the drivetrain to the target angle
        if(TargetAngle >= 0){
            NextOutput = PID.calculate(Rotation + 180, TargetAngle);
        } else {
            NextOutput = PID.calculate(Rotation - 180, TargetAngle);
        }
        SwerveRequest drive = new SwerveRequest.FieldCentric().withRotationalRate(NextOutput);
        CommandScheduler.getInstance().schedule(Constants.drivetrain.applyRequest(() -> drive)
            .until(() -> counter > Constants.sDrivetrain.tCounterAmount ||
                         timer.get() > Constants.sDrivetrain.tMaxTimer));
        
        //Get the error of the drivetrain angle
        double error = Math.abs(TargetAngle - Rotation);

        //If the error is ls than the tolerance, increase the counter
        if(error < Constants.sDrivetrain.tDegreeTolerance){
            counter++;
        //Else if the error is 1.5 times the tolerance, increase the counter by half the amount
        } else if(error < Constants.sDrivetrain.tDegreeTolerance * 1.5){
            counter += 0.5;
        //If the error is not near enough, reset the counter
        } else {
            counter = 0;
        }
    }

    @Override
    public boolean isFinished(){
        return counter > Constants.sDrivetrain.tCounterAmount || timer.get() > Constants.sDrivetrain.tMaxTimer;
    }

    @Override
    public void end(boolean isInterrupted){
        PID.close();
        timer.stop();
    }
    
}
