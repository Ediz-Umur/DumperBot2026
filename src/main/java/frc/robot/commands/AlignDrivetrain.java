package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;

public class AlignDrivetrain extends Command{
    
    Timer timer = new Timer();
    PIDController PID = new PIDController(Constants.sDrivetrain.pHubAlignP, Constants.sDrivetrain.pHubAlignI, Constants.sDrivetrain.pHubAlignD);
    double counter = 0;

    Translation2d TargetPosition;
    String aimType;

    public AlignDrivetrain(String aimType){
        this.aimType = aimType;
    }

    @Override
    public void initialize(){
        if(aimType.equals("hub")){
            if(Constants.blueAlliance){
            TargetPosition = Constants.sFieldElement.kBlueHubPosition;
            } else {
            TargetPosition = Constants.sFieldElement.kRedHubPosition;
            }
        } else if (aimType.equals("feed")){
            TargetPosition = Constants.sFieldElement.kFeedTarget;

            //NOTE! THIS IS IN FIELD COORDINATES, SO ORIGIN IS BOTTOM(NEXT TO THE HP) LEFT WHEN THE BLUE ALLIANCE IS ON THE LEFT SIDE.
            //Flip the target to the upper side if the robot is "higher" than the hub
            if(Constants.drivetrain.getState().Pose.getMeasureY().compareTo(Constants.sFieldElement.kBlueHubPosition.getMeasureY()) < 0){
                TargetPosition.plus(new Translation2d(0,Constants.sFieldElement.kFieldSize.getY() - 2 * TargetPosition.getY()));
            }
            //Flip the target over to the red side if were the red alliance
            if(!Constants.blueAlliance){
                TargetPosition.plus(new Translation2d(Constants.sFieldElement.kFieldSize.getX() - 2 * TargetPosition.getX(), 0));
            }
        }

        timer.restart();
    }

    @Override
    public void execute(){
        System.out.println("aligning");
        System.out.println(counter);
        //Getting the target and current angle
        Translation2d CurrentPosition = Constants.drivetrain.getState().Pose.getTranslation();
        Translation2d TargetVector = CurrentPosition.minus(TargetPosition);
        double TargetAngle = TargetVector.getAngle().getDegrees();
        double Rotation = Constants.drivetrain.getState().Pose.getRotation().getDegrees();

        //Moving the drivetrain to the target angle
        double NextOutput = PID.calculate(Rotation + 180, TargetAngle);
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
