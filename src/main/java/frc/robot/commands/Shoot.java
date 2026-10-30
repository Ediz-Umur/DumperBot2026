package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command{
    
    public Shooter shooter = Constants.shooter;
    public Indexer indexer = Constants.indexer;

    static Translation2d TargetTranslation;
    double TargetRPS;
    double TargetAngle;
    double Distance;
    double multiplier = Constants.sShooter.kFlywheelMult;

    boolean isIndexerStarted = false;

    Timer timer = new Timer();
    double endTime;

    public Shoot(double endTime){
        this.endTime = endTime;
        addRequirements(shooter, indexer);
    }
    
    public double DistanceToRPS(double distance, double angle){
        //Meters --> RotationPerSecond

        double height = Constants.sFieldElement.kHubHeight - Constants.sShooter.kFlywheelHeight;
        
        double Velocity = Math.sqrt((-9.8*Math.pow(distance, 2)*(1 + Math.pow(Math.tan(Math.toRadians(angle)), 2))) / ((2 * height) - 2 * distance * Math.tan(Math.toRadians(angle))));
        SmartDashboard.putNumber("Raw Target Velocity", Velocity);

        return (2 * Velocity * multiplier) / (0.1237 * Math.PI);
        //return((Units.radiansPerSecondToRotationsPerMinute(Velocity / Constants.sShooter.kFlywheelRadius)) * multiplier) / 60;
    }

    public static double DistanceToAngle(double distance){
        //Meters --> Degrees
        
        double S = Math.toRadians(Constants.sShooter.kIdealAngle);
        double H = Constants.sFieldElement.kHubHeight - Constants.sShooter.kFlywheelHeight;

        double Angle = Math.toDegrees(Math.atan((Math.tan(S) * distance - (2 * H)) / -distance));
        return(Angle);
    }

    public static double getDistance(){
        //Meters
        //Get the Hub Translation
        if (Constants.blueAlliance){
            TargetTranslation = Constants.sFieldElement.kBlueHubPosition;
        } else {
            TargetTranslation = Constants.sFieldElement.kRedHubPosition;
        }

        //Get the robots translation
        Translation2d ShooterTranslation = Constants.drivetrain.getState().Pose.getTranslation().minus(Constants.sShooter.kFlywheelOffsetFromCenter);

        //Return the distance between the translations
        return(ShooterTranslation.getDistance(TargetTranslation));
    }

    @Override
    public void initialize(){
        isIndexerStarted = false;

        //Getting the target RPS and angle data
        Distance = getDistance();
        TargetAngle = DistanceToAngle(Distance);
        TargetRPS = DistanceToRPS(Distance, TargetAngle);

        TargetAngle = (90 - TargetAngle) - Constants.sShooter.kHoodMinAngle;

        //Setting flywheel and hood to the desired values
        shooter.FlywheelMove(TargetRPS);
        shooter.HoodMove(TargetAngle);

        //Putting angle and speed onto the dashboard (ONLY FOR DEBUGGING, REMOVE AFTER TESTS) //TODO
        SmartDashboard.putNumber("Calculated Distance", Distance);
        SmartDashboard.putNumber("Target RPS", TargetRPS);
        SmartDashboard.putNumber("Target Angle", TargetAngle);
    }

    @Override
    public void execute(){
        //Getting the real RPS and angle (THIS IS JUST FOR TESTING, DELETE WHEN DONE) //TODO
        SmartDashboard.putNumber("Flywheel RPS", shooter.GetFlywheelVelocity());
        SmartDashboard.putNumber("Hood Angle", shooter.GetHoodAngle());

        //Checks whether everything is up to speed, and starts the indexer if everything is ready
        if(
        Math.abs(shooter.GetFlywheelVelocity() - TargetRPS) < Constants.sShooter.tFlywheelTolerance &&
        Math.abs(shooter.GetHoodAngle() - TargetAngle) < Constants.sShooter.tHoodTolerance &&
        !isIndexerStarted
        ){
            indexer.IndexerMove(Constants.sIndexer.kIndexerSpeed);
            shooter.KickerMove(Constants.sShooter.kKickerSpeed);
            isIndexerStarted = true;
        }
    }

    @Override
    public boolean isFinished(){
        if(!isIndexerStarted && !timer.isRunning() && !shooter.BeamBreakDetected()){
            timer.start();
        } else if(shooter.BeamBreakDetected()){
            timer.stop();
            timer.reset();
        }
        if(timer.hasElapsed(endTime)){
            SmartDashboard.putString("End Reason", "Timer Ran Out");
            timer.stop();
            timer.reset();
            return true;
        }

        if(TargetRPS > Constants.sShooter.kFlywheelMaxSpeed){
            SmartDashboard.putString("End Reason", "Speed Too High");
            return true;
        }

        if(TargetAngle > Constants.sShooter.kHoodMaxAngle - Constants.sShooter.kHoodMinAngle){
            SmartDashboard.putString("End Reason", "Angle Too High");
            return true;
        } else if(TargetAngle < 0){
            SmartDashboard.putString("End Reason", "Angle Too Low");
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean isInterrupted){
        timer.stop();
        timer.reset();
        System.out.println("Command ended");
        shooter.FlywheelStop();
        shooter.KickerStop();
        shooter.HoodMove(Constants.sShooter.kHoodClosedAngle);
        indexer.IndexerStop();
    }
}
