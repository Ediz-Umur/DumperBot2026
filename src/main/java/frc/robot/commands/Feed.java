package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class Feed extends Command{

    Shooter shooter = Constants.shooter;
    Indexer indexer = Constants.indexer;
    boolean isIndexerStarted;

    Timer timer = new Timer();

    public Feed(){
        isIndexerStarted = false;
        addRequirements(shooter, indexer);
    }

    @Override
    public void initialize(){
        isIndexerStarted = false;
        //Setting flywheel, hood and kicker to the desired values
        shooter.FlywheelMove(Constants.sShooter.kFlywheelFeedSpeed);
        shooter.HoodMove(Constants.sShooter.kHoodFeedAngle);
        shooter.KickerMove(Constants.sShooter.kKickerSpeed);
    }

    @Override
    public void execute(){
        //Getting the real RPS and angle (THIS IS JUST FOR TESTING, DELETE WHEN DONE) //TODO
        SmartDashboard.putNumber("Flywheel RPS", shooter.GetFlywheelVelocity());
        SmartDashboard.putNumber("Hood Angle", shooter.GetHoodAngle());

        //Checks whether everything is up to speed, and starts the indexer if everything is ready
        if(
        Math.abs(shooter.GetFlywheelVelocity() - Constants.sShooter.kFlywheelFeedSpeed) < Constants.sShooter.tFlywheelTolerance &&
        Math.abs(shooter.GetHoodAngle() - Constants.sShooter.kHoodFeedAngle) < Constants.sShooter.tHoodTolerance &&
        Math.abs(shooter.GetKickerVelocity() - Constants.sShooter.kKickerSpeed) < Constants.sShooter.tKickerTolerance &&
        !isIndexerStarted
        ){
            indexer.IndexerMove(Constants.sIndexer.kIndexerSpeed);
            isIndexerStarted = true;
        }
    }

    @Override
    public void end(boolean isInterrupted){
        shooter.FlywheelStop();
        shooter.KickerStop();
        shooter.HoodMove(Constants.sShooter.kHoodClosedAngle);
        indexer.IndexerStop();
    }
}
