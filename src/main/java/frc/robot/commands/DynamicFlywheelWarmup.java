package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class DynamicFlywheelWarmup extends Command{

    Shooter shooter = Constants.shooter;

    public DynamicFlywheelWarmup(){
        addRequirements(shooter);
    }

    @Override
    public void initialize(){
        /*double distance = Shoot.getDistance();
        if (distance <= Constants.sShooter.kWarmupMaxDistance){
            double RPS = Shoot.DistanceToRPS(distance, Shoot.DistanceToAngle(distance)) * Constants.sShooter.kWarmupScale;
            shooter.FlywheelMove(RPS);
        } else {
            shooter.FlywheelStop();
        }*/
            shooter.FlywheelMove(13);
    }
}
