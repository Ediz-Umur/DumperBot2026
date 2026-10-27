package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class RollerToggle extends Command{
    Intake intake = Constants.intake;
    double targetSpeed = 0.0;
    boolean isRollerStarted = false;

    public RollerToggle(){
        if(Math.abs(intake.GetRollerVelocity() - Constants.sIntake.kRollerSpeed) < Constants.sIntake.tRollerTolerance){
            targetSpeed = -Constants.sIntake.kRollerSpeed;
        } else {
            targetSpeed = Constants.sIntake.kRollerSpeed;
        }
        addRequirements(intake);
    }

    public RollerToggle(String forcedDirection){
        if(forcedDirection.equals("forwards")){
            targetSpeed = Constants.sIntake.kRollerSpeed;
        } else if(forcedDirection.equals("backwards")){
            targetSpeed = -Constants.sIntake.kRollerSpeed;
        } else if(forcedDirection.equals("zero")){
            targetSpeed = 0;
        }
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        intake.RollerMove(targetSpeed);
    }
}
