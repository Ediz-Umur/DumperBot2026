package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeMove extends Command{
    Intake intake = Constants.intake;
    double TargetPosition;

    boolean isForced = false;
    String forcedPosition;

    public IntakeMove(){
        addRequirements(intake);
        isForced = true;
    }

    public IntakeMove(String forcedPosition){
        this.forcedPosition = forcedPosition;
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        if(isForced){ //If the position is forced
            if(forcedPosition.equals("open")){
                //Open the intake
                TargetPosition = Constants.sIntake.kPivotOpenPosition;
                intake.RollerMove(Constants.sIntake.kRollerSpeed);
            } else if(forcedPosition.equals("close")){
                //Close the intake
                TargetPosition = Constants.sIntake.kPivotClosedPosition;
                intake.RollerStop();
            }
        } else { //If the position is not forced
            //If the intake is withing the tolerance for the closed position, set target to open position and start the roller 
            if(Math.abs(intake.GetPivotPosition() - Constants.sIntake.kPivotClosedPosition) < Constants.sIntake.tPivotTolerance){
            TargetPosition = Constants.sIntake.kPivotOpenPosition;
            intake.RollerMove(Constants.sIntake.kRollerSpeed);
            //Else, set the target to the closed position and stop the roller
            } else {
            TargetPosition = Constants.sIntake.kPivotClosedPosition;
            intake.RollerStop();
            }
        }

        //Send the intake to the target position
        intake.GoToPosition(TargetPosition);
    }

    @Override
    public boolean isFinished(){
        //End command when the target position is within tolerance
        return (Math.abs(intake.GetPivotPosition() - TargetPosition) < Constants.sIntake.tPivotTolerance);
    }
}
