package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeOscillate extends Command{
    Intake intake = Constants.intake;
    double TargetPosition;

    public IntakeOscillate(){
        addRequirements(intake);
    }

    @Override
    public void execute(){
        //If the intake is withing the tolerance for the closed position, set target to the oscillate position and start the roller 
        if(Math.abs(intake.GetPivotPosition() - Constants.sIntake.kPivotClosedPosition) < Constants.sIntake.tPivotTolerance){
            TargetPosition = Constants.sIntake.kPivotOpenPosition;
            intake.RollerMove(Constants.sIntake.kRollerSpeed);
        //Else, set the target to the closed position and stop the roller
        } else if(Math.abs(intake.GetPivotPosition() - Constants.sIntake.kPivotOpenPosition) < Constants.sIntake.tPivotTolerance){
            TargetPosition = Constants.sIntake.kPivotClosedPosition;
            intake.RollerStop();
        }

        //Send the intake to the target position
        intake.GoToPosition(TargetPosition);
    }
}
