package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class EmptyHopper extends Command{
    
    Intake intake = Constants.intake;
    Indexer indexer = Constants.indexer;

    boolean isIntakeOpen = false;

    public EmptyHopper(){
        addRequirements(intake, indexer);
    }

    @Override
    public void initialize(){
        intake.GoToPosition(Constants.sIntake.kPivotOpenPosition);
    }

    @Override
    public void execute(){
        //If the intake is open, start moving the roller and indexer in the opposite direction
        if(Math.abs(intake.GetPivotPosition() - Constants.sIntake.kPivotOpenPosition) < Constants.sIntake.tPivotTolerance && !isIntakeOpen){
            indexer.IndexerMove(-Constants.sIndexer.kIndexerSpeed);
            intake.RollerMove(-Constants.sIntake.kRollerSpeed);
            isIntakeOpen = true;
        }
    }

    @Override
    public void end(boolean isInterrupted){
        indexer.IndexerStop();
        intake.RollerStop();
    }
}
