package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;


public class autoStop extends Command {

    Feeder FEEDER;
    Indexer INDEXER;
    Shooter SHOOTER;
    Hood HOOD;
    Roller ROLLER;
    Intake INTAKE;

    public autoStop(Feeder feeder, Indexer indexer, Shooter shooter, Hood hood, Roller roller, Intake intake) 
    {
        this.FEEDER = feeder;
        this.INDEXER = indexer;
        this.SHOOTER = shooter;
        this.HOOD = hood;
        this.ROLLER = roller;
        this.INTAKE = intake;
        addRequirements(getRequirements());
    
    }

    public void initialize()
    {   

        FEEDER.setSpeed(0)
        .andThen(INDEXER.setSpeed(0))
        .andThen(SHOOTER.stopMotors())
        .andThen(HOOD.toSetpoint(0))
        .andThen(ROLLER.setSpeed(0))
        .andThen(INTAKE.runOnce(() -> {INTAKE.toSetpoint(1);}));
        isFinished();
    }

    //sets boolean for its done to stop the comand
    public boolean isFinished()
    {
      return true;
    }
}