package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;


public class AdjustShooter extends Command {

    Hood HOOD;
    Shooter SHOOTER;
    Limelight LIMELIGHT;
    
    InterpolatingDoubleTreeMap shooterRPM;
    InterpolatingDoubleTreeMap hoodAngle;


    public AdjustShooter(Hood hood, Shooter shooter, Limelight limelight) 
    {
        this.HOOD = hood;
        this.SHOOTER = shooter;
        this.LIMELIGHT = limelight;

        addRequirements(hood);
        addRequirements(shooter);
        addRequirements(limelight);
        
        shooterRPM = new InterpolatingDoubleTreeMap();

        //Key: distance 
        //Value: velocity of shooter

        //Put more data points
        shooterRPM.put(0.0,0.0);

        hoodAngle = new InterpolatingDoubleTreeMap();

        //Key: distance
        //Value: hood angle

        //Put more data points
        hoodAngle.put(0.0,0.0);
        

    }

    public void initialize()
    {          
    }

    public void execute()
    {
        double distance;
        RawFiducial[] currentFiducials = LIMELIGHT.getFiducialData(); //this is the raw data from the limelight
        /*for (RawFiducial fiducial : currentFiducials) {//moves raw fiducial ids to target ids

        }*/
        
        //will keep running until finished

        //runs "isFinished" function to say how it is done
        isFinished();
    }


    //sets boolean for its done to stop the comand
    public boolean isFinished()
    {
      return true;
    }
}