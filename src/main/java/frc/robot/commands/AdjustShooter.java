package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.*;

import java.util.ArrayList;
import java.util.List;

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

        //Furthest distance
        //shooterRPM.put(,37.0)

        hoodAngle = new InterpolatingDoubleTreeMap();

        //Key: distance
        //Value: hood angle

        //Put more data points
        hoodAngle.put(0.0,0.0);
        
        /*notes form 2/17
        From Outpost:
           Hood: 8.3916015625
           Shooter: 80%
        */
    }

    public void initialize()
    {          
    }

    public void execute()
    {
        double distance;
        int crtTargetID = 0;

        
        RawFiducial[] currentFiducials = LIMELIGHT.getFiducialData(); //this is the raw data from the limelight
        List<Integer> alCurrentTargetsIDs = new ArrayList<>(); //sets up list of IDs for the field

        for (RawFiducial fiducial : currentFiducials) {//moves raw fiducial ids to target ids

            alCurrentTargetsIDs.add(fiducial.id);
        }

        //groups tags & picks a tag
        if (alCurrentTargetsIDs.contains(5) && alCurrentTargetsIDs.contains(8) && alCurrentTargetsIDs.contains(9) && alCurrentTargetsIDs.contains(10)){
            crtTargetID = 9;
        }
        else if (alCurrentTargetsIDs.contains(2) && alCurrentTargetsIDs.contains(9) && alCurrentTargetsIDs.contains(10) && alCurrentTargetsIDs.contains(11)){
            crtTargetID = 11;
        }
        else if (alCurrentTargetsIDs.contains(8) && alCurrentTargetsIDs.contains(9) && alCurrentTargetsIDs.contains(10)){
            crtTargetID = 9;
        }
        else if (alCurrentTargetsIDs.contains(10) && alCurrentTargetsIDs.contains(11) && alCurrentTargetsIDs.contains(2)){
            crtTargetID = 11;
        }
        else if (alCurrentTargetsIDs.contains(9) && alCurrentTargetsIDs.contains(10) && alCurrentTargetsIDs.contains(11)){
            crtTargetID = 10;
        }
        else if (alCurrentTargetsIDs.contains(9) && alCurrentTargetsIDs.contains(10)){
            crtTargetID = 10;
        }
        else if (alCurrentTargetsIDs.contains(11) && alCurrentTargetsIDs.contains(2)){
            crtTargetID = 11;
        }
        else if (alCurrentTargetsIDs.contains(10)){
            crtTargetID = 10;
        }
        
        RawFiducial[] crtFiducials = LIMELIGHT.getFiducialData();

      //moves raw fiducial to current target ids, takes only tx of the current target id    
            for (RawFiducial fiducial : crtFiducials) {

                if (fiducial.id == crtTargetID){
                    //distToRobot = fiducial.distToRobot; // X offset (no crosshair)
                }
            }
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