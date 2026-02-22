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

    
    InterpolatingDoubleTreeMap shooterRPS;
    InterpolatingDoubleTreeMap hoodAngle;


    public AdjustShooter(Hood hood, Shooter shooter, Limelight limelight) 
    {
        this.HOOD = hood;
        this.SHOOTER = shooter;
        this.LIMELIGHT = limelight;

        addRequirements(hood);
        addRequirements(shooter);
        addRequirements(limelight);
        
        shooterRPS = new InterpolatingDoubleTreeMap();
        //Key: distance 
        //Value: velocity of shooter
        shooterRPS.put(0.8732327907316006,24.0);        
        shooterRPS.put(1.1529219342235464,26.0);
        shooterRPS.put(1.8059915426872029,32.0);
        shooterRPS.put(2.5236079021737163,35.0);
        shooterRPS.put(4.226945331446267,40.0);
        

        hoodAngle = new InterpolatingDoubleTreeMap();
        //Key: distance
        //Value: hood angle
        hoodAngle.put(0.8732327907316006,-0.06005859375);
        hoodAngle.put(1.1529219342235464,2.23974609375);
        hoodAngle.put(1.8059915426872029,5.30224609375);
        hoodAngle.put(2.5236079021737163,6.87060546875);
        hoodAngle.put(4.226945331446267,3.01318359375);
    }

    public void initialize()
    {          

    }

    public void execute()
    {
        SmartDashboard.putNumber("Predicted Hood angle",hoodAngle.get(LIMELIGHT.getDistToNearestTag()));
        SmartDashboard.putNumber("Predicted Shooter RPS",shooterRPS.get(LIMELIGHT.getDistToNearestTag()));
   
        /*//will keep running until finished
        HOOD.customPosition(hoodAngle.get(LIMELIGHT.getDistToNearestTag()));
        SHOOTER.PIDrunMotors(shooterRPS.get(LIMELIGHT.getDistToNearestTag()));*/
    }
}