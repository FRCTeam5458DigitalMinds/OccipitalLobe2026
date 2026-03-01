package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.RobotContainer;

public class Limelight extends SubsystemBase{
   //SwerveDrive class
   CommandSwerveDrivetrain drivetrain;
   SwerveRequest.RobotCentric robotDrive;
   
   //name of limelight IDs (don't forget)
   String dmllName = Constants.LimelightConstants.ll_Name;
   
   //info from tag
   int id;

//pose thingys
   private LimelightHelpers.PoseEstimate limelightMeasurement = new LimelightHelpers.PoseEstimate();

    private final SendableChooser<Integer> limelightPipelineChooser;

   public Limelight(){
        limelightPipelineChooser = new SendableChooser<>();

        limelightPipelineChooser.setDefaultOption("Day", 0);
        limelightPipelineChooser.addOption("Night", 1);
        limelightPipelineChooser.addOption("Comp", 2);

        SmartDashboard.putData("Pipeline Chooser", limelightPipelineChooser);
        LimelightHelpers.setPipelineIndex(Constants.LimelightConstants.ll_Name,limelightPipelineChooser.getSelected());

   }
   //Change smartdashboard to elastic later
    // Basic targeting data
   public Double getTX(){
      //Take tx of current tag
      //double tx = LimelightHelpers.getTX(dmllName);
      //SmartDashboard.putNumber("TX", tx);
      return LimelightHelpers.getTX(dmllName); // Horizontal offset from crosshair to target in degrees

   }

   //just up and down off of the crosshair
   public Double getTY(){

      double ty = LimelightHelpers.getTY(dmllName);
      SmartDashboard.putNumber("TY", ty);
      return ty; // Vertical offset from crosshair to target in degrees
   }

   //Output for TA (Target Area)
   public Double getTA(){
      return LimelightHelpers.getTA(dmllName);
   }

   //Target stuff 
   public boolean checkForTarget(){
      return LimelightHelpers.getTV(dmllName); // Do you have a valid target?
   }

   //current id for the Dashboard
   public Double getTargetID(){

      Double dTarget = Double.parseDouble("0");

      //return the ID
      dTarget = LimelightHelpers.getFiducialID(dmllName);
      SmartDashboard.putNumber("LL-ID", dTarget);
      return dTarget;
   }
   public int priorityTag(){
        RawFiducial[] currentFiducials = getFiducialData(); //this is the raw data from the limelight
        List<Integer> alCurrentTargetsIDs = new ArrayList<>(); //sets up list of IDs for the field

        for (RawFiducial fiducial : currentFiducials) {//moves raw fiducial ids to target ids
            alCurrentTargetsIDs.add(fiducial.id);
        }

        //groups tags & picks a tag
        if (alCurrentTargetsIDs.contains(5) && alCurrentTargetsIDs.contains(8) && alCurrentTargetsIDs.contains(9) && alCurrentTargetsIDs.contains(10)){
            return 9;
        }
        else if (alCurrentTargetsIDs.contains(2) && alCurrentTargetsIDs.contains(9) && alCurrentTargetsIDs.contains(10) && alCurrentTargetsIDs.contains(11)){
            return 11;
        }
        else if (alCurrentTargetsIDs.contains(8) && alCurrentTargetsIDs.contains(9) && alCurrentTargetsIDs.contains(10)){
            return 9;
        }
        else if (alCurrentTargetsIDs.contains(10) && alCurrentTargetsIDs.contains(11) && alCurrentTargetsIDs.contains(2)){
            return 11;
        }
        else if (alCurrentTargetsIDs.contains(9) && alCurrentTargetsIDs.contains(10) && alCurrentTargetsIDs.contains(11)){
            return 10;
        }
        else if (alCurrentTargetsIDs.contains(9) && alCurrentTargetsIDs.contains(10)){
            return 10;
        }
        else if (alCurrentTargetsIDs.contains(11) && alCurrentTargetsIDs.contains(2)){
            return 11;
        }
        else if (alCurrentTargetsIDs.contains(10)){
            return 10;
        }
        else{
            return 0;
        }
   }

   public Double getDistToNearestTag(){

      double distance = 2.2;
      double halfofBumper = 0.87/2;   
        
    RawFiducial[] crtFiducials = getFiducialData();

      //moves raw fiducial to current target ids, takes only tx of the current target id    
         for (RawFiducial fiducial : crtFiducials) {

                if (fiducial.id == priorityTag()){
                    distance = fiducial.distToRobot; // X offset (no crosshair)
               }
         }
      SmartDashboard.putNumber("Dist to Robot", distance - halfofBumper);
      
      return distance - halfofBumper;
   }
   
  public double limelight_aim_proportional(Double robotMaxAngularSpeed, double currentTX){    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double AutoalignkP = 0.01; //0.035

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = currentTX * AutoalignkP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= robotMaxAngularSpeed;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;
    
    return targetingAngularVelocity;
  }

  public double limelight_range_proportional(Double robotMaxLinearSpeed){    
    double AutoalignkP = 1.0; 

    double targetingLinearVelocity = getTY() * AutoalignkP;

    // convert to radians per second for our drive method
    targetingLinearVelocity *= robotMaxLinearSpeed;

    targetingLinearVelocity *= -1.0;
    
    return targetingLinearVelocity;
  }

  //Raw fiducial data
  public RawFiducial[] getFiducialData() {
    // Get raw AprilTag/Fiducial data from the Limelight (assuming default name "limelight")
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(dmllName);

    return fiducials;
  }
  
  public boolean isCentered(){
    double txnc = 99;
    RawFiducial[] crtFiducials = getFiducialData();

      //moves raw fiducial to current target ids, takes only tx of the current target id    
    for (RawFiducial fiducial : crtFiducials) {

        if (fiducial.id == priorityTag()){
            txnc = fiducial.txnc; // X offset (no crosshair)
        }
    }

    if (-14 < txnc && txnc < 9){ //-23 & -30
        return true;
        
    }
    return false;
  }
}

