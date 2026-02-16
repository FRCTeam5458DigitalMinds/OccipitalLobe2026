package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Limelight extends SubsystemBase{
   //SwerveDrive class
   CommandSwerveDrivetrain drivetrain;
   SwerveRequest.RobotCentric robotDrive;
   
   //name of limelight IDs (don't forget)
   String dmllName = Constants.LimelightConstants.ll_Name;
   
   //info from tag
   int id;

   int selectedPipeline;

//pose thingys
   private LimelightHelpers.PoseEstimate limelightMeasurement = new LimelightHelpers.PoseEstimate();

   public Limelight(){   
   
      //selectedPipeline = SmartDashboard.getNumber("Pipeline Chooser_selected", 0)
      LimelightHelpers.setPipelineIndex(dmllName,Constants.LimelightConstants.pipeline);
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
   
  public double limelight_aim_proportional(Double robotMaxAngularSpeed){    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double AutoalignkP = 0.01; //0.035

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = getTX() * AutoalignkP;

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
}

