package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.*;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AutoalignRotate extends Command {

    Limelight LIMELIGHT;
    CommandSwerveDrivetrain DRIVETRAIN;
    SwerveRequest.RobotCentric robotDrive;

    double range = Constants.LimelightConstants.range;


    Double maxAnglSpeed; //Max Angular Speed

    public AutoalignRotate(Limelight limelight, CommandSwerveDrivetrain drivetrain, Double maxAngularSpeed) 
    {
        this.LIMELIGHT = limelight;
        this.DRIVETRAIN = drivetrain;
        robotDrive = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        maxAnglSpeed = maxAngularSpeed;

        addRequirements(limelight);
        addRequirements(drivetrain);
    
    }
    //puts the Target ID on the Dashboard
    public void initialize()
    {   
        SmartDashboard.putNumber("ID", LIMELIGHT.getTargetID());
        
    }

    //runs 
    public void execute()
    {
        
        boolean isCentered = false;
        int crtTargetID = 0;
        double currentTXNC = 0.0;

        
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

//gets tx from target id, autoaligns, keeps looping until tx centered
        do{

            //gets raw fiducial (again)
        
            RawFiducial[] crtFiducials = LIMELIGHT.getFiducialData();

      //moves raw fiducial to current target ids, takes only tx of the current target id    
            for (RawFiducial fiducial : crtFiducials) {

                if (fiducial.id == crtTargetID){
                    currentTXNC = fiducial.txnc; // X offset (no crosshair)
                }
            }

            if (crtTargetID != 0){

                    //Runs function to get turn speed
                    double turnSpeed = LIMELIGHT.limelight_aim_proportional(maxAnglSpeed);

                    SmartDashboard.putNumber("TXNC", currentTXNC);

                    //Uses turn speed to run robot
                    DRIVETRAIN.setControl(robotDrive.withRotationalRate(turnSpeed));

                    //If crosshair is between these horizontal values , stop
                    if (-range < LIMELIGHT.getTX() && LIMELIGHT.getTX() < range){
                        isCentered = true;
                    }

            }
            else {

                //End loop in case no tag seen
                isCentered = true;
            }

        } while (isCentered == false); //stop when tag is centered

        //runs "isFinished" function to say how it is done
        isFinished();
    }


    //sets boolean for its done to stop the comand
    public boolean isFinished()
    {
      return true;
    }
}