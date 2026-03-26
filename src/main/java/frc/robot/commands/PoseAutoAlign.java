package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.*;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class PoseAutoAlign extends Command {

    CommandSwerveDrivetrain DRIVETRAIN;
    SwerveRequest.FieldCentric robotDrive;


    Double maxAnglSpeed; //Max Angular Speed

    Translation2d hubPose = new Translation2d();

    public PoseAutoAlign(CommandSwerveDrivetrain drivetrain, Double maxAngularSpeed) 
    {
        this.DRIVETRAIN = drivetrain;
        robotDrive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        maxAnglSpeed = maxAngularSpeed;

        addRequirements(drivetrain);
    }

    //puts the Target ID on the Dashboard
    public void initialize()
    {   
        DRIVETRAIN.setControl(
            robotDrive.withRotationalRate(0)
                      .withVelocityX(0)
                      .withVelocityY(0)
        );
    }

    //runs 
    public void execute()
    {
        DRIVETRAIN.setControl(
            robotDrive.withVelocityX(0)
        );
        //runs "isFinished" function to say how it is done
    }

    public void end(boolean interupted){
        DRIVETRAIN.setControl(
            robotDrive.withRotationalRate(0)
        );
    }
}