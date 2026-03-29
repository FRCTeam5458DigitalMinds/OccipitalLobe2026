package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.*;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class PoseAutoAlign extends Command {

    CommandSwerveDrivetrain DRIVETRAIN;
    SwerveRequest.FieldCentric robotDrive;
    public double targetRotation;

    public double AngleError;
    
    private static final Translation2d shooterOffset = new Translation2d(Inch.of(9).in(Meters), 0.184);

    public PoseAutoAlign(CommandSwerveDrivetrain drivetrain) {
        this.DRIVETRAIN = drivetrain;
        robotDrive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        addRequirements(drivetrain);

    }

    // puts the Target ID on the Dashboard
    public void initialize() {
    }

    // runs
    public void execute() {

        Pose2d pose = DRIVETRAIN.getPose();

        Translation2d shooterPose = pose.getTranslation().plus(shooterOffset.rotateBy(pose.getRotation()));

        Translation2d toHub = DRIVETRAIN.setHub().minus(shooterPose);
        targetRotation = toHub.getAngle().getRadians();

        AngleError = targetRotation - pose.getRotation().getRadians();
        AngleError = MathUtil.angleModulus(AngleError);

        DRIVETRAIN.setControl(
            robotDrive.withVelocityX(0)
                      .withVelocityY(0)
                      .withRotationalRate(AngleError*4)
        );      
        isFinished();
    }

    // sets boolean for its done to stop the comand
    @Override
    public boolean isFinished() {
        return (Math.abs(AngleError) < Math.toRadians(1.5));
    }

    @Override
    public void end(boolean interrupted){
        DRIVETRAIN.setControl(
            new SwerveRequest.Idle() 
        );  
    }
}