// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;

import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final Climber m_Climber = new Climber();

    public final Feeder m_Feeder = new Feeder();
    
    public final Hood m_Hood = new Hood();

    public final Indexer m_Indexer = new Indexer();

    public final Intake m_Intake = new Intake();

    public final Limelight m_Limelight = new Limelight();

    public final Shooter m_Shooter = new Shooter();

    
    // Remove comment later: private final SendableChooser<Command> autoChooser2;

    private final int redConst = -1;
    private int sideConst = -1;


    public RobotContainer() {
        /* Autos
        //Pathplanner Auto Commands

        //Climber
        NamedCommands.registerCommand("Deploy Hood", m_Climber.run(()-> {}));

        //Hood
        NamedCommands.registerCommand("Deploy Hood", m_Hood.run(()-> {m_Hood.toSetpoint(1);}));
        NamedCommands.registerCommand("Retract Hood", m_Hood.run(()-> {m_Hood.toSetpoint(0);}));

        //Indexer
        NamedCommands.registerCommand("Deploy Hood", m_Hood.run(()-> {}));


        //Intake
        NamedCommands.registerCommand("Deploy Intake", m_Intake.run(() -> {m_Intake.toSetpoint(1);}));
        NamedCommands.registerCommand("Retract Intake", m_Intake.run(() -> {m_Intake.toSetpoint(0);}));

        //Shooter 
        NamedCommands.registerCommand("Start Shoot", m_Indexer.run(() -> {m_Indexer.setIndexer(60);}));
        NamedCommands.registerCommand("Stop Shoot", m_Indexer.run(() -> {m_Indexer.setIndexer(0);}));

        //Other commands
        NamedCommands.registerCommand("Pose", drivetrain.runOnce(()-> drivetrain.resetPoseEstimator()));



        //Make an auto chooser on the smart dashboard
        autoChooser2 = AutoBuilder.buildAutoChooser();
        
        SmartDashboard.putData("Auto Chooser", autoChooser2); */

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        m_Shooter.setDefaultCommand(
            m_Shooter.setSpeed(0)
        );

        m_Feeder.setDefaultCommand(
            m_Feeder.setSpeed(0)
        );
        m_Indexer.setDefaultCommand(
            m_Indexer.setSpeed(0)
        );



        //m_Hood.setDefaultCommand(/*put hood down*/);

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Reset the field-centric heading on left bumper press.
        joystick.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Controls
        //joystick.y().whileTrue(new AutoalignRotate(m_Limelight, drivetrain,MaxAngularRate));

        //Test intake (change to left trigger)
        joystick.povUp().whileTrue(
            m_Intake.runEnd(
                () -> {m_Intake.setIntake(5);}, 
                () -> {m_Intake.setIntake(0);}
            )
        );

        joystick.povDown().whileTrue(
            m_Intake.runEnd(
                () -> {m_Intake.setIntake(-5);}, 
                () -> {m_Intake.setIntake(0);}
            )
        );


        //Get position of intake encoder at position
        joystick.x().whileTrue(
            m_Intake.runOnce(() -> m_Intake.getPosition())
        );

        //Test hood
        joystick.a().whileTrue(
            m_Hood.runEnd(
                () -> {m_Hood.setHood(-5);},
                () -> {m_Hood.setHood(0);}
            )
        );
        joystick.y().whileTrue(
            m_Hood.runEnd(
                () -> {m_Hood.setHood(5);},
                () -> {m_Hood.setHood(0);}
            )
        );

        joystick.b().whileTrue(
            m_Hood.runOnce(() -> {m_Hood.getPosition();})
        );


    


        //Run climb
        /*joystick.povUp().whileTrue(
            m_Climber.runEnd(
                () -> {}, 
                () -> {}
            )
        );*/

        //run intake
        /*joystick.axisGreaterThan(2, 0.05).whileTrue(
            m_Intake.runEnd( //REMEMBER TO CHANGE SET POINT VALUE
                () -> {m_Intake.setRollers(60); m_Intake.toSetpoint(0); }, 
                () -> {m_Intake.setRollers(0); m_Intake.toSetpoint(0);}
            )
        );*/
    
        joystick.rightTrigger(0.05).whileTrue(
            Commands.parallel(
                m_Shooter.setSpeed(42),
                Commands.waitSeconds(1)
                .andThen(
                    Commands.parallel(
                        m_Feeder.setSpeed(65),
                        m_Indexer.setSpeed(65)
                    )
                )
            )
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }
    /* Remove comment later:
    public Command getAutonomousCommand() {
        return autoChooser2.getSelected();
    } */
}
