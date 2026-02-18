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
import frc.robot.Constants.LimelightConstants;
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

    public final LED m_LED = new LED();

    public final Limelight m_Limelight = new Limelight();

    public final Roller m_Roller = new Roller();

    public final Shooter m_Shooter = new Shooter();

    
    private final SendableChooser<Command> autoChooser2;

    private final SendableChooser<Integer> limelightPipelineChooser;

    private final int redConst = -1;
    private int sideConst = -1;


    public RobotContainer() {
        //Autos
        //Pathplanner Auto Commands

        //Climber
        //NamedCommands.registerCommand("Deploy Hood", m_Climber.run(()-> {}));

        //Hood
        //NamedCommands.registerCommand("Deploy Hood", m_Hood.run(()-> {m_Hood.toSetpoint(1);}));
        //NamedCommands.registerCommand("Retract Hood", m_Hood.run(()-> {m_Hood.toSetpoint(0);}));

        //Indexer
        //NamedCommands.registerCommand("Deploy Hood", m_Hood.run(()-> {}));


        //Intake
        //NamedCommands.registerCommand("Deploy Intake", m_Intake.run(() -> {m_Intake.toSetpoint(1);}));
        //NamedCommands.registerCommand("Retract Intake", m_Intake.run(() -> {m_Intake.toSetpoint(0);}));

        //Shooter 
        NamedCommands.registerCommand(
            "Start Shoot", 
            Commands.parallel(
                //note: speed needed directly in front of the tower
                m_Shooter.setSpeed(42),
                Commands.waitSeconds(0.5)
                .andThen(
                    Commands.parallel(
                        m_Feeder.setSpeed(65),
                        m_Indexer.setSpeed(65)
                    )
                )
            )         
        );

        NamedCommands.registerCommand(
            "Stop Shoot", 
            m_Feeder.setSpeed(0)
            .andThen(m_Indexer.setSpeed(0))
            .andThen(m_Shooter.stopMotors())
            //.andThen(m_Hood.toSetpoint(0))
        );

        //Other commands
        NamedCommands.registerCommand("Pose", drivetrain.runOnce(()-> drivetrain.resetPoseEstimator()));
        //NamedCommands.registerCommand("Auto Rotate", new AutoalignRotate(m_Limelight, drivetrain,MaxAngularRate));

        //Make an auto chooser on the smart dashboard
        autoChooser2 = AutoBuilder.buildAutoChooser();
        
        SmartDashboard.putData("Auto Chooser", autoChooser2); 

        limelightPipelineChooser = new SendableChooser<>();

        limelightPipelineChooser.setDefaultOption("Day", 0);
        limelightPipelineChooser.addOption("Night", 1);
        limelightPipelineChooser.addOption("Comp", 2);

        SmartDashboard.putData("Pipeline Chooser", limelightPipelineChooser);

        LimelightConstants.pipeline = limelightPipelineChooser.getSelected();

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

        //Stops the listed motors when not used
        m_Feeder.setDefaultCommand(
            m_Feeder.setSpeed(0)
        );

        /*m_Hood.setDefaultCommand(
            m_Hood.toSetpoint(0)
        );*/

        m_Indexer.setDefaultCommand(
            m_Indexer.setSpeed(0)
        );

        m_LED.setDefaultCommand(
            m_LED.LEDoff()
        );

        m_Roller.setDefaultCommand(
            m_Roller.setSpeed(0)
        );

        m_Shooter.setDefaultCommand(
            m_Shooter.stopMotors()
        );



        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Reset the field-centric heading on left bumper press.
        joystick.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Controls
        joystick.start().whileTrue(new AutoalignRotate(m_Limelight, drivetrain,MaxAngularRate));


        //Test intake (change to left trigger)
        joystick.povUp().whileTrue(
            m_Intake.runEnd(
                () -> {m_Intake.setIntake(90);}, 
                () -> {m_Intake.setIntake(0);}
            )
        );


        //Testing purposes only
        joystick.povDown().whileTrue(
            m_Intake.runEnd(
                () -> {m_Intake.setIntake(-90);}, 
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


        //Test hood
        joystick.y().whileTrue(
            m_Hood.runEnd(
                () -> {m_Hood.setHood(5);},
                () -> {m_Hood.setHood(0);}
            )
        );


        //Sends the encoder value of the Hood to the dashboard 
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

        //Extends intake
        joystick.leftTrigger(0.05).whileTrue(
            Commands.parallel(
                m_Intake.run(
                    () -> {m_Intake.toSetpoint(1);}
                ),
                m_Roller.setSpeed(80)
            )
        );

        /*joystick.leftBumper().whileTrue(
            m_Intake.run(
                () -> {m_Intake.toSetpoint(0);}
            )
        );*/

        //Runs Shooter, waits, then runs the Feeder and Indexer
        joystick.rightTrigger(0.05).whileTrue(
            Commands.parallel(
                //note: speed needed directly in front of the tower
                m_Shooter.setSpeed(60), //
                Commands.waitSeconds(1)
                .andThen(
                    Commands.parallel(
                        m_Feeder.setSpeed(65), //
                        m_Indexer.setSpeed(65)
                    )
                    .andThen(m_LED.blink().repeatedly())
                )
            )
        );

        //Same as above but for feed mode
        joystick.rightBumper().whileTrue(
            Commands.parallel(
                //note: speed needed directly in front of the tower
                Commands.parallel(
                    m_Hood.toSetpoint(1),
                    m_Shooter.setSpeed(42)),
                Commands.waitSeconds(0.5)
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

    
    public Command getAutonomousCommand() {
        return autoChooser2.getSelected();
    }
}
