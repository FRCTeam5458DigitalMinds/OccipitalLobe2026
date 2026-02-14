// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

import com.ctre.phoenix6.hardware.Pigeon2;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    private final String dmllName = Constants.LimelightConstants.ll_Name;
    
    private final Pigeon2 pigeon = new Pigeon2(TunerConstants.kPigeonId);

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        // Retrieves the voltage currently entering the roboRIO
        double batteryVoltage = RobotController.getBatteryVoltage();
    
        // Sends the value to SmartDashboard under the key "Battery Voltage"
        SmartDashboard.putNumber("Battery Voltage", batteryVoltage);
        CommandScheduler.getInstance().run(); 
        
    }

    @Override
    public void disabledInit() {
        NetworkTableInstance.getDefault().getTable(dmllName).getEntry("throttle_set").setNumber(100);
    }

    @Override
    public void disabledPeriodic() {
        LimelightHelpers.SetIMUMode(dmllName, 1); // Seed internal IMU (1)

    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        /*m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }*/
        NetworkTableInstance.getDefault().getTable(dmllName).getEntry("throttle_set").setNumber(0);

    }

    @Override
    public void autonomousPeriodic() {
        LimelightHelpers.SetIMUMode(dmllName, 0);
    }

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        //Removes throttling(slowdown) of the limelight
        NetworkTableInstance.getDefault().getTable(dmllName).getEntry("throttle_set").setNumber(0);

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {
        LimelightHelpers.SetIMUMode(dmllName, 4);
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
