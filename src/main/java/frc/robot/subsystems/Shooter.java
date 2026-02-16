// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Shooter extends SubsystemBase {
    
    //Initalizes the motors
    private TalonFX lowerFlyMotor;
    private TalonFX upperFlyMotor;

    //sets up velocity PID for slot 0
    //final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    //Calculates the max Revolutions Per Second
    private final double RPS = 5500/60;


    public Shooter() {
        
        //Sets settings for the Lower Flywheel
        
        TalonFXConfiguration globalConfigs = new TalonFXConfiguration();
        globalConfigs.CurrentLimits.withStatorCurrentLimit(40);
        globalConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);
        
        //setups the PID value for the intake
        //globalConfigs.Slot0.kS = 0.1; // Add 0.1 V output to overcome static friction
        //globalConfigs.Slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        //globalConfigs.Slot0.kP = Constants.ShooterConstants.p_Value;
        
        lowerFlyMotor = new TalonFX(Constants.ShooterConstants.lowerFlyWheel);
        
        lowerFlyMotor.getConfigurator().apply(globalConfigs);

        //Sets settings for Upper Flywheel
        
        upperFlyMotor = new TalonFX(Constants.ShooterConstants.upperFlyWheel);

        upperFlyMotor.getConfigurator().apply(globalConfigs);

        upperFlyMotor.setControl(new Follower(lowerFlyMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }


    //sets up a command for the speed of the motors
    public Command setSpeed(double OutputPercent){
      return run(
          () -> {
            setLowerFly(OutputPercent);
          }
      );
    }
    //Made seperate because it doesn't use setControl
    public Command stopMotors(){
      return run(
          () -> {
            setLowerFly(0);
          }
      );
    }

    //Sets the speed of the Lower Flywheel
    public void setLowerFly(double OutputPercent)
    {
      OutputPercent /= 100.0;
      lowerFlyMotor.set(OutputPercent);
      //lowerFlyMotor.setControl(m_request.withVelocity(RPS*OutputPercent));
    }

}
