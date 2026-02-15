// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    
    private TalonFX lowerFlyMotor;
    private TalonFX upperFlyMotor;

    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    private final double RPS = 5500/60;

    public Shooter() {
        lowerFlyMotor = new TalonFX(Constants.ShooterConstants.lowerFlyWheel);
        TalonFXConfiguration lowerConfigs = new TalonFXConfiguration();
        lowerConfigs.CurrentLimits.withStatorCurrentLimit(40);
        lowerConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);
        //setups the PID value for the intake
        lowerConfigs.Slot0.kP = Constants.ShooterConstants.lower_P;
        
        lowerFlyMotor.getConfigurator().apply(lowerConfigs);

        
        upperFlyMotor = new TalonFX(Constants.ShooterConstants.upperFlyWheel);
        TalonFXConfiguration upperConfigs = new TalonFXConfiguration();
        upperConfigs.CurrentLimits.withStatorCurrentLimit(40);
        upperConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);
        //setups the PID value for the shooter
        upperConfigs.Slot0.kP = Constants.ShooterConstants.upper_P;

        upperFlyMotor.getConfigurator().apply(upperConfigs);
    }


    public Command setSpeed(double OutputPercent){
      return run(
          () -> {
            setLowerFly(OutputPercent);
            setUpperFly(OutputPercent);
          });
    }

    public void setLowerFly(double OutputPercent)
    {
      OutputPercent /= 100.0;
      lowerFlyMotor.setControl(m_request.withVelocity(RPS*OutputPercent));
    }

     public void setUpperFly(double OutputPercent)
    {
      OutputPercent /= 100.0;
      upperFlyMotor.setControl(m_request.withVelocity(RPS*-OutputPercent));      
    }

}
