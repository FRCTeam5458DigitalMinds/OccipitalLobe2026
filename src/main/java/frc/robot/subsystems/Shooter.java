// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    
    private TalonFX lowerFlyMotor;
    private TalonFX upperFlyMotor;
    private TalonFX feedMotor;

    public Shooter() {
        lowerFlyMotor = new TalonFX(Constants.ShooterConstants.lowerFlyWheel);
        TalonFXConfiguration lowerConfigs = new TalonFXConfiguration();
        lowerConfigs.CurrentLimits.withStatorCurrentLimit(40);
        lowerConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);
        lowerFlyMotor.getConfigurator().apply(lowerConfigs);

        
        upperFlyMotor = new TalonFX(Constants.ShooterConstants.upperFlyWheel);
        TalonFXConfiguration upperConfigs = new TalonFXConfiguration();
        upperConfigs.CurrentLimits.withStatorCurrentLimit(40);
        upperConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);
        upperFlyMotor.getConfigurator().apply(upperConfigs);

        feedMotor = new TalonFX(Constants.ShooterConstants.feeder);
        TalonFXConfiguration feedConfigs = new TalonFXConfiguration();
        feedConfigs.CurrentLimits.withStatorCurrentLimit(40);
        feedConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);
        feedMotor.getConfigurator().apply(feedConfigs);

        setDefaultCommand(
          runOnce(
              () -> {
              setLowerFly(0);
              setUpperFly(0);   
              setFeeder(0);         
            }).andThen(run(() -> {})));
    }

    public Command setSpeed(double OutputPercent){
      return run(
          () -> {
            lowerFlyMotor.set(OutputPercent);
            upperFlyMotor.set(OutputPercent);
            feedMotor.set(OutputPercent);
          });
    }
    public void setLowerFly(double OutputPercent)
    {
      OutputPercent /= 100.;
      lowerFlyMotor.set(OutputPercent);
    }

     public void setUpperFly(double OutputPercent)
    {
      OutputPercent /= 100.;
      upperFlyMotor.set(-OutputPercent);
    }

     public void setFeeder(double OutputPercent)
    {
      OutputPercent /= 100.;
      feedMotor.set(-OutputPercent);
    }

}
