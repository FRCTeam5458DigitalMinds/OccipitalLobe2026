// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
    
    private TalonFX feedMotor;

    public Feeder() {
        
        feedMotor = new TalonFX(Constants.FeederConstants.feeder);
        TalonFXConfiguration feedConfigs = new TalonFXConfiguration();
        feedConfigs.CurrentLimits.withStatorCurrentLimit(40);
        feedConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);
        feedMotor.getConfigurator().apply(feedConfigs);
       
    }

    public Command setSpeed(double OutputPercent){
      return run(
          () -> {
            setFeeder(OutputPercent);
          });
    }

     public void setFeeder(double OutputPercent)
    {
      OutputPercent /= 100.;
      feedMotor.set(-OutputPercent);
    }

}
