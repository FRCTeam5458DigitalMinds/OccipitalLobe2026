// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    TalonFX climberMotor;

    private final double[] setpoints = {};

    private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    public Climber() {
        climberMotor = new TalonFX(Constants.ClimbConstants.climbMotor);

        TalonFXConfiguration climbConfigs = new TalonFXConfiguration();
        //setups the PID value for the intake
        climbConfigs.Slot0.kP = Constants.ClimbConstants.climb_P;
        climbConfigs.Slot0.kD = Constants.ClimbConstants.climb_D; 

        climbConfigs.CurrentLimits.withStatorCurrentLimit(40);
        climbConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);

        climberMotor.getConfigurator().apply(climbConfigs);
    }

    public void setClimber(double OutputPercent){
        OutputPercent /= 100.0;
        climberMotor.set(OutputPercent);
    }
}
