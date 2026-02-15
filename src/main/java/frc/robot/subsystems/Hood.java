// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  TalonFX hoodMotor;
  //min, max, (later numbers)
  private final double[] setpoints = {-0.05712890625, 10.63232421875};
  private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

  public Hood() {
        hoodMotor = new TalonFX(Constants.HoodConstants.hoodMotor);

        TalonFXConfiguration hoodConfigs = new TalonFXConfiguration();

        //setups the PID value for the intake
        hoodConfigs.Slot0.kP = Constants.HoodConstants.hood_P;
        hoodConfigs.Slot0.kD = Constants.HoodConstants.hood_D; 

        hoodConfigs.CurrentLimits.withStatorCurrentLimit(40);
        hoodConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);

        hoodMotor.getConfigurator().apply(hoodConfigs);
    }


    //Set speed for now
    public void setHood(double OutputPercent){
        OutputPercent /= 100.0;
        hoodMotor.set(OutputPercent);
    }

    //Go to certain position based on setpoint index
    public Command toSetpoint(int setpointIndex)
    {
        return runOnce(
            () -> {hoodMotor.setControl(m_request.withPosition(setpoints[setpointIndex]).withSlot(0));}
        );
    }


    //testing purposes only
    public void customPosition(double setPoint)
    {
        hoodMotor.setControl(m_request.withPosition(setPoint).withSlot(0));
    }
    
    //more testing
    public double getPosition()
    {
        double encoder = hoodMotor.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("Hood encoder", encoder);
        return encoder;
    }

}
