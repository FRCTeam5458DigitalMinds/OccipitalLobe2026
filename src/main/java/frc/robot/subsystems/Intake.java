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

public class Intake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  
  private TalonFX intakeMotor;

  //, max, other number...
  private final double[] setpoints = {0.28466796875,17.18115234375};

  private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

  
  public Intake() {

    //Set up motors
        intakeMotor = new TalonFX(Constants.IntakeConstants.intakeMotor);
        
        TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();
        //setups the PID value for the intake
        intakeConfigs.Slot0.kP = Constants.IntakeConstants.intake_P;
        intakeConfigs.Slot0.kD = Constants.IntakeConstants.intake_D; 

        //Setups current limits
        intakeConfigs.CurrentLimits.withStatorCurrentLimit(60);
        intakeConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);

        intakeMotor.getConfigurator().apply(intakeConfigs);


  }

  /*public Command intakeCmd(){
    return runOnce(null)

  }
  public Command retractCmd(){
  
  }*/

    //Negative means extends
    public void setIntake(double OutputPercent)
    {
      OutputPercent /= 100.;
      intakeMotor.set(-OutputPercent);
    }

     //Go to certain position based on setpoint index
    public void toSetpoint(int setpointIndex)
    {
        intakeMotor.setControl(m_request.withPosition(setpoints[setpointIndex]).withSlot(0));
    }


    //testing purposes only
    public void customPosition(double setPoint)
    {
        intakeMotor.setControl(m_request.withPosition(setPoint).withSlot(0));
    }

    //more testing
    public double getPosition()
    {
        double intakeEncoder = intakeMotor.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("Intake Position", intakeEncoder);
        return intakeEncoder;
    }
    //Min: 2.1435546875
    //Max: 17.18115234375
}
