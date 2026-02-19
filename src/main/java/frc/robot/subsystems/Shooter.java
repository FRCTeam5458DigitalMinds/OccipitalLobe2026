// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;




public class Shooter extends SubsystemBase {
    
    //Initalizes the motors
    private TalonFX lowerFlyMotor;
    private TalonFX upperFlyMotor;

    //sets up velocity PID for slot 0
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    //Calculates the max Revolutions Per Second
    private final double maxRPM = 5500;
    /*Notes:
    10%: 550
    20%  1100
    30%  1650
    40%  2200
    50%  2750
    60%  3300
    70%  3850
    80%  4400
    90%  4950
    100% 5500
    */
    /*private final SimpleMotorFeedforward m_shooterFeedforward =
      new SimpleMotorFeedforward(
          ShooterConstants.kSVolts, ShooterConstants.kVVoltSecondsPerRotation);

    private final PIDController m_shooterFeedback = new PIDController(Constants.ShooterConstants.p_Value, 0.0, 0.0);*/

    public Shooter() {

        //m_shooterFeedback.setTolerance(Constants.ShooterConstants.kShooterToleranceRPS);
        
        //Sets settings for the Lower Flywheel
        
        TalonFXConfiguration globalConfigs = new TalonFXConfiguration();
        globalConfigs.CurrentLimits.withStatorCurrentLimit(40);
        globalConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);
        
        //setups the PID value for the intake
      
        globalConfigs.Slot0.kS = Constants.ShooterConstants.s_Value;
        globalConfigs.Slot0.kV = Constants.ShooterConstants.v_Value; // A velocity target of 1 rps results in 0.12 V output
        globalConfigs.Slot0.kP = Constants.ShooterConstants.p_Value;
        globalConfigs.Slot0.kI = Constants.ShooterConstants.i_Value;
        globalConfigs.Slot0.kD = Constants.ShooterConstants.d_Value;

        globalConfigs.Voltage.withPeakForwardVoltage(8)
        .withPeakReverseVoltage(-8);
        
        lowerFlyMotor = new TalonFX(Constants.ShooterConstants.lowerFlyWheel);
        
        lowerFlyMotor.getConfigurator().apply(globalConfigs);

        //Sets settings for Upper Flywheel
        
        upperFlyMotor = new TalonFX(Constants.ShooterConstants.upperFlyWheel);

        upperFlyMotor.getConfigurator().apply(globalConfigs);

        upperFlyMotor.setControl(new Follower(lowerFlyMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

  
    /*//sets up a command for the speed of the motors
    public Command setSpeed(double percent){
      return run(
          () -> {
            setLowerFly(percent);
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
      SmartDashboard.putNumber("Flywheel speed", OutputPercent);
      //lowerFlyMotor.setControl(m_request.withVelocity(RPS*OutputPercent));
    }*/
    

 
    public Command PIDrunMotors(double FlyRPS){
      return run(
          () -> {
            setTargetRPM(FlyRPS);
          }
      );
    }
    public Command stopMotors(){
      return run(
          () -> {
            lowerFlyMotor.set(0);
          }
      );
    }

    public void setTargetRPM(double RPS){
      SmartDashboard.putNumber("RPS", lowerFlyMotor.getVelocity().getValueAsDouble());
      lowerFlyMotor.setControl(m_request.withVelocity(RPS));
    }

}
