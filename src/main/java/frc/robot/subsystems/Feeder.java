// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Volt;


public class Feeder extends SubsystemBase {
    
    private TalonFX feedMotor;

    private BangBangController controller = new BangBangController();

    Double testRPS;



    private final SysIdRoutine m_sysIdRoutine;
    private final VoltageOut m_voltReq = new VoltageOut(0.0);



    public Feeder() {
        
        feedMotor = new TalonFX(Constants.FeederConstants.feeder);
        TalonFXConfiguration feedConfigs = new TalonFXConfiguration();
        feedConfigs.CurrentLimits.withStatorCurrentLimit(40);
        feedConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);
        feedMotor.getConfigurator().apply(feedConfigs);
        
        feedMotor.setNeutralMode(NeutralModeValue.Coast); 
        
        SmartDashboard.putNumber("Feeder Test RPS", 85);

        m_sysIdRoutine = new SysIdRoutine(
              new SysIdRoutine.Config(
         null,        // Use default ramp rate (1 V/s)
                  Volt.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
          null,        // Use default timeout (10 s)
                      // Log state with Phoenix SignalLogger class
         (state) -> SignalLogger.writeString("Feed state", state.toString())
        ),
          new SysIdRoutine.Mechanism(
          (volts) -> feedMotor.setControl(m_voltReq.withOutput(volts.in(Volt))),
          null,
          this
        )
      );
 
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
      return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
      return m_sysIdRoutine.dynamic(direction);
    }
    

    //Sets speed of feeder
    public Command setSpeed(double OutputPercent){
      return run(
          () -> {
            setFeeder(OutputPercent);
          });
    }
    //Just Bang Bang
    public Command BBtestMotors(){
      return run(
        () -> {
          BBrps(testRPS);
        }
      );
    } 
    public Command stopMotors(){
      return run(
        () -> {
          setFeeder(0);
        }
      );
    } 


    public void BBrps(double RPS){
        feedMotor.set(-controller.calculate(feedMotor.getVelocity().getValueAsDouble(), RPS));
    }

    //Function version of setting speed
     public void setFeeder(double OutputPercent)
    {
      OutputPercent /= 100.;
      feedMotor.set(-OutputPercent);
    }
    //Continuously runs
    @Override
    public void periodic() {
      testRPS = SmartDashboard.getNumber("Feeder Test RPS", 85);
      SmartDashboard.putNumber("Feeder RPS", feedMotor.getVelocity().getValueAsDouble());

    } 

}
