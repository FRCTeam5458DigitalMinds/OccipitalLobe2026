package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase{
    private PowerDistribution sparkPDH = new PowerDistribution(Constants.LEDConstants.PDH, ModuleType.kRev);

    public Command LEDon(){
     return run(() -> {turnOn();});
    }
  
    public Command LEDoff(){
     return run(() -> {turnOff();});
    }

    public void turnOff(){
        sparkPDH.setSwitchableChannel(false);

    }

    public void turnOn(){
        sparkPDH.setSwitchableChannel(true);
    }

    public Command blink(){
        return 
            LEDon()
            .andThen(Commands.waitSeconds(0.25))
            .andThen(LEDoff())
            .andThen(Commands.waitSeconds(0.25));
    }
}
