package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase{
    private PowerDistribution sparkPDH = new PowerDistribution(1, ModuleType.kRev);

    public Command LEDon(){
     return runOnce(() -> {sparkPDH.setSwitchableChannel(true);});
    }
  
    public Command LEDoff(){
     return runOnce(() -> {sparkPDH.setSwitchableChannel(false);});
    }

    public Command blink(){
        return 
            LEDon()
            .andThen(Commands.waitSeconds(0.25))
            .andThen(LEDoff())
            .andThen(Commands.waitSeconds(0.25));
    }
}
