package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase{
    private PowerDistribution sparkPDH = new PowerDistribution(1, ModuleType.kRev);

    public void LEDon(){
    sparkPDH.setSwitchableChannel(true);
    }
  
    public void LEDoff(){
    sparkPDH.setSwitchableChannel(false);
    }
}
