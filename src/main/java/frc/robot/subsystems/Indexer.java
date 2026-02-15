// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {

    //Initializes the Indexer Motor
    TalonFX indexerMotor;

    public Indexer() {

        //Sets up the settings for the Indexer Motor
        indexerMotor = new TalonFX(Constants.IndexerConstants.indexMotor);
        TalonFXConfiguration indexerConfigs = new TalonFXConfiguration();
        indexerConfigs.CurrentLimits.withStatorCurrentLimit(20);
        indexerConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);

        indexerMotor.getConfigurator().apply(indexerConfigs);

    }

    //Sets the speed for the Indexer Motor
    public Command setSpeed(double OutputPercent){
        return run(
            () -> {
                setIndexer(OutputPercent);
            });
    }

    //Sets the speed of the Indexer Motor
    public void setIndexer(double OutputPercent)
    {
        OutputPercent /= 100.0;
        indexerMotor.set(OutputPercent);
    }
}
