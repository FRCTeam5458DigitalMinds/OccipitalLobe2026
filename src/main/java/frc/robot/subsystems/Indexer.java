// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {

    TalonFX indexerMotor;

    public Indexer() {
        indexerMotor = new TalonFX(Constants.IndexerConstants.indexMotor);
        TalonFXConfiguration indexerConfigs = new TalonFXConfiguration();
        indexerConfigs.CurrentLimits.withStatorCurrentLimit(40);
        indexerConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);

        indexerMotor.getConfigurator().apply(indexerConfigs);

    }

    public void setIndexer(double OutputPercent)
    {
        OutputPercent /= 100.0;
        indexerMotor.set(OutputPercent);
    }
}
