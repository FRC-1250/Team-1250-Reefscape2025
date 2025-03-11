// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.HealthStatus;
import frc.robot.util.TalonHealthChecker;

public class DeepClimber extends SubsystemBase {

  private TalonFX deepClimber = new TalonFX(41);
   private final boolean healthCheckEnabled = true; 
  private HealthStatus healthStatus = HealthStatus.IS_OK;
  private TalonHealthChecker deepClimberCheck;
  private TorqueCurrentFOC torqueControl = new TorqueCurrentFOC(0);
  private final double requiredRotations = 0; // 235 ~= 13 inches of travel on the deepClimber


  /** Creates a new DeepClimb. */
  public DeepClimber() {
     MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
    motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
    talonFXConfiguration.MotorOutput = motorOutputConfigs;

    deepClimber.getConfigurator().apply(talonFXConfiguration);
    deepClimber.setPosition(0);
    deepClimber.getPosition().setUpdateFrequency(200);
    if (healthCheckEnabled) {
      deepClimberCheck = new TalonHealthChecker(deepClimber, getName());
    }
  }

  @Logged
  public HealthStatus getHealthStatus() {
    return healthStatus;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setTorque(Current newCurrent) {
    deepClimber.setControl(torqueControl.withOutput(newCurrent));
  }

  @Logged(name = "deep climber position")
  public double getRotations(){
    return deepClimber.getPosition().getValueAsDouble();
  }
  


}
