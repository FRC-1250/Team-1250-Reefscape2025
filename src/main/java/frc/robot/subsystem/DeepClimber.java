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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.HealthMonitor;

public class DeepClimber extends SubsystemBase {

    public enum DeepClimberPhase {
        CLIMB(285);

        public final double rotations;

        DeepClimberPhase(double rotations) {
            this.rotations = rotations;
        }
    }

    private TalonFX deepClimber = new TalonFX(41);
    private TorqueCurrentFOC torqueControl = new TorqueCurrentFOC(0);

    /** Creates a new DeepClimb. */
    public DeepClimber() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.MotorOutput = motorOutputConfigs;
        talonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = false;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = false;

        deepClimber.getConfigurator().apply(talonFXConfiguration);
        deepClimber.setPosition(0);
        deepClimber.getPosition().setUpdateFrequency(200);

        HealthMonitor.getInstance().addComponent(getName(), "Winch", deepClimber);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void stop() {
        deepClimber.stopMotor();
    }

    public boolean hasPassedClimbThreshold(DeepClimberPhase deepClimberPhase) {
        return getRotations() >= deepClimberPhase.rotations;
    }

    public void setTorque(Current amps) {
        deepClimber.setControl(torqueControl.withOutput(amps));
    }

    public boolean isNearPositionAndTolerance(double position, double tolerance) {
        return MathUtil.isNear(position, getRotations(), tolerance);
    }

    @Logged(name = "Position")
    public double getRotations() {
        return deepClimber.getPosition().getValueAsDouble();
    }

    @Logged(name = "Amps")
    public double getCurrent() {
        return deepClimber.getStatorCurrent().getValueAsDouble();
    }

}