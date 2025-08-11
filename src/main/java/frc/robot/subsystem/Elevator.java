// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.NotifierCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.HealthMonitor;

public class Elevator extends SubsystemBase {
    public enum ElevatorPosition {
        HOME(1),
        SENSOR(0.8),
        L1(6.08),
        LOW_ALGAE(18),
        HIGH_ALGAE(32),
        BARGE(73),
        PEAK(74);

        public final double rotations;

        ElevatorPosition(double rotations) {
            this.rotations = rotations;
        }
    }

    private TalonFX leftMotor = new TalonFX(30);
    private TalonFX rightMotor = new TalonFX(31);
    private DigitalInput homeSensor = new DigitalInput(1);
    private MotionMagicVoltage motionMagicPostionControl = new MotionMagicVoltage(0).withEnableFOC(false);
    private boolean homeFound = false;
    private boolean previousHomeSensor = isAtHome();

    public Elevator() {
        Slot0Configs positionPIDConfigs = new Slot0Configs()
                .withGravityType(GravityTypeValue.Elevator_Static)
                .withKG(0.3)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                .withKS(0.2)
                .withKV(0.3)
                .withKA(0.01)
                .withKP(1)
                .withKI(0)
                .withKD(0.2);

        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(RotationsPerSecond.of(90))
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(225))
                .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(750));

        SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs();
        softwareLimitSwitchConfigs.ForwardSoftLimitEnable = true;
        softwareLimitSwitchConfigs.ForwardSoftLimitThreshold = ElevatorPosition.PEAK.rotations;
        softwareLimitSwitchConfigs.ReverseSoftLimitEnable = true;
        softwareLimitSwitchConfigs.ReverseSoftLimitThreshold = ElevatorPosition.HOME.rotations;

        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        currentLimitsConfigs.SupplyCurrentLimit = 50;

        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

        VoltageConfigs voltageConfigs = new VoltageConfigs();
        voltageConfigs.PeakReverseVoltage = -8;

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.SoftwareLimitSwitch = softwareLimitSwitchConfigs;
        talonFXConfiguration.Slot0 = positionPIDConfigs;
        talonFXConfiguration.MotionMagic = motionMagicConfigs;
        talonFXConfiguration.CurrentLimits = currentLimitsConfigs;
        talonFXConfiguration.MotorOutput = motorOutputConfigs;
        talonFXConfiguration.Voltage = voltageConfigs;

        leftMotor.getConfigurator().apply(talonFXConfiguration);
        rightMotor.getConfigurator().apply(talonFXConfiguration);

        BaseStatusSignal.setUpdateFrequencyForAll(200,
                leftMotor.getPosition(),
                rightMotor.getPosition(),
                leftMotor.getVelocity(),
                rightMotor.getVelocity());

        HealthMonitor.getInstance()
                .addComponent(getName(), "Left motor", leftMotor)
                .addComponent(getName(), "Right motor", rightMotor);

        cmdHandleSensorTransition().schedule();
    }

    @Logged(name = "Right rotations")
    public double getRightMotorPosition() {
        return rightMotor.getPosition().getValueAsDouble();
    }

    @Logged(name = "Left rotations")
    public double getLeftMotorPosition() {
        return leftMotor.getPosition().getValueAsDouble();
    }

    @Logged(name = "Left velocity")
    public double getLeftMotorVelocity() {
        return leftMotor.getVelocity().getValueAsDouble();
    }

    @Logged(name = "Right velocity")
    public double getRightMotorVelocity() {
        return rightMotor.getVelocity().getValueAsDouble();
    }

    @Logged(name = "Home found")
    public boolean getHomeFound() {
        return homeFound;
    }

    @Logged(name = "At home")
    public boolean isAtHome() {
        return !homeSensor.get();
    }

    public boolean isAbovePosition(ElevatorPosition position) {
        var pos = position.rotations - 2; // Apply some buffer
        return pos <= getLeftMotorPosition() || pos <= getRightMotorPosition();
    }

    public void setPosition(double position) {
        motionMagicPostionControl.Position = position;
        leftMotor.setControl(motionMagicPostionControl);
        rightMotor.setControl(motionMagicPostionControl);
    }

    public void resetMotorPositionToPosition(double rotations) {
        leftMotor.setPosition(rotations);
        rightMotor.setPosition(rotations);
    }

    public boolean isNearPositionAndTolerance(double position, double tolerance) {
        return MathUtil.isNear(position, getLeftMotorPosition(), tolerance)
                || MathUtil.isNear(position, getRightMotorPosition(), tolerance);
    }

    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    @Override
    public void periodic() {

    }

    private Command cmdHandleSensorTransition() {
        return new NotifierCommand(() -> detectSensorTransition(), 0.01)
                .until(() -> homeFound)
                .withName("Elevator: Detect home")
                .ignoringDisable(true);
    }

    private void detectSensorTransition() {
        var isAtHome = isAtHome();
        if (previousHomeSensor != isAtHome) {
            homeFound = true;
            resetMotorPositionToPosition(ElevatorPosition.SENSOR.rotations);
        }
        previousHomeSensor = isAtHome;
    }
}
