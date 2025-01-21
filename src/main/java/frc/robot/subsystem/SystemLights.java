// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.time.StopWatch;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SystemLights extends SubsystemBase {
  /** Creates a new SystemLights. */
  private final CANdle candle = new CANdle(15, "rio");
  private final int LedCount = 300;

  Timer timer = new Timer();
  StrobeAnimation strobeAnimation = new StrobeAnimation(1, 48, 12);
  LarsonAnimation larsonAnimation = new LarsonAnimation(1, 48, 12);

  public double getVbat() {
    return candle.getBusVoltage();
  }

  public double get5V() {
    return candle.get5VRailVoltage();
  }

  public double getCurrent() {
    return candle.getCurrent();
  }

  public double getTemperature() {
    return candle.getTemperature();
  }

  public void configBrightness(double percent) {
    candle.configBrightnessScalar(percent, 0);
  }

  public void configLos(boolean disableWhenLos) {
    candle.configLOSBehavior(disableWhenLos, 0);
  }

  public void configLedType(LEDStripType type) {
    candle.configLEDType(type, 0);
  }

  public void configStatusLedBehavior(boolean offWhenActive) {
    candle.configStatusLedState(offWhenActive, 0);
  }

  public SystemLights() {
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.disableWhenLOS = false;
    configAll.brightnessScalar = 0.1;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    configAll.v5Enabled = false;
    candle.configAllSettings(configAll, 100);
    timer.start();
    larsonAnimation.setBounceMode(BounceMode.Back);
  }

  public void setLEDs(int r, int g, int b) {
    candle.setLEDs(r, g, b);
  }

   private void cycleLights() {
    if (timer.get() > .5 && timer.get() < 15) {
      setLEDs(255, 215, 0);
    } else if (timer.get() > 16 && timer.get() < 35) {
      candle.animate(larsonAnimation);
    } else if (timer.get() > 50) {
      timer.restart();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
