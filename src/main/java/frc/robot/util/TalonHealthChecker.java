// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class TalonHealthChecker {

    private final TalonFX talon;
    private final Alert alert;
    private final Timer timer = new Timer();
    private double minSecondsBetweenCheckups = 2.5;

    public TalonHealthChecker(TalonFX talon, String subsystemName) {
        this.talon = talon;
        alert = new Alert(String.format("[%s] CAN ID %d, check engine.", subsystemName, talon.getDeviceID()),
                AlertType.kError);
        timer.start();
    }

    public TalonHealthChecker(TalonFX talon, String subsystemName, double minSecondsBetweenCheckups) {
        this(talon, subsystemName);
        this.minSecondsBetweenCheckups = minSecondsBetweenCheckups;
    }

    public void checkUp() {
        if (timer.hasElapsed(minSecondsBetweenCheckups) && DriverStation.isDisabled()) {
            alert.set(!isDeviceHealthy());
            timer.reset();
        }
    }

    private boolean isDeviceHealthy() {
        if (!talon.isConnected())
            return false;

        if (areBooleanStatusSignalsHealthy(faultsToBooleanList(talon)))
            return false;

        return true;
    }

    private boolean areBooleanStatusSignalsHealthy(List<Boolean> statusSignalValues) {
        for (boolean statusSignalValue : statusSignalValues) {
            if (statusSignalValue) {
                return false;
            }
        }
        return true;
    }

    private List<Boolean> faultsToBooleanList(TalonFX talon) {
        return Arrays.asList(talon.getStickyFault_BootDuringEnable().getValue(),
                talon.getStickyFault_BridgeBrownout().getValue(),
                talon.getStickyFault_Hardware().getValue(),
                talon.getStickyFault_OverSupplyV().getValue(),
                talon.getStickyFault_RemoteSensorDataInvalid().getValue(),
                talon.getStickyFault_RemoteSensorReset().getValue(),
                talon.getStickyFault_Undervoltage().getValue(),
                talon.getStickyFault_UnstableSupplyV().getValue());
    }
}
