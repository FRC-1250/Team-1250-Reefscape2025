package frc.robot.util;

import java.util.List;

import com.ctre.phoenix6.hardware.ParentDevice;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public abstract class PhoenixHealthChecker {
    private final ParentDevice device;
    private final int deviceID;
    private final String subsystemName;
    private final Alert alert;
    private final Timer timer;
    private final double minSecondsBetweenCheckups;
    private boolean healthy;

    public PhoenixHealthChecker(final ParentDevice device, final String subsystemName) {
        this.device = device;
        this.deviceID = device.getDeviceID();
        this.subsystemName = subsystemName;
        alert = new Alert("", AlertType.kError);
        timer = new Timer();
        timer.start();
        minSecondsBetweenCheckups = 2;
    }

    public boolean isDeviceHealthy() {
        if (!DriverStation.isDisabled()) {
            // If DS is enabled, assume it is healthy...
            healthy = true;
        } else {
            // ... until another check can be conducted
            healthy = checkUp();
            if (timer.hasElapsed(minSecondsBetweenCheckups)) {
                alert.set(!healthy);
                timer.reset();
            }
        }
        return healthy;
    }

    protected abstract List<Boolean> faultsToBooleanList();

    private boolean checkUp() {
        if (!device.isConnected()) {
            setAlertMessage("is device connected? Check for power or CAN bus issues.");
            return false;
        }

        if (!areBooleanStatusSignalsHealthy(faultsToBooleanList())) {
            setAlertMessage("sticky faults detected. Check phoenix tuner.");
            return false;
        }
        return true;
    }

    private void setAlertMessage(final String errorString) {
        alert.setText(String.format("[%s] CAN ID %d, %s", subsystemName, deviceID, errorString));
    }

    private boolean areBooleanStatusSignalsHealthy(final List<Boolean> statusSignalValues) {
        for (final boolean statusSignalValue : statusSignalValues) {
            if (statusSignalValue) {
                return false;
            }
        }
        return true;
    }
}
