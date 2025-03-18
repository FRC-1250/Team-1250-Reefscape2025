package frc.robot.util;

import java.util.List;

import com.ctre.phoenix6.hardware.ParentDevice;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public abstract class PhoenixHealthChecker {
    private final ParentDevice device;
    private final int deviceID;
    public final String subsystemName;
    private final Alert alert;
    private boolean healthy;

    public PhoenixHealthChecker(final ParentDevice device, final String subsystemName) {
        this.device = device;
        this.deviceID = device.getDeviceID();
        this.subsystemName = subsystemName;
        alert = new Alert("", AlertType.kError);
    }

    public boolean isDeviceHealthy() {
        healthy = checkUp();
        alert.set(!healthy);
        return healthy;
    }

    protected abstract List<Boolean> faultsToBooleanList();

    private boolean checkUp() {
        if (!device.isConnected()) {
            setAlertMessage("is device connected? Check for power or CAN bus issues.");
            return false;
        }

        if (!areBooleanStatusSignalsHealthy(faultsToBooleanList())) {
            setAlertMessage("faults detected. Check phoenix tuner.");
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
