package frc.robot.util;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;

public class PigeonHealthChecker extends PhoenixHealthChecker {

    private final Pigeon2 pigeon;

    public PigeonHealthChecker(Pigeon2 pigeon, String subsystemName) {
        super(pigeon, subsystemName);
        this.pigeon = pigeon;
    }

    @Override
    protected List<Boolean> faultsToBooleanList() {
        return Arrays.asList(
                pigeon.getFault_BootDuringEnable().getValue(),
                pigeon.getFault_BootupGyroscope().getValue(),
                pigeon.getFault_DataAcquiredLate().getValue(),
                // pigeon.getStickyFault_Hardware().getValue(),
                pigeon.getFault_Undervoltage().getValue());
    }

}
