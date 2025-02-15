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
                pigeon.getStickyFault_BootDuringEnable().getValue(),
                pigeon.getStickyFault_BootupGyroscope().getValue(),
                pigeon.getStickyFault_DataAcquiredLate().getValue(),
                pigeon.getStickyFault_Hardware().getValue(),
                pigeon.getStickyFault_Undervoltage().getValue());
    }

}
