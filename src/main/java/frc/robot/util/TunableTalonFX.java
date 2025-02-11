package frc.robot.util;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class TunableTalonFX {
    private double currentG = 0;
    private double currentS = 0;
    private double currentV = 0;
    private double currentA = 0;
    private double currentP = 0;
    private double currentI = 0;
    private double currentD = 0;

    private double previousG = 0;
    private double previousS = 0;
    private double previousV = 0;
    private double previousA = 0;
    private double previousP = 0;
    private double previousI = 0;
    private double previousD = 0;

    private final NetworkTable pid;
    private final DoubleEntry networkG;
    private final DoubleEntry networkS;
    private final DoubleEntry networkV;
    private final DoubleEntry networkA;
    private final DoubleEntry networkP;
    private final DoubleEntry networkI;
    private final DoubleEntry networkD;

    private final TalonFX[] talons;
    private final Alert tuningModeAlert;

    private SlotConfigs slotConfigs;

    public TunableTalonFX(String subsystemName, String tuningGroupName, SlotConfigs slotConfigs, TalonFX... talons) {
        tuningModeAlert = new Alert(String.format("%s PID tuning mode active.", subsystemName), AlertType.kInfo);
        tuningModeAlert.set(true);

        pid = NetworkTableInstance.getDefault().getTable(String.format("Robot tuner/%s", tuningGroupName));
        networkG = pid.getDoubleTopic("kG").getEntry(0);
        networkG.set(slotConfigs.kG);
        previousG = slotConfigs.kG;

        networkS = pid.getDoubleTopic("kS").getEntry(0);
        networkS.set(slotConfigs.kS);
        previousS = slotConfigs.kS;

        networkV = pid.getDoubleTopic("kV").getEntry(0);
        networkV.set(slotConfigs.kV);
        previousV = slotConfigs.kV;

        networkA = pid.getDoubleTopic("kA").getEntry(0);
        networkA.set(slotConfigs.kA);
        previousV = slotConfigs.kV;

        networkP = pid.getDoubleTopic("kP").getEntry(0);
        networkP.set(slotConfigs.kP);
        previousP = slotConfigs.kP;

        networkI = pid.getDoubleTopic("kI").getEntry(0);
        networkI.set(slotConfigs.kI);
        previousI = slotConfigs.kI;

        networkD = pid.getDoubleTopic("kD").getEntry(0);
        networkD.set(slotConfigs.kD);
        previousD = slotConfigs.kD;

        this.slotConfigs = slotConfigs;
        this.slotConfigs.SlotNumber = 0;
        for (TalonFX talonFX : talons) {
            talonFX.getConfigurator().apply(slotConfigs);
        }

        this.talons = talons;
    }

    public void updateValuesFromSmartNT() {
        currentG = networkG.get();
        currentS = networkS.get();
        currentV = networkV.get();
        currentA = networkA.get();
        currentP = networkP.get();
        currentI = networkI.get();
        currentD = networkD.get();

        if (currentG != previousG ||
                currentS != previousS ||
                currentV != previousV ||
                currentA != previousA ||
                currentP != previousP ||
                currentI != previousI ||
                currentD != previousD) {
            slotConfigs.kG = currentG;
            slotConfigs.kS = currentS;
            slotConfigs.kV = currentV;
            slotConfigs.kA = currentA;
            slotConfigs.kP = currentP;
            slotConfigs.kI = currentI;
            slotConfigs.kD = currentD;

            for (TalonFX talonFX : talons) {
                talonFX.getConfigurator().apply(slotConfigs);
            }
        }

        previousG = currentG;
        previousS = currentS;
        previousV = currentV;
        previousA = currentA;
        previousP = currentP;
        previousI = currentI;
        previousD = currentD;
    }
}
