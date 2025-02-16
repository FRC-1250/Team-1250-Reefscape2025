package frc.robot.util;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;

public class SwerveModuleHealthChecker {

    private TalonHealthChecker driveCheck;
    private TalonHealthChecker steerCheck;
    private CANcoderHealthChecker encoderCheck;
    private final String subsystemName = "Drivetrain";

    public SwerveModuleHealthChecker(SwerveModule<TalonFX, TalonFX, CANcoder> module) {
        this.driveCheck = new TalonHealthChecker(module.getDriveMotor(), subsystemName);
        this.steerCheck = new TalonHealthChecker(module.getSteerMotor(), subsystemName);
        this.encoderCheck = new CANcoderHealthChecker(module.getEncoder(), subsystemName);
    }

    public boolean isModuleHealthy() {
        return driveCheck.isDeviceHealthy() &&
                steerCheck.isDeviceHealthy() &&
                encoderCheck.isDeviceHealthy();
    }
}
