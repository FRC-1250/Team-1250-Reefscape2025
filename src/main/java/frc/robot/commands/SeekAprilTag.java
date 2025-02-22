package frc.robot.commands;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest.NativeSwerveRequest;
import com.ctre.phoenix6.swerve.jni.SwerveJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.LimelightHelpers;

public class SeekAprilTag implements NativeSwerveRequest {
    // Default Robot Centric drive variables
    private double VelocityX = 0;
    private double VelocityY = 0;
    private double RotationalRate = 0;
    private double Deadband = 0;
    private double RotationalDeadband = 0;
    private Translation2d CenterOfRotation = new Translation2d();
    private SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
    private SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.Position;
    private boolean DesaturateWheelSpeeds = true;

    // Custom attributes
    private Pose2d robotPose2d = new Pose2d();
    private double targetAngle = 0;
    private double aprilTagID = -1;
    private Pose3d aptilTagPose3d = new Pose3d();
    private PIDController headingController = new PIDController(5, 0, 0);
    private PIDController yController = new PIDController(0.3, 0, 0.00);
    private PIDController xController = new PIDController(0.64, 0, 0.00);
    private double yFF = 0.1;
    private double xFF = 0.5;
    private boolean userDriveForward = false;

    public SeekAprilTag() {
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        headingController.setTolerance(0.05);
        xController.setTolerance(0.01);
        yController.setTolerance(0.01);
    }

    public SeekAprilTag withDriveRequestType(SwerveModule.DriveRequestType newDriveRequestType) {
        this.DriveRequestType = newDriveRequestType;
        return this;
    }

    public SeekAprilTag withSteerRequestType(SwerveModule.SteerRequestType newSteerRequestType) {
        this.SteerRequestType = newSteerRequestType;
        return this;
    }

    public SeekAprilTag withRobotPose(Pose2d pose) {
        this.robotPose2d = pose;
        return this;
    }

    public SeekAprilTag withVelocityX(double velocity) {
        this.VelocityX = velocity;
        return this;
    }

    public SeekAprilTag withUserDriveForward(boolean value) {
        this.userDriveForward = value;
        return this;
    }

    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        determineMovementValues();
        return StatusCode.valueOf(SwerveJNI.JNI_Request_Apply_RobotCentric(parameters.drivetrainId,
                VelocityX,
                VelocityY,
                RotationalRate,
                Deadband,
                RotationalDeadband,
                CenterOfRotation.getX(),
                CenterOfRotation.getY(),
                DriveRequestType.value,
                SteerRequestType.value,
                DesaturateWheelSpeeds));
    }

    public void applyNative(int id) {
        determineMovementValues();
        SwerveJNI.JNI_SetControl_RobotCentric(id,
                VelocityX,
                VelocityY,
                RotationalRate,
                Deadband,
                RotationalDeadband,
                CenterOfRotation.getX(),
                CenterOfRotation.getY(),
                DriveRequestType.value,
                SteerRequestType.value,
                DesaturateWheelSpeeds);
    }

    private void determineMovementValues() {
        aprilTagID = LimelightHelpers.getFiducialID("limelight");
        aptilTagPose3d = LimelightHelpers.getTargetPose3d_RobotSpace("limelight");

        if (Double.compare(aprilTagID, -1) == 0 || Double.compare(aprilTagID, 0) == 0) {
            VelocityX = 0;
            VelocityY = 0;
            RotationalRate = 0;
        } else {
            if (Double.compare(aprilTagID, 11) == 0 || Double.compare(aprilTagID, 17) == 0) {
                targetAngle = 60;
            } else if (Double.compare(aprilTagID, 10) == 0 || Double.compare(aprilTagID, 18) == 0) {
                targetAngle = 0;
            } else if (Double.compare(aprilTagID, 9) == 0 || Double.compare(aprilTagID, 19) == 0) {
                targetAngle = -60;
            } else if (Double.compare(aprilTagID, 8) == 0 || Double.compare(aprilTagID, 20) == 0) {
                targetAngle = -120;
            } else if (Double.compare(aprilTagID, 7) == 0 || Double.compare(aprilTagID, 21) == 0) {
                targetAngle = 180;
            } else if (Double.compare(aprilTagID, 6) == 0 || Double.compare(aprilTagID, 22) == 0) {
                targetAngle = 120;
            }

            if (!userDriveForward) {
                if (xController.atSetpoint())
                    VelocityX = 0;
                else
                    VelocityX = -xController.calculate(aptilTagPose3d.getZ(), 0.43) + xFF;
            }

            if (yController.atSetpoint())
                VelocityY = 0;
            else
                VelocityY = yController.calculate(aptilTagPose3d.getX(), 0) + yFF;

            if (headingController.atSetpoint())
                RotationalRate = 0;
            else
                RotationalRate = headingController.calculate(robotPose2d.getRotation().getRadians(),
                        Math.toRadians(targetAngle));
        }
    }
}
