package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Drive extends Subsystem {

    private VictorSP frontLeft, rearLeft, frontRight, rearRight;
    private SpeedControllerGroup left, right;
    private DifferentialDrive drive;

    private ADXRS450_Gyro gyro;
    private boolean gyroEnabled;

    private Encoder leftEncoder, rightEncoder;
    private boolean encodersEnabled;

    public Drive() {
        frontLeft = new VictorSP(RobotMap.FRONT_LEFT_MOTOR_CHANNEL);
        rearLeft = new VictorSP(RobotMap.REAR_LEFT_MOTOR_CANNEL);
        frontRight = new VictorSP(RobotMap.FRONT_RIGHT_MOTOR_CHANNEL);
        rearRight = new VictorSP(RobotMap.REAR_RIGHT_MOTOR_CHANNEL);
        left = new SpeedControllerGroup(frontLeft, rearLeft);
        right = new SpeedControllerGroup(frontRight, rearRight);
        drive = new DifferentialDrive(left, right);
        drive.setMaxOutput(Constants.MAX_DRIVE_OUTPUT);

        try {
            gyro = new ADXRS450_Gyro();
            gyro.calibrate();
            gyroEnabled = true;
            System.out.println("Gyro initialized");
        } catch (NullPointerException e) {
            gyroEnabled = false;
            System.out.println("GYRO NOT ENABLED");
        }

        try {
            leftEncoder = new Encoder(RobotMap.LEFT_ENCODER_CHANNEL_A, RobotMap.LEFT_ENCODER_CHANNEL_B, true);
            rightEncoder = new Encoder(RobotMap.RIGHT_ENCODER_CHANNEL_A, RobotMap.RIGHT_ENCODER_CHANNEL_B, false);
            leftEncoder.setDistancePerPulse(Constants.DISTANCE_PER_ENCODER_PULSE);
            rightEncoder.setDistancePerPulse(Constants.DISTANCE_PER_ENCODER_PULSE);
            encodersEnabled = true;
            System.out.println("Encoders initialized");
        } catch (NullPointerException e) {
            encodersEnabled = false;
            System.out.println("ENCODERS NOT ENABLED");
        }
    }

    public void drive(double speedInput, double rotationInput) {

        double speed = speedInput;
        double rotation = Math.abs(rotationInput) * rotationInput * Constants.MAX_TURN_SPEED;
        drive.curvatureDrive(speed, rotation, true);
    }

    public void setLeftRightMotorSpeeds(double leftPower, double rightPower) {
        left.set(leftPower);
        right.set(rightPower);
    }

    public boolean sensorsEnabled() {
        return gyroEnabled && encodersEnabled;
    }

    public double getHeading() {
        return gyro.getAngle();
    }

    public double getLeftDistance() {
        return leftEncoder.getDistance();
    }

    public double getRightDistance() {
        return rightEncoder.getDistance();
    }

    public void reset() {
        gyro.reset();
        leftEncoder.reset();
        rightEncoder.reset();
    }

    @Override
    public void initDefaultCommand() {
    }
}