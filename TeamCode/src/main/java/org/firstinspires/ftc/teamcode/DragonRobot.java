package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/**
 * How to connect to control hub wirelessly:
 *  adb kill-server
 *  adb tcpip 5555
 *  adb connect 192.168.43.1
 */
public abstract class DragonRobot extends LinearOpMode {
    public static final int FIELD_LENGTH_IN_MILLIMETER = 3590;
    public static final int SHIPPING_HUB_RADIUS_IN_MILLIMETER = 226;
    public static final int CAROUSAL_RADIUS_IN_MILLIMETER = 191;
    public static final int RABBIT_POSITION_MIN_IN_MILLIMETER = 700;
    public static final int RABBIT_POSITION_MAX_IN_MILLIMETER = 800;
    public static final int HALF_CHASSIS_WIDTH_IN_MILLIMETER = 114;
    public static final int HALF_CHASSIS_LENGTH_IN_MILLIMETER = 219;
    public static final int BLUE_SHIPPING_HUB_X_IN_MILLIMETER = 1187;
    public static final int BLUE_SHIPPING_HUB_Y_IN_MILLIMETER = 1543;
    public static final int RED_SHIPPING_HUB_X_IN_MILLIMETER = 2375;
    public static final int RED_SHIPPING_HUB_Y_IN_MILLIMETER = 1543;
    public static final int STORAGE_START_X_IN_MILLIMETER = 630;
    public static final int STORAGE_START_Y_IN_MILLIMETER = 0;
    public static final int STORAGE_END_X_IN_MILLIMETER = 1150;
    public static final int STORAGE_END_Y_IN_MILLIMETER = 630;
    public static final int STORAGE_CENTER_X_IN_MILLIMETER = 890;
    public static final int DISTANCE_FROM_CAROUSAL_SPINNER_TO_RIGHT_DISTANCE_SENSOR = 100;
    public static final int DISTANCE_FROM_CAROUSAL_SPINNER_TO_FRONT_DISTANCE_SENSOR = 100;
    public static final double ANGLE_RATIO = 1.4;
    public static final double SLIDE_RATIO = 1.2;
    public static final double CHASSIS_RADIUS = 203;
    public static final double COUNTS_PER_ENCODER_REV = 28;
    public static final double WHEEL_GEAR_RATIO = 19.203208556149733;
    public static final double ARM_GEAR_RATIO = 139.13824192336588;
    public static final double WHEEL_DIAMETER_MILLIMETTER = 96;
    public static final int WHEEL_MOTOR_SPPED_IN_RPM = 312;
    public static final int ARM_MOTOR_SPPED_IN_RPM = 43;
    public static final double WHEEL_COUNTS_PER_DEGREE = COUNTS_PER_ENCODER_REV * WHEEL_GEAR_RATIO / 360;
    public static final double WHEEL_FULL_SPEED_IN_COUNTS = COUNTS_PER_ENCODER_REV * WHEEL_GEAR_RATIO * WHEEL_MOTOR_SPPED_IN_RPM;
    public static final double ARM_COUNTS_PER_DEGREE = COUNTS_PER_ENCODER_REV * ARM_GEAR_RATIO / 360;
    public static final double ARM_FULL_SPEED_IN_COUNTS = COUNTS_PER_ENCODER_REV * ARM_GEAR_RATIO * ARM_MOTOR_SPPED_IN_RPM;
    public static final double COUNTS_PER_MILLIMETER = (COUNTS_PER_ENCODER_REV * WHEEL_GEAR_RATIO) / (WHEEL_DIAMETER_MILLIMETTER * Math.PI);

    public DcMotorEx motorFrontLeft;
    public DcMotorEx motorFrontRight;
    public DcMotorEx motorBackLeft;
    // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
    // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
    // and named "imu".
    public BNO055IMU gyro;
    public DcMotorEx motorBackRight;
    public DcMotorEx motorArm;
    public DcMotorEx motorAcuator;

    public DcMotorEx motorCarousel;
    public ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public Servo servoClaw;
    public Servo servoGrabber;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    public void spinCarousel(double power, int time) throws InterruptedException {
        motorCarousel.setPower(power);
        sleep(time);
    }

    public void stopCarousel() throws InterruptedException {
        motorCarousel.setPower(0);
    }

    public void setDrivingMotorMode(DcMotor.RunMode mode) {
        motorFrontRight.setMode(mode);
        motorFrontLeft.setMode(mode);
        motorBackRight.setMode(mode);
        motorBackLeft.setMode(mode);
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    public boolean isStillDriving() {
        return motorFrontLeft.isBusy() || motorFrontRight.isBusy() || motorBackLeft.isBusy() || motorBackRight.isBusy();
    }

    public void slide(int distanceInMillimeter, double speed) {
        int direction = 1;
        int distanceInCounts = (int) (distanceInMillimeter * COUNTS_PER_MILLIMETER);
        this.setDrivingMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorFrontRight.setTargetPosition(distanceInCounts);
        this.motorFrontLeft.setTargetPosition(distanceInCounts);
        this.motorBackRight.setTargetPosition(-distanceInCounts);
        this.motorBackLeft.setTargetPosition(-distanceInCounts);
        this.setDrivingMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motorFrontRight.setVelocity(WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
        this.motorFrontLeft.setVelocity(WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
        this.motorBackRight.setVelocity(-WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
        this.motorBackLeft.setVelocity(-WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
        while (this.isStillDriving()) {
            // this.telemetry.addData("Front right motor speed", this.motorFrontRight.getVelocity());
            // this.telemetry.addData("Front left motor speed", this.motorFrontLeft.getVelocity());
            // this.telemetry.addData("Back right speed", this.motorBackRight.getVelocity());
            // this.telemetry.addData("Back left speed", this.motorBackLeft.getVelocity());
            // this.telemetry.update();
        }
    }

    protected void driveDistance(int distanceInMilliMeter, double speed) {
        int direction = distanceInMilliMeter / Math.abs(distanceInMilliMeter);
        int distanceInCounts = (int) (distanceInMilliMeter * COUNTS_PER_MILLIMETER);
        setDrivingMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setTargetPosition(distanceInCounts);
        motorFrontLeft.setTargetPosition(-distanceInCounts);
        motorBackRight.setTargetPosition(distanceInCounts);
        motorBackLeft.setTargetPosition(-distanceInCounts);
        setDrivingMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setVelocity(WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
        motorFrontLeft.setVelocity(-WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
        motorBackRight.setVelocity(WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
        motorBackLeft.setVelocity(-WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
        while (isStillDriving()) {
            // telemetry.addData("Front right motor speed", motorFrontRight.getVelocity());
            // telemetry.addData("Front left motor speed", motorFrontLeft.getVelocity());
            // telemetry.addData("Back right speed", motorBackRight.getVelocity());
            // telemetry.addData("Back left speed", motorBackLeft.getVelocity());
            // telemetry.update();
        }
    }

    public void turn(double angle, double speed) {
        if (angle == 0.0) return;
        int direction = ((int) (100 * angle)) / Math.abs((int) (100 * angle));

        double distanceInMilliMeter = Math.toRadians(angle) * CHASSIS_RADIUS;
        int distanceInCounts = (int) (distanceInMilliMeter * COUNTS_PER_MILLIMETER);
        setDrivingMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setTargetPosition(distanceInCounts);
        motorFrontLeft.setTargetPosition(distanceInCounts);
        motorBackRight.setTargetPosition(distanceInCounts);
        motorBackLeft.setTargetPosition(distanceInCounts);
        setDrivingMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setVelocity(WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
        motorFrontLeft.setVelocity(-WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
        motorBackRight.setVelocity(WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
        motorBackLeft.setVelocity(-WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
        while (isStillDriving()) {
            // telemetry.addData("Front right motor speed", motorFrontRight.getVelocity());
            // telemetry.addData("Front left motor speed", motorFrontLeft.getVelocity());
            // telemetry.addData("Back right speed", motorBackRight.getVelocity());
            // telemetry.addData("Back left speed", motorBackLeft.getVelocity());
            // telemetry.update();
        }
    }

    public void raiseArm(double angle, double speed) {
        this.motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorArm.setTargetPosition((int) (-ARM_COUNTS_PER_DEGREE * angle));
        this.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motorArm.setVelocity(-ARM_FULL_SPEED_IN_COUNTS * speed);

        while (this.motorArm.isBusy()) {
            this.telemetry.addData("Arm Speed", this.motorArm.getVelocity());
            this.telemetry.update();
        }
        boolean droppedFreight = false;
        while (!droppedFreight) {
            servoGrabber.setPosition(0.32);
            sleep(200);
            servoGrabber.setPosition(0);
            break;
        }
    }

    public void parkArm(double angle, double speed) {
        this.motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorArm.setTargetPosition((int) (ARM_COUNTS_PER_DEGREE * angle));
        this.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motorArm.setVelocity(ARM_FULL_SPEED_IN_COUNTS * speed);
        // while(this.motorArm.isBusy()){
        //     // this.telemetry.addData("Arm Speed", this.motorArm.getVelocity());
        //     // this.telemetry.update();
        // }
    }

    public double roundAngle(double angle) {
        if (angle > 180) {
            return angle - 360;
        } else if (angle < -180) {
            return 360 + angle;
        } else {
            return angle;
        }
    }

    /**
     * Calculate the angle from robot to target position, counterclockwise is positive angle from IMU, clockwise is negative angle from IMU.
     * Turn right
     *
     * @param angleChassis
     * @param xChassis
     * @param yChassis
     * @param xTarget
     * @param yTarget
     * @return the angle robot need to turn, turn right is negative, turn left is positive.
     */
    public double calcualteAngle(double angleChassis, double xChassis, double yChassis, double xTarget, double yTarget) {
        double xDelta = xTarget - xChassis;
        double yDelta = yTarget - yChassis;
        double angle = 180 * Math.atan(yDelta / xDelta) / Math.PI;
        if (xDelta > 0) {
            if (yDelta > 0) {
                return roundAngle(angle - angleChassis);
            } else {
                return roundAngle(angle - angleChassis);
            }
        } else {
            if (yDelta > 0) {
                return roundAngle(angleChassis - (180 - angle));
            } else {
                return roundAngle(angleChassis - (angle - 180));
            }
        }
    }


    public double calculate_distance(double x1, double y1, double x2, double y2) {
        return Math.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    }

    public enum RobotStartPosition {
        RED_RABBIT,
        RED_WAREHOUSE,
        BLUE_RABBIT,
        BLUE_WAREHOUSE
    }

    public void setup(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        gyro.initialize(parameters);
        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        motorBackLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        motorBackRight = hardwareMap.get(DcMotorEx.class, "backRight");
//        motorCarousel = hardwareMap.get(DcMotorEx.class, "carousel");
//        motorArm = hardwareMap.get(DcMotorEx.class, "arm");
//        motorAcuator = hardwareMap.get(DcMotorEx.class, "acuator");


        // set motor mode
//        setDrivingMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
//        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        // Initialize carousel motor
//        motorCarousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motorCarousel.setPower(0);
//        servoClaw.setPosition(0.5);
//        servoGrabber.setPosition(0);
    }
}
