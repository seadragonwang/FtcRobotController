package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "DragonTele", group = "Dragon")
public class DragonTele extends DragonRobot {
    @Override
    public void runOpMode() throws InterruptedException {
        setup();
//        this.motorArm.setPower(1);
//        this.motorAcuator.setPower(1);

        waitForStart();
        if (isStopRequested()) return;

        boolean runSlowMode = false;
        while (opModeIsActive()) {
            double y = gamepad1.left_stick_x;
            double x = gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;
            double radius = Math.sqrt(x * x + y * y);
            double sinTheta = -x / radius;
            double cosTheta = -y / radius;
            double frontLeftPower = (cosTheta - sinTheta);
            double backLeftPower = (cosTheta + sinTheta);
            double frontRightPower = (cosTheta + sinTheta);
            double backRightPower = (cosTheta - sinTheta);
            if (gamepad1.a) {
                runSlowMode = true;
            } else if (gamepad1.b) {
                runSlowMode = false;
            }
            if (runSlowMode) {
                if (x != 0 || y != 0) {
                    motorFrontLeft.setPower(frontLeftPower / 2);
                    motorBackLeft.setPower(-backLeftPower / 2);
                    motorFrontRight.setPower(frontRightPower / 2);
                    motorBackRight.setPower(-backRightPower / 2);
                } else {
                    motorFrontLeft.setPower(-rx / 2);
                    motorBackLeft.setPower(-rx / 2);
                    motorFrontRight.setPower(-rx / 2);
                    motorBackRight.setPower(-rx / 2);
                }
            } else {
                if (x != 0 || y != 0) {
                    motorFrontLeft.setPower(frontLeftPower);
                    motorBackLeft.setPower(-backLeftPower);
                    motorFrontRight.setPower(frontRightPower);
                    motorBackRight.setPower(-backRightPower);
                } else {
                    motorFrontLeft.setPower(-rx);
                    motorBackLeft.setPower(-rx);
                    motorFrontRight.setPower(-rx);
                    motorBackRight.setPower(-rx);
                }
            }
            telemetry.addLine().addData("angle", this.gyro.getAngularOrientation().firstAngle);
            telemetry.update();
//            this.motorAcuator.setTargetPosition(currentAcuatorPosition);
//            this.motorAcuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
}
