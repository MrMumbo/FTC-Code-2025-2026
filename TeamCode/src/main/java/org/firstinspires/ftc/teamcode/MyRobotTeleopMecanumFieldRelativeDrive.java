package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * Simple ROBOT-relative mecanum teleop with shooter, intake, and servos.
 * No field-relative drive, no IMU.
 */
@TeleOp(name = "Test : Do NOT Touch", group = "Test")
public class MyRobotTeleopMecanumFieldRelativeDrive extends OpMode {

    // Drive motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Mechanisms
    private DcMotorEx shootMotor;
    private DcMotorEx intakeMotor;
    private Servo leftServo;
    private Servo rightServo;
    IMU imu;

    private float nX;
    private float nY;

    @Override
    public void init() {
        // Map hardware (names must match your RC configuration)
        frontLeft   = hardwareMap.get(DcMotor.class,   "frontLeft");
        frontRight  = hardwareMap.get(DcMotor.class,   "frontRight");
        backLeft    = hardwareMap.get(DcMotor.class,   "backLeft");
        backRight   = hardwareMap.get(DcMotor.class,   "backRight");
        shootMotor  = hardwareMap.get(DcMotorEx.class, "shootMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        leftServo   = hardwareMap.get(Servo.class,     "leftServo");
        rightServo  = hardwareMap.get(Servo.class,     "rightServo");

        // Typical mecanum: left side reversed, right side forward
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Use encoders on shooter / intake
        shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.addLine("Robot-relative mecanum teleop init complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        // -------------------------
        // CONTROLS
        // -------------------------
        // Left stick Y = forward/back  (up = forward)
        // Left stick X = strafe        (right = strafe right)
        // Right stick X = rotate       (right = turn clockwise)
        double y  = -gamepad1.left_stick_y;  // forward/back
        double x  =  -gamepad1.left_stick_x;  // strafe
        double rx =  gamepad1.right_stick_x; // rotate

        if (gamepad1.triangle) {
            imu.resetYaw();
        }

        driveFieldRelative((float) y, (float) x);

        double frontLeftPower  = (rx + nY - nX);
        double backLeftPower   = (nX + nY + rx);
        double frontRightPower = (-nX - nY + rx);
        double backRightPower  = (nX - nY + rx);

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

        // -------------------------
        // SHOOTER / INTAKE / SERVOS
        // -------------------------
        // Shooter on right trigger, capped at ~1200 ticks/sec
        if (gamepad1.right_trigger > 0.0) {
            if (shootMotor.getVelocity() < 1200) {
                shootMotor.setPower(-gamepad1.right_trigger);
            } else {
                shootMotor.setPower(0);
            }
        } else {
            shootMotor.setPower(0);
        }

        // Intake on left trigger (reverse sign if needed)
        intakeMotor.setPower(-gamepad1.left_trigger);

        // Servos toggle with right bumper
        leftServo.setPosition(gamepad1.right_bumper ? 0.0 : 1.0);
        rightServo.setPosition(gamepad1.right_bumper ? 1.0 : 0.0);

        // -------------------------
        // TELEMETRY
        // -------------------------
        telemetry.addData("Drive", "y=%.2f x=%.2f rx=%.2f", y, x, rx);
        telemetry.addData("FL", "%.2f", frontLeftPower);
        telemetry.addData("FR", "%.2f", frontRightPower);
        telemetry.addData("BL", "%.2f", backLeftPower);
        telemetry.addData("BR", "%.2f", backRightPower);
        telemetry.addData("Shooter velocity", "%.0f", shootMotor.getVelocity());
        telemetry.addData("IMU:", "%.2f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();
    }

    private void driveFieldRelative(float forward, float right) {
        float theta;
        float thetaRobot;
        float thetaDriver;
        float magnitude = (float) Math.hypot(right, forward);

        thetaDriver = (float) (Math.atan2(right, forward));
        thetaRobot = (float) imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        theta = thetaDriver - thetaRobot;

        nY = (float) (Math.cos(theta) * magnitude);
        nX = (float) (Math.sin(theta) * magnitude);
    }
}
