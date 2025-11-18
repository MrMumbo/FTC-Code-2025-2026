package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

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
        double rx  = -gamepad1.left_stick_y;  // forward/back
        double x  =  gamepad1.left_stick_x;  // strafe
        double y =  gamepad1.right_stick_x; // rotate

        // -------------------------
        // DRIVE: ROBOT-RELATIVE MECANUM
        // -------------------------
        // Standard FTC mecanum mixing
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        double frontLeftPower  = (y + x + rx) / denominator;
        double backLeftPower   = (y - x + rx) / denominator;
        double frontRightPower = (y + x - rx) / denominator;
        double backRightPower  = (y - x - rx) / denominator;

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
        telemetry.update();
    }
}
