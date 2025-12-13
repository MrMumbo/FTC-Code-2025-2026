package org.firstinspires.ftc.teamcode.FCSSDemo;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Disabled
@TeleOp(name = "Test : Do NOT TOUCH", group = "Test")
public class MyRobotTeleopMecanumFieldRelativeDrive extends OpMode {
    private AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    double frontLeftPower;
    double backLeftPower;
    double frontRightPower;
    double backRightPower;

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
    private double ATRot;
    private double ATMov;
    private  double ATSel;

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
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
        aprilTagWebcam.update();

        java.util.List<AprilTagDetection> detections = aprilTagWebcam.getDetectedTags();
        telemetry.addData("Tag count", detections.size());

        if (!detections.isEmpty()) {
            AprilTagDetection tag = detections.get(0);
            aprilTagWebcam.displayDetectionTelemetry(tag);

            telemetry.addData("Tag ID", tag.id);
            telemetry.addData("Tag range (in)", tag.ftcPose.range);
            ATRot = tag.ftcPose.bearing;
            ATMov = tag.ftcPose.y;
            ATSel = tag.id;
        } else {
            telemetry.addLine("No tags detected");
            ATRot = 0.0f;
            ATMov = 0.0f;
            ATSel = 0.0f;
        }

        double y  = -gamepad1.left_stick_y;  // forward/back
        double x  =  -gamepad1.left_stick_x;  // strafe
        double rx =  gamepad1.right_stick_x; // rotate
        double aRx; //apriltag rotation

        if (detections.isEmpty()){
            aRx = 0;
        } else {
            if (ATRot > 0){
                aRx = -(ATRot / 30);
            } else{
                aRx = -(ATRot / 30);
            }
        }

        if (gamepad1.triangle) {
            imu.resetYaw();
        }

        driveFieldRelative((float) y, (float) x);

        if(!gamepad1.left_bumper){
            frontLeftPower  = (rx + nY - nX);
            backLeftPower   = (nX + nY + rx);
            frontRightPower = (-nX - nY + rx);
            backRightPower  = (nX - nY + rx);
        } else {
            if(ATSel == 24 ){
                frontLeftPower  = (aRx + nY - nX);
                backLeftPower   = (nX + nY + aRx);
                frontRightPower = (-nX - nY + aRx);
                backRightPower  = (nX - nY + aRx);
            }
        }

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

        //shooter motor capped at 1200 velocity
        if (gamepad1.right_trigger > 0.0) {
            if (shootMotor.getVelocity() < 1350) {
                shootMotor.setPower(-gamepad1.right_trigger);
            } else {
                shootMotor.setPower(0);
            }
        } else {
            shootMotor.setPower(0);
        }

        // Intake on left trigger output on left bumper
        if(gamepad1.left_bumper){
            intakeMotor.setPower(gamepad1.left_bumper ? 1.0 : 0.0);
        } else {
            intakeMotor.setPower(-gamepad1.left_trigger);
        }
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
        telemetry.addData("RotationToAprilTag", "%.2f", ATRot);
        telemetry.addData("AprilTagID", "%.2f", ATSel);
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

    @Override
    public void stop() {
        aprilTagWebcam.stop();
    }
}
