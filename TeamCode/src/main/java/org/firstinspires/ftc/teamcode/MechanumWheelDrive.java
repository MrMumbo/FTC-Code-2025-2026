package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.HwChasis;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name="MovingRobot Teleop", group="Robot")
public class MechanumWheelDrive extends OpMode {

    private AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private HwChasis hwchasis = new HwChasis();
    double shootTimer = 0;

    public double nY;
    public double nX;
    IMU imu;
    boolean relativeToggle = true;
    public float selectedAprilTag;
    public boolean selectAT = false;

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
        hwchasis.init(hardwareMap);

        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        if(gamepad1.dpadUpWasPressed()){
            selectAT = !selectAT;
        }

        if (aprilTagWebcam.aprilTagID != 0) {
            if(selectAT && aprilTagWebcam.aprilTagID == 20){
                selectedAprilTag = 20;
            }
            if(!selectAT && aprilTagWebcam.aprilTagID == 24){
                selectedAprilTag = 24;
            }
        } else {
            selectedAprilTag = 0;
        }

        float horizantalMovement = gamepad1.left_stick_x;
        float verticalMovement   =  -gamepad1.left_stick_y;
        float rotationalMovement =  gamepad1.right_stick_x;
        float xButton            =  gamepad1.x ? 1.0f : 0.0f;
        float maxPower           = 1f + (xButton / 2.0f);
        double maxShoot           = aprilTagWebcam.distanceToAprilTag / 100;
        float loadBall           = gamepad1.right_bumper ? 1.0f : 0.0f;
        float shootBall          = gamepad1.right_trigger;

        if (gamepad1.a) {
            imu.resetYaw();
        }

        if(gamepad1.dpadDownWasPressed()) {relativeToggle = !relativeToggle;}

        driveFieldRelative(horizantalMovement, verticalMovement);

        // drive system (strafe, turn, forward/backward)
        if(!relativeToggle){
            hwchasis.backLeft.setPower( (-verticalMovement + horizantalMovement - rotationalMovement) * maxPower);
            hwchasis.frontLeft.setPower((-verticalMovement - horizantalMovement - rotationalMovement) * maxPower);
            hwchasis.backRight.setPower((-verticalMovement - horizantalMovement + rotationalMovement) * maxPower);
            hwchasis.frontRight.setPower( (verticalMovement - horizantalMovement - rotationalMovement) * maxPower);
        } else {
            hwchasis.backLeft.setPower( (-nY - nX - rotationalMovement) * maxPower);
            hwchasis.frontLeft.setPower((-nY + nX - rotationalMovement) * maxPower);
            hwchasis.backRight.setPower((-nY + nX + rotationalMovement) * maxPower);
            hwchasis.frontRight.setPower( (nY + nX - rotationalMovement) * maxPower);
        }

        // shoot motors
        //TEMP
        maxShoot = 2.0f;
        //TEMP
        hwchasis.shootPower.setPower(shootBall * maxShoot);

        hwchasis.holdLeft.setPosition(loadBall);
        hwchasis.holdRight.setPosition(loadBall);

        // braking movement system
        if (horizantalMovement == 0.0f && verticalMovement == 0.0f && rotationalMovement == 0.0f) {
            hwchasis.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hwchasis.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hwchasis.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hwchasis.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // stop flywheel
        if (shootBall == 0.0f) {
            hwchasis.shootPower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        //apriltag
        aprilTagWebcam.update();

        java.util.List<AprilTagDetection> detections = aprilTagWebcam.getDetectedTags();
        telemetry.addData("Tag count", detections.size());

        if (!detections.isEmpty()) {
            AprilTagDetection tag = detections.get(0);
            if (tag.id == 24 || tag.id == 20){
                if (selectedAprilTag == 24 || selectedAprilTag == 20){
                    aprilTagWebcam.displayDetectionTelemetry(tag);
                }
            }

            telemetry.addData("Tag ID", tag.id);
            telemetry.addData("Tag range (in)", tag.ftcPose.range);
        } else {
            telemetry.addLine("No tags detected");
        }

        telemetry.addData("speed",               "%.2f", maxPower);
        telemetry.addData("ball load",           "%.2f", loadBall);
        telemetry.addData("ball shoot",          "%.2f", shootBall);
        telemetry.addData("rotation to AT",          "%.2f", aprilTagWebcam.rotationToAprilTag);
        telemetry.addData("distance to AT (cm)",          "%.2f", aprilTagWebcam.distanceToAprilTag);
        telemetry.addData("motorSpeed through AT transitor",          "%.2f", maxShoot);
        telemetry.addData("Selected AT", "%.2f", selectedAprilTag);

        if(relativeToggle){
            telemetry.addLine("Using Field Relative");
        } else {
            telemetry.addLine("Using Robot Relative");
        }
        if(selectAT){
            telemetry.addLine("Selecting Blue Goal");
        } else {
            telemetry.addLine("Selecting Red Goal");
        }

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
