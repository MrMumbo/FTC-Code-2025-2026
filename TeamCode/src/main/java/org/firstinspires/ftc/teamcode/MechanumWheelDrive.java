package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.HwChasis;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name="MovingRobot Teleop", group="Robot")
public class MechanumWheelDrive extends OpMode {

    private AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private HwChasis hwchasis = new HwChasis();
    double shootTimer = 0;

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
        hwchasis.init(hardwareMap);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        float horizantalMovement = -gamepad1.left_stick_x;
        float verticalMovement   =  gamepad1.left_stick_y;
        float rotationalMovement =  gamepad1.right_stick_x;
        float xButton            =  gamepad1.x ? 1.0f : 0.0f;
        float maxPower           = 0.5f + (xButton / 2.0f);
        double maxShoot           = aprilTagWebcam.distanceToAprilTag / 10;
        float loadBall           = gamepad1.right_bumper ? 1.0f : 0.0f;
        float shootBall          = gamepad1.right_trigger;

        // drive system (strafe, turn, forward/backward)
        hwchasis.backLeft.setPower( (-verticalMovement + horizantalMovement - rotationalMovement) * maxPower);
        hwchasis.frontLeft.setPower((-verticalMovement - horizantalMovement + rotationalMovement) * maxPower);
        hwchasis.backRight.setPower((-verticalMovement - horizantalMovement - rotationalMovement) * maxPower);
        hwchasis.frontRight.setPower( (verticalMovement - horizantalMovement - rotationalMovement) * maxPower);

        // shoot motors
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
            aprilTagWebcam.displayDetectionTelemetry(tag);

            telemetry.addData("Tag ID", tag.id);
            telemetry.addData("Tag range (in)", tag.ftcPose.range);
        } else {
            telemetry.addLine("No tags detected");
        }

        telemetry.addData("horizantalMovement",  "%.2f", horizantalMovement);
        telemetry.addData("verticalMovement",    "%.2f", verticalMovement);
        telemetry.addData("rotationalMovement",  "%.2f", rotationalMovement);
        telemetry.addData("speed",               "%.2f", maxPower);
        telemetry.addData("ball load",           "%.2f", loadBall);
        telemetry.addData("ball shoot",          "%.2f", shootBall);
        telemetry.addData("AprilTagDistance %SHOOT",          "%.2f", maxShoot);

        telemetry.update();
    }

    @Override
    public void stop() {
        aprilTagWebcam.stop();
    }
}
