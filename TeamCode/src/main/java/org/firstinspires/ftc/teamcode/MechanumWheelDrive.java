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

    public double nY;
    public double nX;
    IMU imu;
    boolean relativeToggle = true;
    public boolean selectAT = false;
    public double ATDis = 0.0f;
    public double ATRot = 0.0f;
    public double ATTag = 0.0f;
    public double realRot = 0.0f;
    boolean bToggle = true;

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

        float horizantalMovement = gamepad1.left_stick_x;
        float verticalMovement   =  -gamepad1.left_stick_y;
        double rotationalMovement =  gamepad1.right_stick_x;
        float xButton            =  gamepad1.x ? 1.0f : 0.0f;
        float maxPower           = 1f + (xButton / 2.0f);
        double robDis = Math.sqrt(Math.pow(ATDis, 2)-Math.pow(29.5,2));
        double maxShoot;
        if(ATDis > 165){
            maxShoot = (Math.pow(((robDis+75)*645.7), 0.613));
        }else {
            maxShoot = (Math.pow(((robDis+100)*645.7), 0.613));
        }
        float loadBall           = gamepad1.right_bumper ? 1.0f : 0.0f;
        float shootBall          = gamepad1.right_trigger;
        double rotationalMult = ATRot / 30;
        double backLeft;
        double frontLeft;
        double backRight;
        double frontRight;
        boolean lightEnabled = ATTag != 0;

        if(bToggle){
            if(lightEnabled){
                hwchasis.LED1green.enableLight(true);
                hwchasis.LED2green.enableLight(true);
                hwchasis.LED3green.enableLight(true);
                hwchasis.LED4green.enableLight(true);
                hwchasis.LED1red.enableLight(false);
                hwchasis.LED2red.enableLight(false);
                hwchasis.LED3red.enableLight(false);
                hwchasis.LED4red.enableLight(false);
            } else {
                hwchasis.LED1green.enableLight(false);
                hwchasis.LED2green.enableLight(false);
                hwchasis.LED3green.enableLight(false);
                hwchasis.LED4green.enableLight(false);
                hwchasis.LED1red.enableLight(true);
                hwchasis.LED2red.enableLight(true);
                hwchasis.LED3red.enableLight(true);
                hwchasis.LED4red.enableLight(true);
            }
        } else {
            hwchasis.LED1green.enableLight(false);
            hwchasis.LED2green.enableLight(false);
            hwchasis.LED3green.enableLight(false);
            hwchasis.LED4green.enableLight(false);
            hwchasis.LED1red.enableLight(false);
            hwchasis.LED2red.enableLight(false);
            hwchasis.LED3red.enableLight(false);
            hwchasis.LED4red.enableLight(false);
        }


        if(gamepad1.bWasPressed()){
            bToggle = !bToggle;
        }

        if (gamepad1.a) {
            imu.resetYaw();
        }
        if (bToggle) {
            if (ATTag != 0){
                rotationalMovement = -rotationalMult;
            }
        }

        if(gamepad1.dpadDownWasPressed()) {relativeToggle = !relativeToggle;}

        driveFieldRelative(horizantalMovement, verticalMovement);

        // drive system (strafe, turn, forward/backward)
        if(!relativeToggle){
            backLeft = ((-verticalMovement + horizantalMovement - rotationalMovement) * maxPower);
            frontLeft = ((-verticalMovement - horizantalMovement - rotationalMovement) * maxPower);
            backRight = ((-verticalMovement - horizantalMovement + rotationalMovement) * maxPower);
            frontRight = ((verticalMovement - horizantalMovement - rotationalMovement) * maxPower);
        } else {
            backLeft = ((-nY - nX - rotationalMovement) * maxPower);
            frontLeft = ((-nY + nX - rotationalMovement) * maxPower);
            backRight = ((-nY + nX + rotationalMovement) * maxPower);
            frontRight = ((nY + nX - rotationalMovement) * maxPower);
        }

        hwchasis.backLeft.setPower(backLeft);
        hwchasis.backRight.setPower(backRight);
        hwchasis.frontLeft.setPower(frontLeft);
        hwchasis.frontRight.setPower(frontRight);

        // shoot motors & servos
        if (!bToggle){
            if (-hwchasis.shootPower.getVelocity() < 1200){
                hwchasis.shootPower.setPower(shootBall);
            } else {
                hwchasis.shootPower.setPower(0);
            }

            if(-hwchasis.shootPower.getVelocity() > (1150)){
                if(loadBall == 1){
                    hwchasis.holdLeft.setPosition(1);
                    hwchasis.holdRight.setPosition(1);
                } else {
                    hwchasis.holdLeft.setPosition(0);
                    hwchasis.holdRight.setPosition(0);
                }
            }
        } else {
            if (-hwchasis.shootPower.getVelocity() < maxShoot){
                hwchasis.shootPower.setPower(shootBall);
            } else {
                hwchasis.shootPower.setPower(0);
            }

            if(-hwchasis.shootPower.getVelocity() > (maxShoot-50) && -hwchasis.shootPower.getVelocity() < (maxShoot+50) ){
                if(loadBall == 1){
                    hwchasis.holdLeft.setPosition(1);
                    hwchasis.holdRight.setPosition(1);
                } else {
                    hwchasis.holdLeft.setPosition(0);
                    hwchasis.holdRight.setPosition(0);
                }
            }
        }

        // breaking movement system
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

        telemetry.addData("bToggle", bToggle);

        //apriltag
        aprilTagWebcam.update();

        java.util.List<AprilTagDetection> detections = aprilTagWebcam.getDetectedTags();

        if (!detections.isEmpty()) {
            AprilTagDetection tag = detections.get(0);
            if (tag.id == 24 || tag.id == 20){
                if(selectAT && tag.id == 20){
                    telemetry.addData("Selected AT", tag.id);
                    telemetry.addData("distance to AT (cm)", tag.ftcPose.y);
                    telemetry.addData("rotation to AT (cm)", tag.ftcPose.bearing);
                    ATRot = tag.ftcPose.bearing;
                    realRot = tag.ftcPose.bearing;
                    ATDis = tag.ftcPose.y;
                    ATTag = tag.id;
                } else {
                    if (selectAT){
                        realRot = 0.0f;
                        ATDis = 0.0f;
                        ATRot = 0.0f;
                        ATTag = 0.0f;
                    }
                }

                if(!selectAT && tag.id == 24){
                    telemetry.addData("Selected AT", tag.id);
                    telemetry.addData("distance to AT (cm)", tag.ftcPose.y);
                    telemetry.addData("rotation to AT (cm)", tag.ftcPose.bearing);
                    ATRot = tag.ftcPose.bearing;
                    realRot = tag.ftcPose.bearing;
                    ATDis = tag.ftcPose.y;
                    ATTag = tag.id;
                } else {
                    if (!selectAT){
                        ATDis = 0.0f;
                        ATRot = 0.0f;
                        ATTag = 0.0f;
                        realRot = 0.0f;
                    }
                }
            }
        } else {
            telemetry.addLine("No tags detected");
            ATDis = 0.0f;
            ATTag = 0.0f;
            realRot = 0.0f;
            ATRot = 0.0f;
        }

        telemetry.addData("speed",               "%.2f", maxPower);
        telemetry.addData("ball load",           "%.2f", loadBall);
        telemetry.addData("ball shoot",          "%.2f", shootBall);
        telemetry.addData("realRot", realRot);
        telemetry.addData("motorSpeed through AT transitor",          "%.2f", maxShoot);
        telemetry.addData("Shooter Velocity", "%.2f", -hwchasis.shootPower.getVelocity());
        telemetry.addLine("");

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
