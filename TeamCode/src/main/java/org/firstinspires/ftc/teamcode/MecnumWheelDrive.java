package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.openftc.apriltag.AprilTagDetection;

@TeleOp(name="Robot: Teleop Tank", group="Robot")

public class MecnumWheelDrive extends OpMode{
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    //declare motor types
    public DcMotor  backRight   = null;
    public DcMotor  frontRight  = null;
    public DcMotor  backLeft   = null;
    public DcMotor  frontLeft  = null;
    public DcMotor  shootPower  = null;
    public Servo  holdLeft  = null;
    public Servo  holdRight  = null;
    
    double shootTimer = 0;

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
        //define motors
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft  = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        shootPower = hardwareMap.get(DcMotor.class, "shootPower");
        holdLeft  = hardwareMap.get(Servo.class, "holdLeft");
        holdRight = hardwareMap.get(Servo.class, "holdRight");

        //set motor directions
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        holdLeft.setDirection(Servo.Direction.REVERSE);
        holdRight.setDirection(Servo.Direction.FORWARD);
        shootPower.setDirection(DcMotor.Direction.REVERSE);
        
        //reset servos
        holdLeft.setPosition(0.00f);
        holdRight.setPosition(0.00f);
        
        
        //send telemetry
        telemetry.addData(">", "Autonomous Running");
        
        //move out of box
        backLeft.setPower(0.1f);
        frontLeft.setPower(0.1);
        backRight.setPower(0.1f);
        frontRight.setPower(-0.1f);
    }
    
    @Override
    public void init_loop() {
        
    }
    
    @Override
    public void start() {
    }

    @Override
    public void loop() {
        //drive var declaration
        float horizantalMovement = -gamepad1.left_stick_x;
        float verticalMovement = gamepad1.left_stick_y;
        float rotationalMovement = gamepad1.right_stick_x;
        float xButton = gamepad1.x ? 1.0f : 0.0f;
        float maxPower = 0.5f + (xButton / 2.00f);
        float maxShoot = gamepad1.y ? 0.9f : 0.75f;
        float loadBall = gamepad1.right_bumper ? 1.0f : 0.0f;
        float shootBall = gamepad1.right_trigger;
        
        //drive system(strafe, turn, forward/backward)
        backLeft.setPower((-verticalMovement + horizantalMovement - rotationalMovement) * maxPower);
        frontLeft.setPower((-verticalMovement - horizantalMovement + rotationalMovement) * maxPower);
        backRight.setPower((-verticalMovement - horizantalMovement - rotationalMovement) * maxPower);
        frontRight.setPower((verticalMovement - horizantalMovement - rotationalMovement) * maxPower);
        
        //shoot motors
        shootPower.setPower(shootBall * (maxShoot));
        
        holdLeft.setPosition(loadBall);
        holdRight.setPosition(loadBall);

        //braking movement system
        if((horizantalMovement == 0.00f) && (verticalMovement == 0.00f) && (rotationalMovement == 0.00f)) {
          backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
          backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
          frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
          frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        };
        
        //stop flywheel
        if(shootBall == 0.00f) {
            shootPower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        };

        //send telemetry
        telemetry.addData("horizantalMovement",  "%.2f", horizantalMovement);
        telemetry.addData("verticalMovement", "%.2f", verticalMovement);
        telemetry.addData("rotationalMovement",  "%.2f", rotationalMovement);
        telemetry.addData("speed",  "%.2f", maxPower);
        telemetry.addData("ball load",  "%.2f", loadBall);
        telemetry.addData("ball shoot",  "%.2f", shootBall);
    }

    @Override
    public void stop() {
    }
}
