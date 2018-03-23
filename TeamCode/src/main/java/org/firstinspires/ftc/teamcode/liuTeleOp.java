package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
/**
 * Created by Patrick on 3/12/2018.
 */
@TeleOp (name = "liuTeleOp")
public class liuTeleOp extends OpMode{
    // defines the motor objects
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor lifter;
    // variables for motor power

    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;
    @Override
    public void init() {
        //initializes motors in configuration
        backLeft = hardwareMap.dcMotor.get("back left motor");
        backRight = hardwareMap.dcMotor.get("back right motor");
        frontLeft = hardwareMap.dcMotor.get("front left motor");
        frontRight = hardwareMap.dcMotor.get("front right motor");
        lifter = hardwareMap.dcMotor.get("lift motor");
        
        // puts the left motors in reverse, so it allows for turning
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    @Override
    public void loop(){
        //driving power values
        frontLeftPower = (gamepad1.left_stick_x + (-gamepad1.left_stick_y * 1.5));
        frontRightPower = (-gamepad1.left_stick_x + (-gamepad1.left_stick_y * 1.5));
        backLeftPower = (-gamepad1.left_stick_x + (-gamepad1.left_stick_y * 1.5));
        backRightPower = (gamepad1.left_stick_x + (-gamepad1.left_stick_y * 1.5));
        //turning values
        frontLeftPower = gamepad1.right_stick_x;
        frontRightPower = -gamepad1.right_stick_x;
        backLeftPower = gamepad1.right_stick_x;
        backRightPower = -gamepad1.right_stick_x;

        frontLeft.setPower(frontLeftPower * 0.75);
        frontRight.setPower(frontRightPower * 0.75);
        backLeft.setPower(backLeftPower * 0.75);
        backRight.setPower(backRightPower * 0.75);
        //lifter motor power
        if (gamepad1.a)
            lifter.setPower(0.5);
        if (gamepad1.b)
            lifter.setPower(-0.5);
        else
            lifter.setPower(0);
    }
}
