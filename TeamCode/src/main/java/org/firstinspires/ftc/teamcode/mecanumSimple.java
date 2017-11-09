package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.RobotConstants;



@TeleOp(name="Simple Drive")
public class mecanumSimple extends OpMode{

    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;



    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;


    public final double SPEED = 0.75;
    public final double forwardBonus = 1.5;



    @Override
    public void init() {
        frontleft = hardwareMap.dcMotor.get("front_left");
        frontright = hardwareMap.dcMotor.get("front_right");
        backleft = hardwareMap.dcMotor.get("back_left");
        backright = hardwareMap.dcMotor.get("back_right");

        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backright.setDirection(DcMotorSimple.Direction.REVERSE);



    }

    @Override
    public void loop() {



        // Movement
        frontLeftPower = (gamepad1.left_stick_x + (gamepad1.left_stick_y * forwardBonus));
        frontRightPower = (-gamepad1.left_stick_x + (gamepad1.left_stick_y * forwardBonus));
        backLeftPower = (-gamepad1.left_stick_x + (gamepad1.left_stick_y * forwardBonus));
        backRightPower = (gamepad1.left_stick_x + (gamepad1.left_stick_y * forwardBonus));

        // Turning
        frontLeftPower += gamepad1.right_stick_x;
        frontRightPower += -gamepad1.right_stick_x;
        backLeftPower += gamepad1.right_stick_x;
        backRightPower += -gamepad1.right_stick_x;

        frontleft.setPower(frontLeftPower * SPEED);
        frontright.setPower(frontRightPower * SPEED);
        backleft.setPower(backLeftPower * SPEED);
        backright.setPower(backRightPower * SPEED);



    }

    @Override
    public void stop() {
        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);
    }
}
