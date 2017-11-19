package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name="Motor Test", group="Test")
//@Disabled
public class motorTest extends OpMode {
    DcMotor motor1;
    DcMotor motor2;
    DcMotor motor3;
    DcMotor motor4;
    public void init()
    {
        motor1=hardwareMap.dcMotor.get("front_left");
        motor2=hardwareMap.dcMotor.get("front_right");
        motor3=hardwareMap.dcMotor.get("back_left");
        motor4=hardwareMap.dcMotor.get("back_right");

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor3.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void loop()
    {
        if(gamepad1.a) motor1.setPower(0.5);
        else motor1.setPower(0);
        if(gamepad1.b) motor2.setPower(0.5);
        else motor2.setPower(0);
        if(gamepad1.x) motor3.setPower(0.5);
        else motor3.setPower(0);
        if(gamepad1.y) motor4.setPower(0.5);
        else motor4.setPower(0);

        //DQ173A0G
    }
}
