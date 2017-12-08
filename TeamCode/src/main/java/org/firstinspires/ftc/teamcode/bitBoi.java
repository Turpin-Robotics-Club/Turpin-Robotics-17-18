package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by evandehlinger on 10/24/17.
 */

@Autonomous(name="Bit Boi Audio", group="Bit Boi")
//@Disabled
public class bitBoi extends LinearOpMode {


    DcMotor motor1;
    DcMotor motor2;
    DcMotor motor3;
    DcMotor motor4;
    OpticalDistanceSensor odSensor;
    public void runOpMode() throws InterruptedException{
        motor1=hardwareMap.dcMotor.get("motor1");
        motor2=hardwareMap.dcMotor.get("motor2");
        motor3=hardwareMap.dcMotor.get("motor3");
        motor4=hardwareMap.dcMotor.get("motor4");
        odSensor = hardwareMap.opticalDistanceSensor.get("od_sensor");

        motor3.setDirection(DcMotorSimple.Direction.REVERSE);
        motor4.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        motor1.setPower(gamepad1.right_stick_y);
        motor2.setPower(gamepad1.right_stick_y);
        motor3.setPower(gamepad1.right_stick_y);
        motor4.setPower(gamepad1.right_stick_y);

        telemetry.addData("right stick y", gamepad1.right_stick_y);


    }
}
