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


/**
 * Created by evandehlinger on 10/24/17.
 */

@Autonomous(name="Bit Boi Audio", group="Bit Boi is cooler than you")
//@Disabled
public class bitBoi extends LinearOpMode {

    DcMotor motor1;
    DcMotor motor2;

    public void runOpMode() throws InterruptedException{
        motor1=hardwareMap.dcMotor.get("motor_1");
        motor2=hardwareMap.dcMotor.get("motor_2");

        motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        motor1.setPower(.5);
        motor2.setPower(.5);

        sleep(10000);


    }
}
