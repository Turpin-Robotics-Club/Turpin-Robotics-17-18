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


    DcMotor motorL;
    DcMotor motorR;
    DcMotor motorC;
    OpticalDistanceSensor odSensor;
    public void runOpMode() throws InterruptedException{
        motorL=hardwareMap.dcMotor.get("motor_1");
        motorR=hardwareMap.dcMotor.get("motor_2");
        motorC=hardwareMap.dcMotor.get("motor_3");
        odSensor = hardwareMap.opticalDistanceSensor.get("od_sensor");

        motorL.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        motorL.setPower(.5);
        motorR.setPower(.5);
        sleep(10000);
        if(odSensor.getLightDetected()!=0){
            motorR.setPower(0);
            motorL.setPower(0);
        }


    }
}
