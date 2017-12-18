package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;


/**
 * Created by evandehlinger on 10/24/17.
 */
@TeleOp(name = "TeleOp Mode", group = "Bit Boi")
//@Disabled
public class bitBoiTeleOp extends OpMode{
    DcMotor motorFL;
    DcMotor motorBL;
    DcMotor motorFR;
    DcMotor motorBR;
    DcMotor liftMotor;
    ModernRoboticsI2cGyro gyro;

    public void init() {
        motorBR=hardwareMap.dcMotor.get("motor4");
        motorBL=hardwareMap.dcMotor.get("motor2");
        motorFR=hardwareMap.dcMotor.get("motor3");
        motorFL=hardwareMap.dcMotor.get("motor1");
        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        liftMotor = hardwareMap.dcMotor.get("motorL");

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        gyro.calibrate();

        gyro.resetZAxisIntegrator();

    }

    public void loop() {





        motorFL.setPower((gamepad1.right_stick_y+gamepad1.right_stick_x-gamepad1.left_stick_x)*Math.asin(45+gyro.getIntegratedZValue()));
        motorFR.setPower((gamepad1.right_stick_y-gamepad1.right_stick_x+gamepad1.left_stick_x)*Math.acos(45+gyro.getIntegratedZValue()));
        motorBL.setPower((gamepad1.right_stick_y-gamepad1.right_stick_x-gamepad1.left_stick_x)*Math.acos(45+gyro.getIntegratedZValue()));
        motorBR.setPower((gamepad1.right_stick_y+gamepad1.right_stick_x+gamepad1.left_stick_x)*Math.asin(45+gyro.getIntegratedZValue()));
        if(gamepad1.a) {
            liftMotor.setTargetPosition(liftMotor.getCurrentPosition() + liftMotor.getCurrentPosition() / 4);
            liftMotor.setPower(.75);
        }
        telemetry.addData("Lift", liftMotor.getCurrentPosition());
        //telemetry.addData("gyro z:",gyro.getIntegratedZValue());\
        //telemetry.addData("motorL",(gamepad1.right_stick_y-gamepad1.right_stick_x-gamepad1.left_stick_x));
        //telemetry.addData("motorR",(gamepad1.right_stick_y+gamepad1.right_stick_x+gamepad1.left_stick_x));
        //telemetry.addData("motorTL",(gamepad1.right_stick_y/5+gamepad1.right_stick_x/5-gamepad1.left_stick_x/5));
        //telemetry.addData("motorTR",(gamepad1.right_stick_y/5-gamepad1.right_stick_x/5+gamepad1.left_stick_x/5));



    }
}
