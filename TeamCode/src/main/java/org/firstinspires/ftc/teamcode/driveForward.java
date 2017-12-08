package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.move;

@Autonomous(name="A C 0", group="Autonomous Finals")
//@Disabled
public class driveForward extends LinearOpMode{
    ElapsedTime y = new ElapsedTime();
    DcMotor flmotor;
    DcMotor frmotor;
    DcMotor blmotor;
    DcMotor brmotor;
    double motorPower = 0.75;

    public void runOpMode()
    {
        flmotor = hardwareMap.get(DcMotor.class, "front_left");
        frmotor = hardwareMap.get(DcMotor.class, "front_right");
        blmotor = hardwareMap.get(DcMotor.class, "back_left");
        brmotor = hardwareMap.get(DcMotor.class, "back_right");
        frmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brmotor.setDirection(DcMotorSimple.Direction.REVERSE);

        new move(this, true);
        y.reset();

        flmotor.setPower(motorPower);
        frmotor.setPower(motorPower);
        blmotor.setPower(motorPower);
        brmotor.setPower(motorPower);
        while(opModeIsActive()&&y.milliseconds()<750);

        flmotor.setPower(0);
        frmotor.setPower(0);
        blmotor.setPower(0);
        brmotor.setPower(0);

    }
}
