package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.utils.RobotConstants;
import org.firstinspires.ftc.teamcode.utils.Sensors;
import org.firstinspires.ftc.teamcode.utils.move;

@Autonomous(name="B D 0", group="Autonomous Finals")
//@Disabled
public class driveLeft extends LinearOpMode {

    DcMotor flmotor;
    DcMotor frmotor;
    DcMotor blmotor;
    DcMotor brmotor;
    private Servo clamp;
    private DcMotor liftMotor;
    private DcMotor liftMotor2;

    public void runOpMode() {


        move drive = new move(this);
        drive.forward(27,0.3);
        drive.turnRight(90,0.5);
        RobotConstants.vuMark = RelicRecoveryVuMark.CENTER;
        drive.release();
        drive.toColumn();
        drive.forward(-2,-0.5);
        drive.liftZero();
        while (opModeIsActive());
    }
}