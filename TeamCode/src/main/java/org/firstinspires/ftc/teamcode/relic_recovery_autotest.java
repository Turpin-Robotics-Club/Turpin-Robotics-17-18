package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.move;
import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
/**
 * Created by Patrick on 11/20/2017.
 */
@Autonomous (name="relic_autotest", group="Autonomous Finals")
public class relic_recovery_autotest extends LinearOpMode{

    DcMotor flmotor;
    DcMotor frmotor;
    DcMotor blmotor;
    DcMotor brmotor;
    private Servo clamp;
    private DcMotor liftMotor;
    private DcMotor liftMotor2;

    VuforiaLocalizer vuforia;
    OpenGLMatrix lastLocation = null;

    public void runOpMode() throws InterruptedException{
        // Initializing vuforia
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AYt3nnz/////AAAAGeWqnGxuREQ8gPunRf7bkzYaJ6lsas+H/ryI/7UQ6Kg/QpCi1ObUnVT96byceD0lMQsIV4bqROkXYKwfjL+79oOM19r9qKF3OnRKItM47YmGatBI9Z0u2rkFRRz1rd/ESSZxvBKLsnVn5uaNvTIgkMJ/Lh0HCl0aQfAf1khSVuZR/6mlcAwf++ejAl+lXPdk716k7fXZvnvEDAkWu7GqG2esiLDoXPcsrWIKAbv9UAwSLIvxVIzHTJBgncJ5a3etLPI0bxwlk/1AZb4ZZ6iDFXLoyv7suXac2ek30Tar6UdJ1EXSxdOMlCZRfes8HdpbmBcyElEmC8+mBsJhaaMN+erUF6Es5eCgilirNZ/Rbf0S";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        // could be front or back
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();
    }
}
