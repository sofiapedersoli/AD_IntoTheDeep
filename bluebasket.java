package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "BlueBasket")
public class auto2 extends LinearOpMode {
    PIDController bracocontroler = new PIDController(p, i, d);
    PIDController vipercontroller = new PIDController(Kp, Ki, Kd);
    private VoltageSensor ControlHub_VoltageSensor;

    public static double
            p = 0.002, d = 0.0001, i = 0, target = 0,
            Kp = 0.0054, Kd = 0.0001, Ki = 0, vipertarget = 0;
    public static double f = 0.0004;
    public static double ticks_in_degree = 2140.0 / 90.0;
    double servo = 0.8;

    //BRACO
    private DcMotorEx braco;
    private DcMotorEx viper;
    //GARRA
    private Servo claw1, claw2, claw;

    boolean clip, clipoff1,clipoff2, pegarS1, pegarS2, basket1, cbasket1,vbasket1, basket2, cbasket2, vbasket2, estaci = false;
    boolean start = true;

    public class Motors {

        // Inicializa os motores
        public Motors(HardwareMap hardwareMap) {
            // Configurações básicas para os motores dos bracos
            braco = hardwareMap.get(DcMotorEx.class, "braco");
            bracocontroler.setPID(p, i, d); // Configura PID
            braco.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            braco.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            viper = hardwareMap.get(DcMotorEx.class, "viper");
            vipercontroller.setPID(Kp, Ki, Kd); // Configura PID
            viper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            viper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public class Garra {
        public Garra(HardwareMap hardwareMap) {
            claw1 = hardwareMap.get(Servo.class, "claw1");
            claw2 = hardwareMap.get(Servo.class, "claw2");
            claw = hardwareMap.get(Servo.class, "claw");
        }
    }

    public void moveBraco(double power) {
        braco.setPower(power);
    }

    public void moveViper(double power) {
        viper.setPower(power);
    }

    public void moveServo(double servoPosition) {
        claw1.setPosition(servoPosition);
        claw2.setPosition(servoPosition);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Inicializa subsistemas
        ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(33, 62, Math.PI + Math.PI/2));
        Garra garra = new Garra(hardwareMap);
        Motors motors = new Motors(hardwareMap);

        // Define trajeto do robô
        Action trajectoryAction1 = mecanumDrive.actionBuilder(mecanumDrive.pose)
                .strafeToConstantHeading(new Vector2d(9, 37))
                .build();

        Action trajectoryAction2 = mecanumDrive.actionBuilder(new Pose2d(9, 37, Math.PI + Math.PI/2))
                .strafeToConstantHeading(new Vector2d(50, 48))
                .build();

        Action parada1sample = mecanumDrive.actionBuilder(new Pose2d(50, 48, Math.PI + Math.PI/2))
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(50, 51))
                .build();

        Action trajectoryAction3 = mecanumDrive.actionBuilder(new Pose2d(50, 51, Math.PI + Math.PI/2))
                .strafeToLinearHeading(new Vector2d( 61, 56), Math.PI + Math.PI/4)
                .build();

        Action trajectoryAction4 = mecanumDrive.actionBuilder(new Pose2d(61, 56, Math.PI + Math.PI/4))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d( 61, 55), Math.PI + Math.PI/4)
                .build();

        Action trajectoryAction5 = mecanumDrive.actionBuilder(new Pose2d(61, 55, Math.PI + Math.PI/4))
                .strafeToLinearHeading(new Vector2d( 58, 47), Math.PI + Math.PI/2)
                .build();

        Action parada2sample = mecanumDrive.actionBuilder(new Pose2d(58, 47, Math.PI + Math.PI /2))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(57, 48), Math.PI + Math.PI/2)
                .build();

        Action trajectoryAction6 = mecanumDrive.actionBuilder(new Pose2d(57, 48, Math.PI + Math.PI/2))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(62, 56), Math.PI + Math.PI/4)
                .build();

        Action trajectoryAction7 = mecanumDrive.actionBuilder(new Pose2d(62, 56, Math.PI + Math.PI/4))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(63, 55), Math.PI + Math.PI/4)
                .build();

        Action trajectoryAction8 = mecanumDrive.actionBuilder(new Pose2d(63, 55, Math.PI + Math.PI/4))
                .waitSeconds(0.5)
                .splineTo(new Vector2d(25, 8), Math.PI)
                .build();

        // Espera início da partida
        waitForStart();
        braco.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        braco.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        viper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {
            int viperpos = viper.getCurrentPosition();
            double viperpid = vipercontroller.calculate(viperpos, vipertarget);

            // Atualiza a configuração do PID
            bracocontroler.setPID(p, i, d);
            vipercontroller.setPID(Kp, Ki, Kd);

            // Calcula a posição do braço e o valor do PID
            int bracoPos = braco.getCurrentPosition() * -1;
            double pid = bracocontroler.calculate(bracoPos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

            double clawPos = claw.getPosition();
            double claw1Pos = claw1.getPosition();


            // BRACO e VIPER funções
            moveBraco(pid + ff);
            moveViper(viperpid);
            moveServo(servo);

            telemetry.addData("pos", bracoPos);
            telemetry.addData("viper", viperpos);
            telemetry.update();

            if (ControlHub_VoltageSensor.getVoltage() < 6) {
                break;
            }

            if (opModeIsActive() && !isStopRequested()) {
                // Executa as ações simultâneas: movimento do robô e atualizações dos motores
                Actions.runBlocking(trajectoryAction1);
                if (start == true) {
                    target = 1480;
                    if (bracoPos > 1385) {
                        servo = 0.3;
                        vipertarget = 1040;    //1080
                        if (viperpos > 940) {
                            clip = true;
                        }
                    }
                }
                if (clip == true) {
                    target = 1500; //1520
                    start = false;
                    if (bracoPos > 1390) { //1450
                        claw.setPosition(1);
                        servo = 0.8;
                        if (clawPos > 0.6) {
                            vipertarget = 5;
                            if (viperpos < 100) {
                                Actions.runBlocking(trajectoryAction2);
                                clipoff1 = true;
                            }
                        }
                    }
                }
                if (clipoff1 == true) {
                    target = 240;
                    clip = false;
                    claw.setPosition(1);
                    if (clawPos > 0.8) { //if (bracoPos < 300)
                        vipertarget = 810;//830
                        servo = 0.0;
                        if (viperpos > 780){
                            pegarS1 = true;
                        }
                    }
                }
                if (pegarS1 == true) {
                    clipoff1 = false;
                    target = 155; //170
                    if (bracoPos < 216) { //200 180 219
                        claw.setPosition(0);
                        if (clawPos < 0.1) {
                            claw.setPosition(0);
                            basket1 = true;
                        }
                    }
                }
                if (basket1 == true) {
                    pegarS1 = false;
                    Actions.runBlocking(parada1sample);
                    servo = 0.3;
                    vipertarget = 10;
                    if (viperpos < 120) {
                        Actions.runBlocking(trajectoryAction3);
                        servo = 0.5;
                        if (claw1Pos > 0.4) {
                            cbasket1 = true;
                        }
                    }
                }
                if (cbasket1 == true) {
                    basket1 = false;
                    target = 2310;
                    if (bracoPos > 1450){
                        vipertarget = 1450; //1570
                        if (viperpos > 1402) { //1410 1405 aqui
                            servo = 0.8;
                            claw.setPosition(1);
                            if (clawPos > 0.95) {
                                vbasket1 = true;
                            }
                        }
                    }
                }
                if (vbasket1 == true) {
                    Actions.runBlocking(trajectoryAction4);
                    cbasket1 = false;
                    target = 1900;
                    if (bracoPos < 2000) {
                        vipertarget = 10;
                        if(viperpos < 200){
                            clipoff2 = true;
                        }
                    }
                }
                if (clipoff2 == true) {
                    vbasket1 = false;
                    Actions.runBlocking(trajectoryAction5);
                    target = 250;
                    if (bracoPos < 300) {
                        vipertarget = 780;//830 800 790
                        servo = 0.0;
                        if (viperpos > 765){ //780 772
                            pegarS2 = true;
                        }
                    }
                }
                if (pegarS2 == true) {
                    clipoff2 = false;
                    target = 150;
                    if (bracoPos < 199) {   //200
                        claw.setPosition(0);
                        if (clawPos < 0.1) {
                            claw.setPosition(0);
                            basket2 = true;
                        }
                    }
                }
                if (basket2 == true) {
                    pegarS2 = false;
                    Actions.runBlocking(parada2sample);
                    servo = 0.25;
                    vipertarget = 10;
                    if (viperpos < 120) {
                        Actions.runBlocking(trajectoryAction6);
                        servo = 0.5;
                        if (claw1Pos > 0.4) {
                            cbasket2 = true;
                        }
                    }
                }
                if (cbasket2 == true) {
                    basket2 = false;
                    target = 2325; //2360
                    if (bracoPos > 1500){
                        vipertarget = 1460; //1540
                        if (viperpos > 1410) {
                            servo = 0.8;
                            claw.setPosition(1);
                            if (clawPos > 0.95) {
                                vbasket2 = true;
                            }
                        }
                    }
                }
                if (vbasket2 == true) {
                    cbasket2 = false;
                    Actions.runBlocking(trajectoryAction7);
                    servo = 0.6;
                    target = 1750;
                    if (bracoPos < 1900) {
                        vipertarget = 50;
                        if ( viperpos < 200){
                            estaci = true;
                        }
                    }
                }
                if (estaci == true) {
                    Actions.runBlocking(trajectoryAction8);
                    target = 1220;
                    vbasket2 = false;
                    if (bracoPos > 1080){
                        vbasket2 = false;
                        vipertarget = 800;
                        if (viperpos > 700){
                            break;
                        }
                    }
                }
            }
        }
    }
}
