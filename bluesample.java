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

@Autonomous(name = "BlueSample")
public class auto4 extends LinearOpMode {
    PIDController bracocontroler = new PIDController(p, i, d);
    PIDController vipercontroller = new PIDController(Kp, Ki, Kd);
    private VoltageSensor ControlHub_VoltageSensor;

    public static double
            p = 0.002, d = 0.0001, i = 0, target = 0,
            Kp = 0.0046, Kd = 0.0001, Ki = 0, vipertarget = 0;
    public static double f = 0.0004;
    public static double ticks_in_degree = 2140.0 / 90.0;
    double servo = 0.8;

    //BRACO
    private DcMotorEx braco;
    private DcMotorEx viper;
    //GARRA
    private Servo claw1, claw2, claw;

    boolean clip, clipoff1 = false;
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
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(-14, 62, Math.PI +Math.PI / 2));
        Garra garra = new Garra(hardwareMap);
        Motors motors = new Motors(hardwareMap);

        // Define trajeto do robô
        Action trajectoryAction1 = mecanumDrive.actionBuilder(mecanumDrive.pose)
                .strafeToConstantHeading(new Vector2d(-7, 37))
                .build();

        Action trajectoryAction2 = mecanumDrive.actionBuilder(new Pose2d(-7, 37, Math.PI + Math.PI / 2))
                .strafeToConstantHeading(new Vector2d(-27, 34))
                .splineToLinearHeading(new Pose2d(-46, 12, Math.PI*0), Math.PI)
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(-46, 55))
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(-46, 19))
                .splineToConstantHeading(new Vector2d(-55, 14), Math.PI) // vai pro 2 sample
                .strafeToConstantHeading(new Vector2d(-55, 55))//empurra
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(-55, 18))
                .splineToConstantHeading(new Vector2d(-60, 12), Math.PI)// vai pro 3 sample
                .strafeToConstantHeading(new Vector2d(-61, 55)) //empurra
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(-61, 20))
                .strafeToConstantHeading(new Vector2d(-61, 55))
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

            // Controle de voltagem da bateria
            if (ControlHub_VoltageSensor.getVoltage() < 6) {
                break;
            }


            if (opModeIsActive() && !isStopRequested()) {
                // Executa as ações simultâneas: movimento do robô e atualizações dos motores
                Actions.runBlocking(trajectoryAction1
                );
                if (start) {
                    target = 1480;
                    if (bracoPos > 1385) {
                        servo = 0.3;
                        vipertarget = 1040;
                        if (viperpos > 940) {
                            clip = true;
                        }
                    }
                }
                if (clip) {
                    target = 1500; //1520
                    start = false;
                    if (bracoPos > 1384) { //1450
                        claw.setPosition(1);
                        servo = 0.8;
                        if (clawPos > 0.65) {
                            vipertarget = 10;
                            if (viperpos < 120) {
                                clipoff1 = true;
                            }
                        }
                    }
                }
                if (clipoff1){
                    Actions.runBlocking(trajectoryAction2);
                    clip = false;
                    target = 50;
                    vipertarget = 5;
                    if (bracoPos < 80){
                        break;
                    }
                }
            }
        }
    }
}
