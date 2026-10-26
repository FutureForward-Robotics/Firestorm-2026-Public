package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.field.ScoringElement;
import org.firstinspires.ftc.teamcode.util.BusWrites;
import org.firstinspires.ftc.teamcode.util.RunLog;

/** A belt up to two gates, one in front of each shooter. */
public class Indexer extends ForwardSubsystem {

    public enum State {
        HOLD,
        LOAD,
        FEED_POLLEN,
        FEED_NECTAR,
        UNJAM
    }

    // Placeholders until measured on the robot.
    public static final double FEED_POWER = 1.0;
    public static final double LOAD_POWER = 0.5;
    public static final double GATE_CLOSED_POSITION = 0.0;
    public static final double GATE_OPEN_POSITION = 1.0;
    public static final String BELT_MOTOR_NAME = "indexerBelt";
    public static final String POLLEN_SERVO_NAME = "gatePollen";
    public static final String NECTAR_SERVO_NAME = "gateNectar";

    private final DcMotorEx indexerMotor;
    private final CRServo pollenGate;
//    private final Servo nectarGate;

    private State state = State.HOLD;

    // NaN so the first loop always writes. A servo keeps the position it powered up in.
    private double lastBeltPowerWritten = Double.NaN;
    private double lastPollenGatePositionWritten = Double.NaN;
    private double lastNectarGatePositionWritten = Double.NaN;

    public Indexer(HardwareMap hardwareMap) {
        this.indexerMotor = hardwareMap.get(DcMotorEx.class, BELT_MOTOR_NAME);
        this.pollenGate = hardwareMap.get(CRServo.class, POLLEN_SERVO_NAME);
//        this.nectarGate = hardwareMap.get(Servo.class, NECTAR_SERVO_NAME);
        this.indexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void sense() {}

    @Override
    public void act() {
        double beltPower = 0;
        double pollenGatePosition = GATE_CLOSED_POSITION;
        double nectarGatePosition = GATE_CLOSED_POSITION;

        switch (state) {
            case HOLD:
                break;
            case LOAD:
                beltPower = LOAD_POWER;
                pollenGatePosition = -0.12;
                break;
            case FEED_POLLEN:
                beltPower = FEED_POWER;
                pollenGatePosition = GATE_OPEN_POSITION;
                break;
            case FEED_NECTAR:
                beltPower = FEED_POWER;
                nectarGatePosition = GATE_OPEN_POSITION;
                break;
            case UNJAM:
                beltPower = -FEED_POWER;
                break;
        }

        if (BusWrites.worthMotor(beltPower, lastBeltPowerWritten)) {
            indexerMotor.setPower(beltPower);
            lastBeltPowerWritten = beltPower;
        }
        if (BusWrites.worthServo(pollenGatePosition, lastPollenGatePositionWritten)) {
            pollenGate.setPower(pollenGatePosition);
            lastPollenGatePositionWritten = pollenGatePosition;
        }
        if (BusWrites.worthServo(nectarGatePosition, lastNectarGatePositionWritten)) {
//            nectarGate.setPosition(nectarGatePosition);
            lastNectarGatePositionWritten = nectarGatePosition;
        }
    }

    @Override
    public void logSignals(RunLog log) {
        log.addSignal("indexer.state", () -> state.ordinal());
        log.addSignal("indexer.beltPower", () -> lastBeltPowerWritten);
        log.addSignal("indexer.pollenGate", () -> lastPollenGatePositionWritten);
        log.addSignal("indexer.nectarGate", () -> lastNectarGatePositionWritten);
    }

    public Command hold() {
        return new InstantCommand(() -> state = State.HOLD);
    }


    /** Runs the belt with both gates closed, to carry balls up from the intake. */
    public Command load() {
        return new InstantCommand(() -> state = State.LOAD);
    }

    public Command feed(ScoringElement element) {
        State feedState = element == ScoringElement.POLLEN ? State.FEED_POLLEN : State.FEED_NECTAR;
        return new InstantCommand(() -> state = feedState);
    }

    public Command unjam() {
        return new InstantCommand(() -> state = State.UNJAM);
    }
}
