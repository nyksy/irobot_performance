// File:          Create_avoid_obstacles_ver2.java
// Date:
// Description:
// Author:
// Modifications:

import com.cyberbotics.webots.controller.*;

import java.util.Random;

public class Create_avoid_obstacles_ver2 extends Robot {

    //robotin speksit ja vakioita
    static final int BUMPERS_NUMBER = 2;
    static final int BUMPER_LEFT = 0;
    static final int BUMPER_RIGHT = 1;

    final TouchSensor[] bumpers = new TouchSensor[BUMPERS_NUMBER];
    static final String[] bumpers_name = new String[]{"bumper_left", "bumper_right"};

    static final int CLIFF_SENSORS_NUMBER = 4;
    static final int CLIFF_SENSOR_LEFT = 0;
    static final int CLIFF_SENSOR_FRONT_LEFT = 1;
    static final int CLIFF_SENSOR_FRONT_RIGHT = 2;
    static final int CLIFF_SENSOR_RIGHT = 3;

    final DistanceSensor[] cliffSensors = new DistanceSensor[4];
    static final String[] cliff_sensors_name = new String[]
            {
                    "cliff_left", "cliff_front_left", "cliff_front_right", "cliff_right"
            };

    static final int MAX_SPEED = 16;
    static final int NULL_SPEED = 0;
    static final int HALF_SPEED = 8;
    static final int MIN_SPEED = -16;
    static final double WHEEL_RADIUS = 0.031;
    static final double AXLE_LENGTH = 0.271756;
    static final double ENCODER_RESOLUTION = 507.9188;

    static final int LEDS_NUMBER = 3;
    static final int LED_ON = 0;
    static final int LED_PLAY = 1;
    static final int LED_STEP = 2;

    final LED[] leds = new LED[LEDS_NUMBER];
    static final String[] leds_name = new String[]{"led_on", "led_play", "led_step"};

    private static final int POS_SENSOR_LEFT = 0;
    private static final int POS_SENSOR_RIGHT = 1;
    private final PositionSensor[] positionSensors;

    private static final int MOTOR_LEFT = 0;
    private static final int MOTOR_RIGHT = 1;
    private final Motor[] motors;

    private final Receiver receiver = getReceiver("receiver");

    static Create_avoid_obstacles_ver2 robot;

    //additional bodyslot utilities
    private final Camera camera = getCamera("camera");
    private final GPS gps = getGPS("gps");
    private final Compass compass = getCompass("compass");

    /**
     * Constructor
     */
    public Create_avoid_obstacles_ver2() {
        //init devices

        //motors
        motors = new Motor[]{getMotor("left wheel motor"), getMotor("right wheel motor")};
        motors[MOTOR_LEFT].setPosition(Double.POSITIVE_INFINITY);
        motors[MOTOR_RIGHT].setPosition(Double.POSITIVE_INFINITY);
        motors[MOTOR_LEFT].setVelocity(NULL_SPEED);
        motors[MOTOR_RIGHT].setVelocity(NULL_SPEED);

        //position sensors
        positionSensors = new PositionSensor[]
                {
                        getPositionSensor("left wheel sensor"), getPositionSensor("right wheel sensor")
                };
        positionSensors[POS_SENSOR_LEFT].enable(getTimeStep());
        positionSensors[POS_SENSOR_RIGHT].enable(getTimeStep());

        //receiver
        receiver.enable(getTimeStep());

        //camera
        camera.enable(getTimeStep());

        //gps
        //https://cyberbotics.com/doc/reference/gps?tab-language=java
        gps.enable(getTimeStep());

        //compass
        //https://cyberbotics.com/doc/reference/compass?tab-language=java
        compass.enable(getTimeStep());

        //leds
        for (int i = 0; i < leds.length; i++) {
            leds[i] = getLED(leds_name[i]);
        }

        //bumpers
        for (int i = 0; i < bumpers.length; i++) {
            bumpers[i] = getTouchSensor(bumpers_name[i]);
            bumpers[i].enable(getTimeStep());
        }

        //cliff sensors
        for (int i = 0; i < cliffSensors.length; i++) {
            cliffSensors[i] = getDistanceSensor(cliff_sensors_name[i]);
            cliffSensors[i].enable(getTimeStep());
        }
    }

    /**
     * Robotti tulille
     */
    public void run() {

        System.out.println("Robot: " + robot.getModel());

        leds[LED_ON].set(1);

        passiveWait(0.5);

        //lähetääs ajelee
        while (1 == 1) {
            if (isThereAVirtualWall()) {
                System.out.println("Virtual wall detected");
            } else if (isThereCollisionAtLeft() || isThereACliffAtLeft()) {
                System.out.println("Left obstacle detected");
                goBackward();
                passiveWait(0.5);

                // Arvotaan käännös oikealle 90 - 180 astetta.
                turn(-Math.PI / 2 * (1 + randdouble()));

            } else if (isThereCollisionAtRight() || isThereACliffAtRight() || isThereACliffAtFront()) {
                System.out.println("Right obstacle detected");
                goBackward();
                passiveWait(0.5);

                // Arvotaan käännös vasemmalle 90 - 180 astetta.
                turn(Math.PI / 2 * (1 + randdouble()));

            } else {
                goForward();
            }

            //Loggailut
            //System.out.println("---");
            //System.out.println("LOG");
            //System.out.println("GPS vector: " + Arrays.toString(gps.getValues()));
            //System.out.println("GPS speed: " + String.format("%.4f", gps.getSpeed()) + " m/s");
            //System.out.println("Compass values: " + Arrays.toString(compass.getValues()));
            //System.out.println("Bearing: " + Math.round(getBearingInDegrees()) + " °");
            fflushIrReceiver();
            step(getTimeStep());
        }
    }

    public void step() {
        if (step(getTimeStep()) == -1) {
            System.exit(1);
        }
    }

    public boolean isThereCollisionAtLeft() {
        return bumpers[BUMPER_LEFT].getValue() != 0.0;
    }

    public boolean isThereCollisionAtRight() {
        return bumpers[BUMPER_RIGHT].getValue() != 0.0;
    }

    public void fflushIrReceiver() {
        while (receiver.getQueueLength() > 0) {
            receiver.nextPacket();
        }
    }

    public boolean isThereAVirtualWall() {
        return receiver.getQueueLength() > 0;
    }

    public boolean isThereACliffAtLeft() {
        return cliffSensors[CLIFF_SENSOR_LEFT].getValue() < 100.0 ||
                cliffSensors[CLIFF_SENSOR_FRONT_LEFT].getValue() < 100.0;
    }

    public boolean isThereACliffAtRight() {
        return cliffSensors[CLIFF_SENSOR_RIGHT].getValue() < 100.0 ||
                cliffSensors[CLIFF_SENSOR_FRONT_RIGHT].getValue() < 100.0;
    }

    public boolean isThereACliffAtFront() {
        return cliffSensors[CLIFF_SENSOR_FRONT_LEFT].getValue() < 100.0 ||
                cliffSensors[CLIFF_SENSOR_FRONT_RIGHT].getValue() < 100.0;
    }

    public void goForward() {
        motors[MOTOR_LEFT].setVelocity(MAX_SPEED);
        motors[MOTOR_RIGHT].setVelocity(MAX_SPEED);
    }

    public void goBackward() {
        //TODO tähän vois heittää MIN_SPEED periaatteessa
        motors[MOTOR_LEFT].setVelocity(-HALF_SPEED);
        motors[MOTOR_RIGHT].setVelocity(-HALF_SPEED);
    }

    public void stop() {
        motors[MOTOR_LEFT].setVelocity(NULL_SPEED);
        motors[MOTOR_RIGHT].setVelocity(NULL_SPEED);
    }

    public void passiveWait(Double seconds) {
        Double startTime = getTime();
        do {
            step();
        } while (startTime + seconds > getTime());
    }

    public Double randdouble() {
        return new Random().nextDouble();
    }

    /**
     * Kompassin palauttama arvo asteiksi
     *
     * @return robot's bearing in degrees
     */
    public Double getBearingInDegrees() {
        double[] north = compass.getValues();
        double rad = Math.atan2(north[0], north[2]);
        double bearing = (rad - 1.5708) / Math.PI * 180.0;

        if (bearing < 0.0) {
            bearing += 360.0;
        }
        return bearing;
    }


    public void turn(Double angle) {
        System.out.println("Turn angle: " + Math.round(angle / Math.PI * 180) + " degrees.");

        stop();
        double lOffset = positionSensors[POS_SENSOR_LEFT].getValue();
        double rOffset = positionSensors[POS_SENSOR_RIGHT].getValue();
        step();
        double neg = angle < 0.0 ? -1.0 : 1.0;
        motors[MOTOR_LEFT].setVelocity(neg * HALF_SPEED);
        motors[MOTOR_RIGHT].setVelocity(-neg * HALF_SPEED);
        double orientation;
        do {
            double l = positionSensors[POS_SENSOR_LEFT].getValue() - lOffset;
            double r = positionSensors[POS_SENSOR_RIGHT].getValue() - rOffset;
            double dl = l * WHEEL_RADIUS;
            double dr = r * WHEEL_RADIUS;
            orientation = neg * (dl - dr) / AXLE_LENGTH;
            step();
        } while (orientation < neg * angle);
        stop();
        step();
    }

    /**
     * apufunktio timestepille
     *
     * @return timestep
     */
    private int getTimeStep() {
        return (int) getBasicTimeStep();
    }


    public static void main(String[] args) {

        // create the Robot instance.
        robot = new Create_avoid_obstacles_ver2();

        // Main loop
        robot.run();
    }
}
