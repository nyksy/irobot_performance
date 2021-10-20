// File:          create_avoid_obstacles_java.java
// Date:
// Description:
// Author:
// Modifications:

import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.LED;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.PositionSensor;
import com.cyberbotics.webots.controller.Receiver;
import com.cyberbotics.webots.controller.TouchSensor;
import com.cyberbotics.webots.controller.Device;

public class create_avoid_obstacles_java extends Robot {

    static final int BUMPERS_NUMBER = 2;
    static final int BUMPER_LEFT = 0;
    static final int BUMPER_RIGHT = 1;
    static final int CLIFF_SENSORS_NUMBER = 4;
    static final int CLIFF_SENSOR_LEFT = 0;
    static final int CLIFF_SENSOR_FRONT_LEFT = 1;
    static final int CLIFF_SENSOR_FRONT_RIGHT = 2;
    static final int CLIFF_SENSOR_RIGHT = 3;
    static final int MAX_SPEED = 16;
    static final int NULL_SPEED = 0;
    static final int HALF_SPEED = 8;
    static final int MIN_SPEED = -16;
    static final double WHEEL_RADIUS = 0.031;
    static final double AXLE_LENGTH = 0.271756;
    static final double ENCODER_RESOLUTION = 507.9188;

    static create_avoid_obstacles_java robot;
    static int timeStep;

    static final int[] cliff_sensors =
            new int[]{CLIFF_SENSOR_LEFT, CLIFF_SENSOR_FRONT_LEFT, CLIFF_SENSOR_FRONT_RIGHT, CLIFF_SENSOR_RIGHT};
    static final String[] cliff_sensors_name =
            new String[]{"cliff_left", "cliff_front_left", "cliff_front_right", "cliff_right"};

    static final int[] bumpers = new int[]{BUMPER_LEFT, BUMPER_RIGHT};
    static final String[] bumpers_name = new String[]{"bumper_left", "bumper_right"};

    private final PositionSensor[] positionSensors;
    private final Motor[] motors;

    public create_avoid_obstacles_java() {
        motors = new Motor[]{getMotor("left wheel motor"), getMotor("right wheel motor")};
        positionSensors = new PositionSensor[]
                {getPositionSensor("left wheel sensor"), getPositionSensor("right wheel sensor")};
    }

    public void run() {
        //lähetääs ajelee
        while (robot.step(timeStep) != -1) {
            motors[0].setPosition(Double.POSITIVE_INFINITY);
        }
    }


    public static void main(String[] args) {

        // create the Robot instance.
        robot = new create_avoid_obstacles_java();
        timeStep = (int) Math.round(robot.getBasicTimeStep());

        // Main loop
        robot.run();
    }
}
