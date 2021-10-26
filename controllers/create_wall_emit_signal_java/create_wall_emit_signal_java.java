// File:          create_wall_emit_signal_java.java
// Date:
// Description:
// Author:
// Modifications:

// You may need to add other webots classes
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Emitter;

// Here is the main class of your controller.
// This class defines how to initialize and how to run your controller.
public class create_wall_emit_signal_java {

  private static final byte[] dummy_data = {(byte)0xFF};
  private static Robot robot;


  private static void step() {

    // get the time step of the current world.
    int timeStep = (int) Math.round(robot.getBasicTimeStep());

    if (robot.step(timeStep) == -1) {
      System.exit(1);
    }

  }


  // This is the main function of your controller.
  // It creates an instance of your Robot instance and
  // it uses its function(s).
  // Note that only one instance of Robot should be created in
  // a controller program.
  // The arguments of the main function can be specified by the
  // "controllerArgs" field of the Robot node
  public static void main(String[] args) {

    // create the Robot instance.
    robot = new Robot();

    System.out.println("Controller of the iRobot Create Wall (Java) started...\n");

    Emitter emitter = robot.getEmitter("emitter");

    // You should insert a getDevice-like function in order to get the
    // instance of a device of the robot. Something like:
    //  Motor motor = robot.getMotor("motorname");
    //  DistanceSensor ds = robot.getDistanceSensor("dsname");
    //  ds.enable(timeStep);

    // Main loop:
    // - perform simulation steps until Webots is stopping the controller
    while (true) {

      emitter.send(dummy_data, 1);
      step();

    }

    // Enter here exit cleanup code.
  }

}
