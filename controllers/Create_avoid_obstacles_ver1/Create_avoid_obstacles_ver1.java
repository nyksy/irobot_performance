// File:          Create_avoid_obstacles_ver1.java
// Date:
// Description:
// Author:
// Modifications:

import com.cyberbotics.webots.controller.*;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.DecimalFormat;
import java.util.*;
import java.util.stream.Collectors;

public class Create_avoid_obstacles_ver1 extends Robot {

  static State tila = null;

  private static final DecimalFormat df = new DecimalFormat("0.000");

  //delay tulostusten välissä sekunteina
  static final int DELAY = 1;

  static final double DEG_90 = Math.PI / 2.0;

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

  final DistanceSensor[] cliffSensors = new DistanceSensor[CLIFF_SENSORS_NUMBER];
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

  static Create_avoid_obstacles_ver1 robot;

  //additional bodyslot utilities
  private final Camera camera = getCamera("camera");
  private final GPS gps = getGPS("gps");
  private final Compass compass = getCompass("compass");

  private final TableMap tableMap = new TableMap();
  private int alreadyCleanedCounter = 0;


  /**
   * Constructor
   */
  public Create_avoid_obstacles_ver1() {
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

    List<String[]> data = new ArrayList<>();
    //otsikkorivi
    data.add(new String[]{"time", "cleaning"});

    //long time = System.currentTimeMillis();
    double time = robot.getTime();

    leds[LED_ON].set(1);

    passiveWait(0.5);

    //lähetääs ajelee
    while (tableMap.getCurrentProgress() < 95) {

      tableMap.addLocation(getGPSLocation());

      if (isThereAVirtualWall()) {
        System.out.println(tableMap);
        System.out.println("Virtual wall detected");
        System.out.println("Puhdistettu: " + tableMap.getCleaningPercentage() + " %");
        turn(Math.PI);
      } else if (isThereCollisionAtLeft() || isThereACliffAtLeft()) {
        System.out.println(tableMap);
        System.out.println("Left obstacle detected");
        System.out.println("Puhdistettu: " + tableMap.getCleaningPercentage() + " %");
        goBackward();
        passiveWait(0.5);
        turn(Math.PI * randdouble());
      } else if (isThereCollisionAtRight() || isThereACliffAtRight() || isThereACliffAtFront()) {
        System.out.println(tableMap);
        System.out.println("Right obstacle detected");
        System.out.println("Puhdistettu: " + tableMap.getCleaningPercentage() + " %");
        goBackward();
        passiveWait(0.5);
        turn(-Math.PI * randdouble());
      } else {
        goForward();
      }

      if (time + DELAY < robot.getTime()) {
        time = robot.getTime();

        data.add(new String[]{
                Double.toString(robot.getTime()).split("\\.")[0],
                df.format(tableMap.getCurrentProgress()),
        });
      }

      fflushIrReceiver();
      step(getTimeStep());
    }

    try {
      CSVUtils.writeToCSVFile(data, "cleaning_basic.csv");
    } catch (IOException ioe) {
      ioe.printStackTrace();
    }
  }

  /**
   * muutetaan robotin tila
   *
   * @param s uusi tila
   */
  public static void changeState(State s) {
    tila = s;
  }

  /**
   * Lokitusfunktio
   */
  public void log() {
    //Loggailut
    System.out.println(tableMap);
    System.out.println("Tila: " + tila);
    System.out.println("Puhdistettu: " + tableMap.getCleaningPercentage() + " %");
    System.out.println("Muutos edelliseen: " + tableMap.getChange() + " %-yksikköä");
    // System.out.println("Lähin likainen alue löytyy indeksistä: " +
    //        Arrays.toString(tableMap.getClosestZero(getGPSLocation())));
    //System.out.println("---");
    //System.out.println("LOG");
    System.out.println("Direction: " + getBearingInDegrees());
    //System.out.println("GPS vector: " + Arrays.toString(gps.getValues()));
    //System.out.println("GPS speed: " + String.format("%.4f", gps.getSpeed()) + " m/s");
    //System.out.println("Compass values: " + Arrays.toString(compass.getValues()));
    //System.out.println("Bearing: " + Math.round(getBearingInDegrees()) + " °");
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

  public boolean isThereACliffAtFrontv2() {
    return cliffSensors[CLIFF_SENSOR_FRONT_LEFT].getValue() < 100.0 &&
            cliffSensors[CLIFF_SENSOR_FRONT_RIGHT].getValue() < 100.0;
  }

  public void goForward() {
    motors[MOTOR_LEFT].setVelocity(MAX_SPEED);
    motors[MOTOR_RIGHT].setVelocity(MAX_SPEED);
  }

  public void goSlightlyLeft() {
    motors[MOTOR_LEFT].setVelocity(MAX_SPEED - 1d);
    motors[MOTOR_RIGHT].setVelocity(MAX_SPEED);
  }

  public void goSlightlyRight() {
    motors[MOTOR_LEFT].setVelocity(MAX_SPEED);
    motors[MOTOR_RIGHT].setVelocity(MAX_SPEED - 1d);
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

  /**
   * 3D-koordinaateista 2D-koordinaatit.
   *
   * @return 2D coordinates
   */
  public double[] getGPSLocation() {
    double[] temp = gps.getValues();
    return new double[]{temp[0], temp[2]};
  }


  /**
   * Kääntää robottia parametrin osoittaman kulman (rad) verran.
   *
   * @param angle
   */
  public void turn(Double angle) {

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
   * Kääntää robotin parametrin osoittamaan suuntaan.
   *
   * @param newDir
   */
  public void turnToDirection(double newDir) {
    double startDir = getBearingInDegrees();
    double angle = newDir - startDir;

    if (angle < -180)
      angle += 360;

    if (angle > 180)
      angle -= 360;

    // käännöskulman etumerkki eli käännöksen suunta
    int sign = 1;
    if (angle < 0)
      sign = -1;

    System.out.println("Nykyinen kulkusuunta on " + Math.round(startDir) + " astetta.");
    System.out.println("Käännös " + Math.round(angle) + " astetta.");

    // Tehdään kompassikäännös.

    double turned = 0;
    stop();
    step();
    motors[MOTOR_LEFT].setVelocity(-sign * HALF_SPEED);
    motors[MOTOR_RIGHT].setVelocity(sign * HALF_SPEED);

    do {
      step();
      turned = getBearingInDegrees() - startDir;

      if (turned < -180)
        turned += 360;

      if (turned > 180)
        turned -= 360;

    } while (Math.abs(turned) < Math.abs(angle));

    stop();
    step();

    System.out.println("Uusi kulkusuunta on " + Math.round(getBearingInDegrees()) + " astetta.");
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
    robot = new Create_avoid_obstacles_ver1();

    System.out.println(robot.tableMap);

    // Main loop
    robot.run();
  }


  /**
   * Sisäluokka TableMap, jolla pidetään kirjaa siivotusta alueesta.
   */
  static class TableMap {

    private double x0 = 2.5;
    private double y0 = 2.5;
    private double currentProgress = 0;
    private double previousProgress = 0;

    private int modCount = 0;

    private final short[][] table = new short[50][50];

    /**
     * Konstuktori
     * Täytetään taulukko nollilla.
     */
    public TableMap() {
      for (short[] shorts : table) {
        Arrays.fill(shorts, (short) 0);
      }
    }


    /**
     * Lisätään koordinaattien mukainen sijainti taulukoon.
     *
     * @param coords
     * @return wasAlreadyCleaned
     */
    public boolean addLocation(double[] coords) {

      double x = coords[0];
      double y = coords[1];

      int i = (int) Math.round((y0 + y) * 10);
      int j = (int) Math.round((x0 + x) * 10);

      // Asetetaan arvo ykköseksi ko. ruudussa ja viereisissä ruuduissa.
      boolean wasAlreadyCleaned = !setValue(i, j);
      setValue(i, j + 1);
      setValue(i - 1, j);
      setValue(i, j - 1);
      setValue(i + 1, j);

      return wasAlreadyCleaned;
    }


    /**
     * Lasketaan kuinka suuri osa alueesta puhdistettu, (modcount / ruutujen määrä) * 100
     *
     * @return alueesta puhdistettu prosentteina
     */
    public String getCleaningPercentage() {
      previousProgress = currentProgress;
      currentProgress = ((double) modCount / (50 * 50)) * 100;
      return String.format("%.2f", currentProgress);
    }

    /**
     * Kuinka paljon muutosta aiemmin tarkistettuun puhdistustasoon
     * TODO aikavälin huomioiminen muutosta laskettaessa
     *
     * @return muutos
     */
    public String getChange() {
      return String.format("%.2f", currentProgress - previousProgress);
    }

    /**
     * Etsitään lähimmän likaisen ruudun indeksi
     * TODO tätä voisi tehostaa lähtemällä etsimään likaisia alueita robotin välittömästä läheisyydestä
     *
     * @param coords robotin nykyiset koordinaatit
     * @return lähimmän likaisen ruudun indeksi
     */
    public int[] getClosestZero(double[] coords) {

      Double minDist = null;
      int[] index = new int[2];

      //etsitään likaiset alueet
      for (int i = 0; i < table.length; ++i) {
        for (int j = 0; j < table[i].length; ++j) {
          if (table[i][j] == 0) {
            double tmp = calculateEuclideanDistance(coords, j, i);

            if (minDist == null || tmp < minDist) {
              minDist = tmp;
              index[0] = i;
              index[1] = j;
            }
          }
        }
      }

      return index;
    }


    /**
     * Lasketaan suuntakulma lähimpään likaiseen ruutuun.
     *
     * @param coords
     * @return
     */
    public double getDirectionToClosestDirty(double[] coords) {

      // nykyinen sijainti
      double x = coords[0];
      double y = coords[1];

      int i0 = (int) Math.round((y0 + y) * 10);
      int j0 = (int) Math.round((x0 + x) * 10);

      // lähimmän likaisen ruudun sijainti
      int[] target = getClosestZero(coords);
      int i1 = target[0];
      int j1 = target[1];

      // Lasketaan suunta.
      double direction = (-Math.atan2(i0 - i1, j1 - j0) + Math.PI / 2) * 180 / Math.PI;

      if (direction < 0)
        direction += 360;

      System.out.println("Nykyinen indeksi on: [" + i0 + ", " + j0 + "]. " +
              "Lähin likainen alue löytyy indeksistä: [" + i1 + ", " + j1 + "].");

      return direction;
    }


    /**
     * Lasketaan "euclidean distance" kahden taulukossa olevan indeksin välillä
     *
     * @param coords robotin koordinaatit
     * @param x2     vertailtava x-koordinaatti
     * @param y2     vertailtava y-koordinaatti
     * @return indeksi, jossa lähin 0.
     */
    public double calculateEuclideanDistance(double[] coords, int x2, int y2) {
      //robotin indeksi
      int x1 = (int) Math.round((x0 + coords[0]) * 10);
      int y1 = (int) Math.round((y0 + coords[1]) * 10);

      //etäisyydet x- ja y-akseleilla
      double ac = Math.abs(y2 - y1);
      double cb = Math.abs(x2 - x1);

      //euklidinen etäisyys
      return Math.hypot(ac, cb);
    }

    public double getCurrentProgress() {
      return currentProgress;
    }


    /**
     * Lasketaan kartan avulla, paljonko on kulma likaisen alueen reunaan.
     * Nykyistä kulkusuuntaan käytetään haussa apuna.
     * Haku kohdistuu vain hieman vasemmalle ja enemmän oikealle.
     *
     * @param coords
     * @param heading
     * @return
     */
    public double getAngleToDirty(double[] coords, double heading) {
      //robotin indeksi
      int j = (int) Math.round((x0 + coords[0]) * 10);
      int i = (int) Math.round((y0 + coords[1]) * 10);

      // Tarkastellaan robotin lähiympäristöä kartasta.
      short[] clean = new short[16];
      clean[0] = getValue(i - 3, j);
      clean[1] = getValue(i - 3, j + 1);
      clean[2] = getValue(i - 2, j + 2);
      clean[3] = getValue(i - 1, j + 3);
      clean[4] = getValue(i, j + 3);
      clean[5] = getValue(i + 1, j + 3);
      clean[6] = getValue(i + 2, j + 2);
      clean[7] = getValue(i + 3, j + 1);
      clean[8] = getValue(i + 3, j);
      clean[9] = getValue(i + 3, j - 1);
      clean[10] = getValue(i + 2, j - 2);
      clean[11] = getValue(i + 1, j - 3);
      clean[12] = getValue(i, j - 3);
      clean[13] = getValue(i - 1, j - 3);
      clean[14] = getValue(i - 2, j - 2);
      clean[15] = getValue(i - 3, j - 1);

      // Lasketaan robotin kulkusuunnan mukainen indeksi - 2.
      int startInd = (int) Math.round(heading / 22.5) - 2;
      if (startInd < 0)
        startInd += clean.length;

      // Aloitetaan robotin kulkusuunnan mukaisesta indeksistä - 2.
      if (clean[startInd] == 1) {
        // Tarkistetaan seuraavat 10 oikealle.
        int ind = 1;

        while (ind <= 10) {
          if (startInd + ind < clean.length) {
            if (clean[startInd + ind] == 0)
              return (startInd + ind) * 22.5;
          } else {  // Otetaan taulukon epäjatkuvuuskohta huomioon.
            if (clean[startInd + ind - clean.length] == 0)
              return (startInd + ind - clean.length) * 22.5;
          }
          ++ind;
        }
      }

      // Ei löydetty muutoskohtaa.
      return 500;  // Palautetaan virhekoodi.

    }


    /**
     * Asetetaan indeksien mukaisen kartan solun arvo ykköseksi, jos se on kartalla.
     * Palautetaan true, jos muutos tehtiin.
     *
     * @param i
     * @param j
     */
    private boolean setValue(int i, int j) {

      // Tarkistetaan lukualueet.
      if (i >= 0 && i < table.length) {
        if (j >= 0 && j < table[i].length) {

          // Onko arvo 1?
          if (table[i][j] != 1) {
            table[i][j] = 1;
            ++modCount;
            return true;
          }

        }
      }
      return false;
    }


    /**
     * Haetaan kartalta indeksien mukainen arvo raja-arvot tarkistaen.
     * Jos sijaintia ei ole kartalla, palautetaan 1.
     *
     * @param i
     * @param j
     * @return
     */
    private short getValue(int i, int j) {
      if (i >= 0 && i < table.length) {

        if (j >= 0 && j < table[i].length)
          return table[i][j];

      }
      return 1;
    }


    /**
     * Taulukon tulostus merkkijonoksi.
     *
     * @return s
     */
    public String toString() {
      String s = "";
      for (int i = 0; i < table.length; ++i) {
        for (int j = 0; j < table[i].length; ++j) {
          s += table[i][j] + " ";
        }
        s += "\n";  // rivinvaihto
      }

      return s;

    }


  }

  /**
   * Start = aloitus
   * Edges = reunojen siivous
   * Center = iterointi kohti keskustaa, kjeh keskusta :D
   * Seek = Jäljelläolevien likaisten kohtien etsiminen
   * Done = siivottu
   */
  enum State {
    START,
    EDGES,
    CENTER,
    SEEK,
    DONE
  }

  static class CSVUtils {

    private CSVUtils() {
    }

    /**
     * Lista merkkijonotaulukoita csv-tiedostoksi
     *
     * @param lines datalista
     */
    public static void writeToCSVFile(List<String[]> lines, String filename) throws IOException {
      //luodaan tiedosto
      File csvOutput = new File(filename);

      //kirjoitetaan
      try (PrintWriter pw = new PrintWriter(csvOutput)) {
        lines.stream()
                .map(CSVUtils::convertLineToCSV)
                .forEach(pw::println);
      }
    }

    /**
     * Muunnetaan merkkijonotaulukko csv-riviksi
     *
     * @param data taulukko
     * @return csv-rivi merkkijonomuotoisena
     */
    public static String convertLineToCSV(String[] data) {
      return Arrays.stream(data)
              .map(element -> element.replace(",", "."))
              .collect(Collectors.joining(","));
    }

  }
}
