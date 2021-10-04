import lejos.hardware.Button;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.navigation.Navigator;

@SuppressWarnings("deprecation")
public class Main {

	// individuelle Nutzereingaben zum Anpassen an die Testumgebung
	private int rotationAngle = 180;
	private double maxSensorRange = 0.7;

	private DifferentialPilot diffPilot;
	private Navigator navi;
	private EV3UltrasonicSensor ultraSensor;
	private EV3GyroSensor gyrosEnsor;
	private Checksensor checksensorThread;
	private float[] gyrosAngleArray = {};
	private float[] distanceArray = {};

	public Main() {
		ultraSensor = new EV3UltrasonicSensor(SensorPort.S4);
		gyrosEnsor = new EV3GyroSensor(SensorPort.S1);
	}

	public static void main(String[] args) {
		Button.ENTER.waitForPress();
		Main labor = new Main();
//		labor.diffPilot = new DifferentialPilot(56, 110, Motor.B, Motor.C);
		labor.diffPilot = new DifferentialPilot(56, 130, Motor.B, Motor.C);
		labor.diffPilot.setAngularSpeed(30);
		labor.diffPilot.setLinearAcceleration(80);
		labor.navi = new Navigator(labor.diffPilot);
		labor.checksensorThread = new Checksensor(labor.gyrosEnsor, labor.ultraSensor, labor.maxSensorRange);
		labor.diffPilot.rotate(-labor.rotationAngle);
		labor.gyrosAngleArray = labor.checksensorThread.getGyrosAngle();
		labor.distanceArray = labor.checksensorThread.getDistance();
		labor.checksensorThread.stop();

		// Berechnung der Koordinaten der Objekte
		float[] xCoords = new float[labor.gyrosAngleArray.length];
		float[] yCoords = new float[labor.gyrosAngleArray.length];
		for (int i = 0; i < labor.gyrosAngleArray.length; i++) {
			xCoords[i] = (-labor.distanceArray[i] * (float) Math.cos(Math.toRadians(labor.gyrosAngleArray[i])));
			yCoords[i] = (-labor.distanceArray[i] * (float) Math.sin(Math.toRadians(labor.gyrosAngleArray[i])));
			System.out.println("(" + xCoords[i] * 100 + " / " + yCoords[i] * 100 + ")");
		}

		// Zentroid der Koordinaten berechnen
		float centerX = 0;
		float centerY = 0;
		for (int i = 1; i < xCoords.length; i++) {
			centerX = centerX + xCoords[i];
			centerY = centerY + yCoords[i];
		}
		centerX = centerX / (float) xCoords.length;
		centerY = centerY / (float) yCoords.length;

		// berechneten Zentroid als Wegpunkt hinzufügen
		labor.navi.addWaypoint(-centerX * 1000, -centerY * 1000);

		// Fahrt zum Wegpunkt antreten
		while (!labor.navi.pathCompleted()) {
			labor.navi.followPath();
		}
	}

}
