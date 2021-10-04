import lejos.hardware.Sound;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

public class Checksensor implements Runnable {

	private double maxSensorRange;
	private EV3UltrasonicSensor ultraSensor;
	private EV3GyroSensor gyrosEnsor;
	private boolean exit = false;
	private Thread t;
	private float[] objectGyrosAngleArray = new float[0];
	private float[] objectDistanceArray = new float[0];

	public Checksensor(EV3GyroSensor gyrosEnsor, EV3UltrasonicSensor ultraSensor, double maxSensorRange) {
		this.ultraSensor = ultraSensor;
		this.gyrosEnsor = gyrosEnsor;
		this.maxSensorRange = maxSensorRange;
		t = new Thread(this);
		t.setDaemon(true);
		t.start();
	}

	public void run() {
		int counter = 0;
		while (!exit) {
			float distanceInfo = getDistanceInfo(ultraSensor);
			if (distanceInfo < maxSensorRange) {
				Sound.beep();
				// vergrößere das Array objectGyrosAngle um 1
				float[] copyObjectGyrosAngle = new float[objectGyrosAngleArray.length + 1];
				System.arraycopy(objectGyrosAngleArray, 0, copyObjectGyrosAngle, 0, objectGyrosAngleArray.length);
				objectGyrosAngleArray = copyObjectGyrosAngle;
				// füge aktuellen Winkel zum Objekt hinzu
				gyrosEnsor.getAngleMode().fetchSample(objectGyrosAngleArray, counter);
				// vergrößere das Array objectDistance um 1
				float[] copyObjectDistance = new float[objectDistanceArray.length + 1];
				System.arraycopy(objectDistanceArray, 0, copyObjectDistance, 0, objectDistanceArray.length);
				objectDistanceArray = copyObjectDistance;
				// füge aktuelle Distanz zum Objekt hinzu
				objectDistanceArray[counter] = distanceInfo;
				counter++;
				// Cooldown
				try {
					Thread.sleep(1500);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}

		}
	}

	public float[] getGyrosAngle() {
		return objectGyrosAngleArray;
	}

	public float[] getDistance() {
		return objectDistanceArray;
	}

	public void stop() {
		exit = true;
	}

	public float getDistanceInfo(EV3UltrasonicSensor ultraSensor) {
		float[] distance = { 0 };
		SampleProvider ultraSensorSample = ultraSensor.getDistanceMode();
		ultraSensorSample.fetchSample(distance, 0);
		return distance[0];
	}
}
