package roboticsUtility;

import java.util.ArrayList;

public class ProjectileSimulation {
	double radius = 0.1778/2; // 3.5 Inch to meters
	
	double x = 0;
	double y = 0;
	double xVel;
	double yVel;
	double xAcc;
	double yAcc;
	
	double spinVel;
	
	double gravity = 9.81;
	double mass = 0.14174; //In KG btw
	double liftCoefficient = 0.2;
	double dragCoefficient = 0.47; //https://en.wikipedia.org/wiki/Drag_coefficient
	double airDensity = 1.225; //At Sea Level
	double crossSectionArea = radius * radius * Math.PI; //Just Circle
	
	ArrayList<Point> data = new ArrayList<>();
	

	
	void simulate(double iterPerSec) {
		for(double t = 0; y >= 0 || t > 1000; t += 1/iterPerSec) {
			data.add(new Point(x, y));
			x += xVel * 1/iterPerSec;
			y += yVel * 1/iterPerSec;
			
			xVel += xAcc * 1/iterPerSec;
			yVel += yAcc * 1/iterPerSec;
			
			double magnusX = Math.sin(velAngRad()) * magnusforce();
			double magnusY = Math.cos(velAngRad()) * magnusforce();
			
			xAcc += (- magnusX - dragforce()) * 1/iterPerSec;
			yAcc += (magnusY - gravity) * 1/iterPerSec;
		}
	}
	
	double dragforce() {
		return (dragCoefficient * crossSectionArea * airDensity * velocity() * velocity()) / 2;
	}
	double magnusforce() {
		return liftCoefficient * 16/3 * (Math.PI * Math.PI * radius * radius * radius * (spinVel / 60) * airDensity * velocity());
		
//		//Average Height of Semicircle = pi/2 * radius
//		double vortexForce = 2 * Math.PI * spinVel * (Math.PI / 2 * radius);
//		double unitForce = velocity() * airDensity * vortexForce;
//		return unitForce * radius;
	}
	double spindragforce() {
		//https://physics.stackexchange.com/questions/304742/angular-drag-on-body#:~:text=Each%20tiny%20surface%20area%20d,T%3DdF%C3%97r.
		return 0;
	}
	
	double r2d(double radian) {
		return radian * 180 / Math.PI;
	}
	double d2r(double degree) {
		return degree * Math.PI / 180;
	}
	double velocity() {
		return Math.hypot(xVel, yVel);
	}
	double velAngRad() {
		return Math.atan2(xVel, yVel);
	}
	double randRange(double min, double max) {
		return min + (max - min) * Math.random();
	}
	ProjectileSimulation(double angle, double velocity, double acceleration, double spinVel){
		xVel = velocity * Math.cos(d2r(angle));
		yVel = velocity * Math.sin(d2r(angle));
		xAcc = acceleration * Math.cos(d2r(angle));
		yAcc = acceleration * Math.sin(d2r(angle));
		this.spinVel = spinVel / (2 * Math.PI);
	}
	ProjectileSimulation(double angle, double velocity, double spinVel){
		//Spin Vel is input as rpm btw
		xVel = velocity * Math.cos(d2r(angle));
		yVel = velocity * Math.sin(d2r(angle));
		xAcc = 0;
		yAcc = 0;
		this.spinVel = spinVel * (2 * Math.PI) / 60;
	}
}
