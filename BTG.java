package roboticsUtility;

import java.util.ArrayList;

public class BTG {
	//Ball Trajectory Generator
	
	double velocity = 1; // meters/sec
	double velocityMax = 15;
	double velIter = 0.05;
	double angle = 15;
	double angleMax = 75;
	double angleIter = 1;
	double simIterPS = 100;
	
	//Targets
	double radius = 0.127; //3.5 inch to meters, but using 5 as a 1.5 inch tolerance
	double t1tol = 0.235; //Additional tolerance for t1
	double t1tol3d = 0.1; //Additional tolerance for t1 when 3d targets
	double height = 2.49555; //98.25 inch to meters
	double t1size = 0.762;//30 inches to meters
	double t2size = 0.3302;//13 inches to meters
	double t1t2dist = 0.7493; //29.5 inches to meters
	double distance; //distance to target in meters
	
	double compressedCirc = (7 - 2.25) * Math.PI * (0.3048 / 12);
	double gearRatio = 9/16;
	double sizeT = 1.25 * 2 * Math.PI * (0.3048 / 12);
	double sizeB = 2 * 2 * Math.PI * (0.3048 / 12);
	double spinvel; //RPM, for holding
	
	ArrayList<Point> trajectory = new ArrayList<>();
	double trajectoryAngle;
	double trajectoryVelocity;

	ProjectileSimulation sim;

	boolean t1pass(double y, boolean t3d) {
		if(!t3d && ((height + t1size/2) - radius - t1tol > y && (height - t1size/2) + radius + t1tol < y)) return true;
		if(t3d && ((height + t1size/2) - radius - t1tol3d > y && (height - t1size/2) + radius + t1tol3d < y)) return true;
		return false;
	}
	boolean t2pass(double y) {
		if((height + t2size/2) - radius > y && (height - t2size/2) + radius < y) return true;
		return false;
	}
	double rpm(double velocity) {
		double prSpeed = 2 * velocity / (sizeB + sizeT * gearRatio); //vel = (prspeed + srspeed )/2 = (prspeed * sizeB + prspeed * sizeT *gratio)/2,  2*vel = prspeed(sizeB + sizeT * gratio)
		double srSpeed = prSpeed * gearRatio * (sizeT / sizeB);
		
		return (prSpeed - srSpeed) / compressedCirc;
	}
	double bottomwheel(double velocity) {
		return 60 * 2 * velocity / (sizeB + sizeT * gearRatio) / sizeB;
	}
	void printTargets(boolean target3d) {
		System.out.println(("((1-t)(" + distance + ") + t(" + distance + "), (1-t)(" + (height + t1size/2) + ") + t(" + (height - t1size/2) + "))"));
		if(target3d) System.out.println(("((1-t)(" + (distance + t1t2dist) + ") + t(" + (distance + t1t2dist) + "), (1-t)(" + (height + t2size/2) + ") + t(" + (height - t2size/2) + "))"));
	}
	
	void simulate(boolean printTrajectory, boolean printIC, boolean target3d) {
		boolean found = false;
		for(double v = velocity; v < velocityMax && !found; v += velIter) {
			for(double a = angle; a < angleMax && !found; a += angleIter) {
				spinvel = rpm(v);
				sim = new ProjectileSimulation(a, v, spinvel);
				sim.y = 0.4572; //39.7 inches to meters
				sim.simulate(simIterPS);
				
				int p1 = -1;
				int p2 = -1;
				for(int i = 0; i < sim.data.size(); i ++) {
					if(sim.data.get(i).x > distance && p1 == -1) {
						p1 = i;
					}
					if(sim.data.get(i).x > distance + t1t2dist && p2 == -1) {
						p2 = i;
					}
				}
				
				if(p1 != -1 && p2 != -1 && p2 < sim.data.size()) {
					line one = new line();
					line two = new line();
					
					one.setPointSlope((sim.data.get(p1).y - sim.data.get(p1 - 1).y)/(sim.data.get(p1).x - sim.data.get(p1 - 1).x), sim.data.get(p1).x, sim.data.get(p1).y);
					two.setPointSlope((sim.data.get(p2).y - sim.data.get(p2 - 1).y)/(sim.data.get(p2).x - sim.data.get(p2 - 1).x), sim.data.get(p2).x, sim.data.get(p2).y);
					
					if((t1pass(one.findYAtX(distance), target3d) && !target3d) || (t1pass(one.findYAtX(distance), target3d) && t2pass(two.findYAtX(distance + t1t2dist)))) {
						trajectory = sim.data;
						trajectoryAngle = a;
						trajectoryVelocity = v;
						found = true;
					}
				}
			}
		}
		if(found == false) {
			System.out.println("Nothing Found ?XD");
		}else {
			if(printTrajectory) {
				DesmosCopyGen printer = new DesmosCopyGen(trajectory);
				printer.printWithLine();
			}
			if(printIC) {
				System.out.println("Distance: " + distance);
				System.out.println("Spin Vel: " + spinvel);
				System.out.println("Bottom Roller Vel: " + bottomwheel(trajectoryVelocity));
				System.out.println("Angle: " + trajectoryAngle);
				System.out.println("Velocity: " + trajectoryVelocity);
			}
		}
		
	}
	
	BTG(double distance){
		this.distance = distance;
	}
	
	class line{
		double b;
		double m;
		
		double x1;
		double x2;
		
		double x1() {
			double termSum = m * Math.pow(x1, 2)/2 + x1 * b;
			return Math.abs(termSum/(-1));
		}
		double x2() {
			double termSum = m * Math.pow(x2, 2)/2 + x2 * b;
			return Math.abs(termSum/(-1));
		}
		double area() {
			return x2() - x1();
		}
		double area(double x1, double x2) {
			this.x1 = x1;
			this.x2 = x2;
			return x2() - x1();
		}
		void setPointSlope(double tM, double tX, double tY) {
			//y - tY = m (x - tX)
			//y = mx - mtX + tY
			m = tM;
			b = -m * tX + tY;
		}
		void setSlopeIntercept(double tM, double tB) {
			m = tM;
			b = tB;
		}
		double findYAtX(double x) {
			return m * x + b;
		}
		double findXAtY(double y) {
			return (y - b) / m;
		}
		//https://www.mathsisfun.com/calculus/integration-rules.html
	}
}
