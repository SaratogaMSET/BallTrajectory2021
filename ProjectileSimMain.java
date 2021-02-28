package roboticsUtility;

public class ProjectileSimMain {

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		BTG sim = new BTG(25 * 0.3048); //Distance
		boolean t3d = false;
		sim.simulate(true, true, t3d);
		sim.printTargets(t3d);
	}

}
