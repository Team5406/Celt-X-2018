package org.cafirst.frc.team5406.auto;

import java.awt.geom.Point2D;
import java.util.ArrayList;

import org.cafirst.frc.team5406.robot.Constants;

public class MotionProfile {

	
	   public ArrayList<double[]> motionProfile = new ArrayList<double[]>();
		ArrayList<Double> pathLeft = new ArrayList<Double>();
		ArrayList<Double> pathRight = new ArrayList<Double>();
		ArrayList<Double> pathAngles = new ArrayList<Double>();
		double totalDistanceLeft = 0;
		double totalDistanceRight = 0;
	
	public Point2D[] findControlPoints(Point2D s1, Point2D s2, Point2D s3) {
		Point2D[] controlPoints = new Point2D[2];
		double l1 = s1.distance(s2);
		double l2 = s2.distance(s3);
		Point2D m1 = new Point2D.Double((s1.getX() + s2.getX()) / 2.0, (s1.getY() + s2.getY()) / 2.0);
		Point2D m2 = new Point2D.Double((s2.getX() + s3.getX()) / 2.0, (s2.getY() + s3.getY()) / 2.0);
		double dxm = m1.getX() - m2.getX();
		double dym = m1.getY() - m2.getY();
		double k = l2 / (l1 + l2);
		Point2D cm = new Point2D.Double(m2.getX() + dxm * k, m2.getY() + dym * k);
		double tx = s2.getX() - cm.getX();
		double ty = s2.getY() - cm.getY();
		controlPoints[0] = new Point2D.Double(m1.getX() + tx, m1.getY() + ty);
		controlPoints[1] = new Point2D.Double(m2.getX() + tx, m2.getY() + ty);
		return controlPoints;
	}

	public ArrayList<Point2D> plotBezierQuad(Point2D s1, Point2D s2, Point2D s3, Point2D s4) {

		ArrayList<Point2D> leftWheel = new ArrayList<Point2D>();
		ArrayList<Point2D> rightWheel = new ArrayList<Point2D>();

		Point2D[] S1 = findControlPoints(s1, s2, s3);
		Point2D[] S2 = findControlPoints(s2, s3, s4);
		double l1 = Math.floor(s1.distance(s2));
		double l2 = Math.floor(s2.distance(s3));
		double l3 = Math.floor(s3.distance(s4));

		double step = 1;

		int ctr = (int) Math.floor(((l1 > 0 ? l1 : l2) + (l3 > 0 ? l3 : l2)) / step); // scale by step to change number
																						// of points interpolated
		Point2D p1 = (Point2D) s2.clone();
		Point2D p2 = S1[1];
		Point2D p3 = S2[0];
		Point2D p4 = (Point2D) s3.clone();

		// Now do actual bezier math
		Point2D pf = p1;
		double ss = 1.0 / (ctr + 1), ss2 = ss * ss, ss3 = ss2 * ss, pre1 = 3.0 * ss, pre2 = 3.0 * ss2, pre4 = 6.0 * ss2,
				pre5 = 6.0 * ss3, tmp1x = p1.getX() - p2.getX() * 2.0 + p3.getX(),
				tmp1y = p1.getY() - p2.getY() * 2.0 + p3.getY(),
				tmp2x = (p2.getX() - p3.getX()) * 3.0 - p1.getX() + p4.getX(),
				tmp2y = (p2.getY() - p3.getY()) * 3.0 - p1.getY() + p4.getY(),
				dfx = (p2.getX() - p1.getX()) * pre1 + tmp1x * pre2 + tmp2x * ss3,
				dfy = (p2.getY() - p1.getY()) * pre1 + tmp1y * pre2 + tmp2y * ss3, ddfx = tmp1x * pre4 + tmp2x * pre5,
				ddfy = tmp1y * pre4 + tmp2y * pre5, dddfx = tmp2x * pre5, dddfy = tmp2y * pre5;
		// System.out.println(ctr);
		double m_right;
		double dy_right;
		double dx_right;
		double distanceLeft = 0;
		double distanceRight = 0;
		double angle = 0;
		boolean firstRight = true;
		Point2D pt2 = (Point2D) pf.clone();
		Point2D last_pt = (Point2D) pt2.clone();
		while (ctr > 0) {
			ctr--;
			pf.setLocation(pf.getX() + dfx, pf.getY() + dfy);
			m_right = -1 * dfx / dfy;
			dx_right = Constants.WHEEL_BASE / Math.sqrt(m_right * m_right + 1);
			dy_right = m_right * dx_right;
			distanceLeft = Math.sqrt(dfx * dfx + dfy * dfy) * Constants.revPerInch;

			if (dfy < 0) {
				pt2.setLocation(pf.getX() - dx_right, pf.getY() - dy_right);
			} else {
				pt2.setLocation(pf.getX() + dx_right, pf.getY() + dy_right);
			}
			if(firstRight) {
				firstRight = false;
				last_pt.setLocation(pt2.getX(), pt2.getY());
			}
			//System.out.println("Rpt: " + pt2.getX() + ", " + pt2.getY() + ", " + last_pt.getX() + ", " + last_pt.getY());
			distanceRight = pt2.distance(last_pt)*Constants.revPerInch;
			last_pt.setLocation(pt2.getX(), pt2.getY());
			
			totalDistanceLeft += distanceLeft;
			totalDistanceRight += distanceRight;
			pathLeft.add(distanceLeft);
			pathRight.add(distanceRight);
			rightWheel.add(pt2);
			dfx += ddfx;
			dfy += ddfy;
			ddfx += dddfx;
			ddfy += dddfy;
			pathAngles.add(Math.toDegrees(Math.atan2(dfx, dfy)));
			leftWheel.add(pf);

			//System.out.println(pf.getX() + ", " + pf.getY() + ", " + pt2.getX() + ", " + pt2.getY());
		}
		pf.setLocation(p4.getX(), p4.getY());
		leftWheel.add(pf);
		m_right = -1 * dfx / dfy;
		dx_right = Constants.WHEEL_BASE / Math.sqrt(m_right * m_right + 1);
		dy_right = m_right * dx_right;
		if (dfy < 0) {
			pt2.setLocation(pf.getX() - dx_right, pf.getY() - dy_right);
		} else {
			pt2.setLocation(pf.getX() + dx_right, pf.getY() + dy_right);
		}
		rightWheel.add(pt2);
		//System.out.println(pf.getX() + ", " + pf.getY() + ", " + pt2.getX() + ", " + pt2.getY());

		return leftWheel;
	}


	public void bezierPoints(ArrayList<Point2D> pts, double angleIn, double angleOut) {

		motionProfile = new ArrayList<double[]>();
		pathLeft = new ArrayList<Double>();
		pathRight = new ArrayList<Double>();
		pathAngles = new ArrayList<Double>();
		
		/*
		 * duplicate first and last points loop over sets of 4 points 0...3; 1...4, etc.
		 */
		// System.out.println("BezierPoints Function");
		if (pts.size() >= 2) {
			Point2D startControl = new Point2D.Double(pts.get(1).getX() + (2*(pts.get(0).getY() - pts.get(1).getY())*(Math.sin(angleIn*Math.PI/180))),
					pts.get(1).getY() + (2*(pts.get(0).getY() - pts.get(1).getY())*(Math.cos(angleIn*Math.PI/180))));
			Point2D endControl  = new Point2D.Double(pts.get(pts.size()-2).getX() + (2*(pts.get(pts.size()-1).getY() - pts.get(pts.size()-2).getY())*(Math.sin(angleOut*Math.PI/180))),
					pts.get(pts.size()-2).getY() + (2*(pts.get(pts.size()-1).getY() - pts.get(pts.size()-2).getY())*(Math.cos(angleOut*Math.PI/180))));


			
			pts.add(0, startControl);
			pts.add(endControl);
			int pt_i = 0;
			do {
				plotBezierQuad(pts.get(pt_i), pts.get(pt_i + 1), pts.get(pt_i + 2), pts.get(pt_i + 3));
				pt_i++;
			} while ((pt_i + 3) < pts.size());
			
			double distanceLeft = 0;
			double distanceRight = 0;
			double accDistance = 0;
			double decDistance = 0;
			double targetSpeed = Constants.maxSpeed;
			double timeLeft = 0;
			double maxDistance =0;
			double totalDistance =0;
			double speedLeft=0, speedRight = 0;
			double cDistanceLeft=0, cDistanceRight = 0;
			double timeRight = 0;
			double accRevs=2;
			double angle = 0;
			double[] motionProfileData = new double[6];

			totalDistance = Math.max(totalDistanceLeft, totalDistanceRight);

			for (int i = 0; i < pathLeft.size(); i++) {

				distanceLeft = pathLeft.get(i);
				distanceRight = pathRight.get(i);
				cDistanceLeft+=distanceLeft;
				cDistanceRight+=distanceRight;

				if(totalDistance < 2*accRevs) {
					accDistance = totalDistance/2;
					decDistance = accDistance;
				}else {
					accDistance = accRevs;
					decDistance = totalDistance - accRevs;
				}
				
				maxDistance = Math.max(cDistanceLeft, cDistanceRight);
				if(maxDistance < accDistance) {
					targetSpeed = Constants.minSpeed+(Constants.maxSpeed-Constants.minSpeed)*maxDistance/accRevs;
				}else if (maxDistance>=accDistance && maxDistance<=decDistance) {
					targetSpeed=Constants.maxSpeed;
				}else {
					targetSpeed = Constants.minSpeed+(Constants.maxSpeed-Constants.minSpeed)*(totalDistance-maxDistance)/accRevs;
				}
				//targetSpeed = maxSpeed;
				
				if(distanceLeft > distanceRight) {
					speedLeft = targetSpeed;
					timeLeft = 60000 * distanceLeft / speedLeft;
					timeRight = timeLeft;
					speedRight = 60000*distanceRight/timeRight;
				}else {
					speedRight = targetSpeed;
					timeRight = 60000 * distanceRight / speedRight;
					timeLeft = timeRight;
					speedLeft = 60000*distanceLeft/timeLeft;
				}
				
				motionProfileData = new double[6];
				motionProfileData[0] = timeLeft;
				motionProfileData[1] = cDistanceLeft;
				motionProfileData[2] = speedLeft;
				motionProfileData[3] = cDistanceRight;
				motionProfileData[4] = speedRight;
				motionProfileData[5] = pathAngles.get(i) - angleIn;

				motionProfile.add(motionProfileData);

			}
			


		}
	}
	
}
