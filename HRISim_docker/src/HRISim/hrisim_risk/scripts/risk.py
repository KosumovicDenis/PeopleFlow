#!/usr/bin/env python

import rospy
from shapely.geometry import *
import math
import message_filters
from hrisim_risk.msg import Risk
from peopleflow_msgs.msg import RobotState, HumanState
from std_msgs.msg import Header

NODE_NAME = "hrisim_risk"
NODE_RATE = 10 # [Hz]

def compute_risk(subject: Point, obstacle: Point, subject_v: Point, obstacle_v: Point):
    # risk = math.sqrt(subject_v.x**2 + subject_v.y**2)
    collision = False
    
    Vrel = Point(obstacle_v.x - subject_v.x, obstacle_v.y - subject_v.y)
    
    try:
        slope_AB = (obstacle.y - subject.y) / (obstacle.x - subject.x)
    except ZeroDivisionError:
        slope_AB = 0.0001

    if slope_AB == 0:
        slope_AB = 0.0001
    slope_PAB = -1 / slope_AB

    delta_x = OBS_SIZE / (1 + slope_PAB ** 2) ** 0.5
    delta_y = delta_x * slope_PAB
    
    left = Point(obstacle.x - delta_x, obstacle.y - delta_y)
    right = Point(obstacle.x + delta_x, obstacle.y + delta_y)
    
    cone_origin = Point(subject.x, subject.y)
    cone = Polygon([cone_origin, left, right])
    
    # P = Point(cone_origin.x + subject_v.x, cone_origin.y + subject_v.y)
    # The 1s-lookahead point P must fall inside the cone, whose base lies at
    # the obstacle distance: clamp the displacement so that relative speeds
    # larger than the distance cannot overshoot past the base and miss it.
    v_rel_norm = math.sqrt(Vrel.x**2 + Vrel.y**2)
    dist = subject.distance(obstacle)
    scale = min(1.0, 0.9 * dist / v_rel_norm) if v_rel_norm > 0 else 1.0
    P = Point(cone_origin.x - Vrel.x * scale, cone_origin.y - Vrel.y * scale)

    collision = P.within(cone) and dist < SAFE_DIST

    # --- old risk formula, kept for reference ---
    # risk = 1 / (abs(subject.x - obstacle.x) + abs(subject.y - obstacle.y))
    # if collision:
    #     if v_rel_norm > 0:
    #         time_collision_measure = dist / v_rel_norm
    #         steering_effort_measure = min(P.distance(LineString([cone_origin, left])), P.distance(LineString([cone_origin, right])))
    #         risk = risk + 1/time_collision_measure + steering_effort_measure
    # risk = math.exp(risk)

    # Risk w.r.t. the subject (the agent standing at the center): proximity
    # inside SAFE_DIST plus closing speed (inverse time-to-collision), then
    # normalized to [0, 1). Zero when the obstacle is far or moving away.
    if dist > 0:
        ux = (obstacle.x - subject.x) / dist
        uy = (obstacle.y - subject.y) / dist
        v_closing = max(0.0, -(Vrel.x * ux + Vrel.y * uy))
        risk_prox = max(0.0, SAFE_DIST / dist - 1.0)
        risk_ttc = v_closing / dist
        risk = 1.0 - math.exp(-(W_PROX * risk_prox + W_TTC * risk_ttc))
    else:
        risk = 1.0

    return risk, collision, cone_origin, left, right


class RiskClass():
    
    def __init__(self) -> None:
        """
        RiskClass constructor
        """
        self.firstcb = True
        self.AgentA = None
        self.AgentAv = None
        self.AgentB = None
        self.AgentBv = None

        # Parameter to decide the subject: 0 for Human, 1 for Robot
        self.subject_mode = int(rospy.get_param("~subject", 0))
        
        self.pub_risk = rospy.Publisher('/hri/risk', Risk, queue_size=10)
        
        sub_robot = message_filters.Subscriber("/roscausal/robot", RobotState)
        sub_people = message_filters.Subscriber('/roscausal/human', HumanState)
        
        self.ats = message_filters.ApproximateTimeSynchronizer([sub_robot, 
                                                                sub_people], 
                                                                queue_size = 10, slop = 0.1,
                                                                allow_headerless = True)
        self.ats.registerCallback(self.cb_risk)
        
    def extract_data(self, robot, person):
        r_pos = Point(robot.pose2D.x, robot.pose2D.y)
        r_vel = Point(robot.twist.linear.x, robot.twist.linear.y)
        
        h_pos = Point(person.pose2D.x, person.pose2D.y)
        h_vel = Point(person.twist.linear.x, person.twist.linear.y)
        
        if self.subject_mode == 1:
            # Robot is the subject, Human is the obstacle
            self.AgentA, self.AgentAv = r_pos, r_vel
            self.AgentB, self.AgentBv = h_pos, h_vel
        else:
            # Human is the subject, Robot is the obstacle (Default)
            self.AgentA, self.AgentAv = h_pos, h_vel
            self.AgentB, self.AgentBv = r_pos, r_vel

    def cb_risk(self, robot: RobotState, person: HumanState):
        """
        Synchronized callback
        """
        if self.firstcb:
            self.extract_data(robot, person)
            self.firstcb = False
        else:
            # Risk calculation: AgentA is the subject, AgentB is the obstacle
            risk, collision, origin, left, right = compute_risk(self.AgentA, self.AgentB, self.AgentAv, self.AgentBv)
            
            msg = Risk()
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            
            msg.risk.data = risk
            msg.collision.data = collision
            msg.origin.x = origin.x
            msg.origin.y = origin.y
            msg.origin.z = 0
            msg.left.x = left.x
            msg.left.y = left.y
            msg.left.z = 0
            msg.right.x = right.x
            msg.right.y = right.y
            msg.right.z = 0
            self.pub_risk.publish(msg)
            
            self.extract_data(robot, person)

if __name__ == '__main__':
    # Node
    rospy.init_node(NODE_NAME, anonymous=True)

    SAFE_DIST = float(rospy.get_param("/hri/safe_distance", default = 2.3))
    OBS_SIZE = float(rospy.get_param("/hri/obs_size", default = 1))
    W_PROX = float(rospy.get_param("/hri/risk_w_prox", default = 1.0))
    W_TTC = float(rospy.get_param("/hri/risk_w_ttc", default = 1.0))
        
    r = RiskClass()

    rospy.spin()
