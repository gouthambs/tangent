import gizeh as gz
from math import pi as PI
from math import sin, cos
import numpy as np


THETA_TO_RADIAN = PI/180





class StickFigure:

    def __init__(self,  height=200, width_scale=1, stroke_width=5):
        head_height = int(height/8)
        self.head_height = head_height
        self.width_scale = width_scale
        self.stroke_width = stroke_width
        self.neck_height = head_height / 2
        self.head_width = int(2 * head_height / 3)
        self.body_height = 1.5 * head_height
        self.hip_height = head_height
        self.joint_height = head_height / 4
        self.shoulder_width = 1.5 * head_height * width_scale
        self.waist_width = 1.0 * head_height * width_scale
        self.hip_width = 1.25 * head_height * width_scale
        self.hip_height = head_height
        self.arm_height1 = 1.5 * head_height - self.joint_height
        self.arm_height2 = head_height + self.joint_height
        self.arm_height3 = head_height - self.joint_height
        self.leg_height1 = 2 * head_height - self.joint_height
        self.leg_height2 = 2 * head_height
        self.leg_height3 = head_height - self.joint_height

    def _compute_xyz(self, left_arm_theta1, left_arm_theta2, left_arm_theta3, left_arm_phi,
                     right_arm_theta1, right_arm_theta2, right_arm_theta3, righ_arm_phi,
                     left_leg_theta1, left_leg_theta2, left_leg_theta3,
                     right_leg_theta1, right_leg_theta2, right_leg_theta3,
                     body_yaw_degree, body_pitch_degree):
        # coordinate system as shown
        # z +---> x
        #   |
        # y v
        #
        # stick man reference
        # right <-----> left
        yaw_matrix = self._yaw_matrix(body_yaw_degree) # rotate about z
        pitch_matrix = self._pitch_matrix(body_pitch_degree) # rotate about y
        body_rotation = np.matmul(pitch_matrix, yaw_matrix)

        # first compute points in plane
        left_arm_pitch_matrix = self._pitch_matrix(left_arm_phi) # roations for out of plane
        left_arm_rotation = np.matmul(body_rotation, left_arm_pitch_matrix)
        left_arm_yaw_matrix1 = np.matmul(left_arm_rotation, self._yaw_matrix(-left_arm_theta1))
        left_arm_yaw_matrix2 = np.matmul(left_arm_rotation,
                                         self._yaw_matrix(-(left_arm_theta1 + 180 - left_arm_theta2))
                                         )
        left_arm_yaw_matrix3 = np.matmul(left_arm_rotation,
                                          self._yaw_matrix(-(left_arm_theta1 + 360 -
                                                             left_arm_theta2 - left_arm_theta3))
                                          )
        # with belly button as origin
        self._head_center_xyz = np.matmul(body_rotation, np.array([0, - 2.5 * self.head_height, 0]))
        self._neck_top_xyz = np.matmul(body_rotation, np.array([0, -2 * self.head_height, 0]))
        neck_bottom_height = -2 * self.head_height + self.neck_height
        self._neck_bottom_xyz = np.matmul(body_rotation, np.array([0, neck_bottom_height, 0]))
        self._left_shoulder_xyz = np.matmul(body_rotation, np.array([self.shoulder_width/2, neck_bottom_height, 0]))
        self._right_shoulder_xyz = np.matmul(body_rotation, np.array([-self.shoulder_width/2, neck_bottom_height, 0]))
        self._left_waist_xyz = np.matmul(body_rotation, np.array([self.waist_width / 2, 0, 0]))
        self._right_waist_xyz = np.matmul(body_rotation, np.array([-self.waist_width / 2, 0, 0]))
        self._left_hip_xyz = np.matmul(body_rotation, np.array([self.hip_width/2, self.head_height, 0]))
        self._right_hip_xyz = np.matmul(body_rotation, np.array([self.hip_width / 2, -self.head_height, 0]))

        #self._left_elbow_xyz




        #
        # left_arm_angle1 = left_arm_theta1 * THETA_TO_RADIAN
        # left_arm_angle2 = left_arm_theta2 * THETA_TO_RADIAN
        # left_arm_angle3 = left_arm_theta3 * THETA_TO_RADIAN
        # right_arm_angle1 = right_arm_theta1 * THETA_TO_RADIAN
        # right_arm_angle2 = right_arm_theta2 * THETA_TO_RADIAN
        # right_arm_angle3 = right_arm_theta3 * THETA_TO_RADIAN
        #
        # left_leg_angle1 = left_leg_theta1 * THETA_TO_RADIAN
        # left_leg_angle2 = left_leg_theta2 * THETA_TO_RADIAN
        # left_leg_angle3 = left_leg_theta3 * THETA_TO_RADIAN
        # right_leg_angle1 = right_leg_theta1 * THETA_TO_RADIAN
        # right_leg_angle2 = right_leg_theta2 * THETA_TO_RADIAN
        # right_leg_angle3 = right_leg_theta3 * THETA_TO_RADIAN
        #
        # # body_angle = body_theta1 * THETA_TO_RADIAN
        #
        # height_to_body = self.head_height + self.neck_height
        # height_to_waist = height_to_body + self.body_height
        # height_to_hip = height_to_waist + self.hip_height
        # height_to_knee = height_to_hip + self.leg_height1
        # height_to_ankle = height_to_knee + self.leg_height2
        #
        # left_shoulder_xyz = np.array([self.shoulder_width / 2, height_to_body, 0])
        # right_shoulder_xyz = (-self.shoulder_width / 2, height_to_body, 0)
        # left_waist_xyz = (self.waist_width / 2, height_to_waist, 0)
        # right_waist_xyz = (-self.waist_width / 2, height_to_waist, 0)
        # left_hip_xyz = (self.hip_width / 2, height_to_hip, 0)
        # right_hip_xyz = (-self.hip_width / 2, height_to_hip, 0)
        # hip_center_xyz = (0, height_to_hip, 0)
        # neck_top_xyz = (0, self.head_height, 0)
        # neck_bottom_xyz = (0, self.head_height + self.neck_height, 0)
        # head_center_xyz = (0, self.head_height/2, 0)
        #
        # left_elbow_xyz = (left_shoulder_xyz[0] + self.arm_height1 * sin(left_arm_angle1),
        #                  left_shoulder_xyz[1] + self.arm_height1 * cos(left_arm_angle1))
        # temp = left_arm_angle1 + PI - left_arm_angle2
        # left_wrist_xyz = (left_elbow_xyz[0] + self.arm_height2 * sin(temp),
        #                  left_elbow_xyz[1] + self.arm_height2 * cos(temp))
        # temp = left_arm_angle1 + 2 * PI - left_arm_angle2 - left_arm_angle3
        # left_fingertip_xyz = (left_wrist_xyz[0] + self.arm_height3 * sin(temp),
        #                      left_wrist_xyz[1] + self.arm_height3 * cos(temp))
        #
        # right_elbow_xyz = (right_shoulder_xyz[0] - self.arm_height1 * sin(right_arm_angle1),
        #                   right_shoulder_xyz[1] + self.arm_height1 * cos(right_arm_angle1))
        # temp = right_arm_angle1 + PI - right_arm_angle2
        # right_wrist_xyz = (right_elbow_xyz[0] - self.arm_height2 * sin(temp),
        #                   right_elbow_xyz[1] + self.arm_height2 * cos(temp))
        # temp = right_arm_angle1 + 2 * PI - right_arm_angle2 - right_arm_angle3
        # right_fingertip_xyz = (right_wrist_xyz[0] - self.arm_height3 * sin(temp),
        #                       right_wrist_xyz[1] + self.arm_height3 * cos(temp))

    def draw_head(self, head_center_xyz, head_width, head_height, body_yaw_degree):
        stroke_width = self.stroke_width
        return gz.ellipse(head_width, head_height,
                          stroke_width=stroke_width,
                          xy=tuple(head_center_xyz[:2])).rotate(body_yaw_degree*THETA_TO_RADIAN, center=head_center_xyz)

    def draw_neck(self, neck_top_xyz, neck_bottom_xyz, neck_width=0):
        stroke_width = self.stroke_width
        return gz.polyline(self._2D([neck_top_xyz, neck_bottom_xyz]), stroke_width=stroke_width)

    def draw_body(self, left_shoulder_xyz, right_shoulder_xyz, left_waist_xyz, right_waist_xyz):
        return gz.polyline(self._2D([right_shoulder_xyz, left_shoulder_xyz,
                            left_waist_xyz, right_waist_xyz, right_shoulder_xyz])
                           , stroke_width=self.stroke_width)
    @classmethod
    def _2D(cls, posns):
        return [tuple(p[:2]) for p in posns]

    def draw_hip(self, right_waist_xy, left_waist_xy,
                 left_hip_xy, right_hip_xy, ):
        return gz.polyline([right_waist_xy, left_waist_xy,
                            left_hip_xy, right_hip_xy, right_waist_xy],
                           stroke_width=self.stroke_width)

    def draw(self, left_arm_theta1=5, left_arm_theta2=170, left_arm_theta3=170, left_arm_phi=0,
                     right_arm_theta1=5, right_arm_theta2=170, right_arm_theta3=170, righ_arm_phi=0,
                     left_leg_theta1=5, left_leg_theta2=170, left_leg_theta3=90,
                     right_leg_theta1=5, right_leg_theta2=170, right_leg_theta3=90,
                     body_yaw_degree=0, body_pitch_degree=0):
        self._compute_xyz(left_arm_theta1, left_arm_theta2, left_arm_theta3, left_arm_phi,
                     right_arm_theta1, right_arm_theta2, right_arm_theta3, righ_arm_phi,
                     left_leg_theta1, left_leg_theta2, left_leg_theta3,
                     right_leg_theta1, right_leg_theta2, right_leg_theta3,
                     body_yaw_degree, body_pitch_degree)
        head = self.draw_head(self._head_center_xyz, self.head_width, self.head_height, body_yaw_degree)
        neck = self.draw_neck(self._neck_top_xyz, self._neck_bottom_xyz)
        body = self.draw_body(self._left_shoulder_xyz, self._right_shoulder_xyz,
                              self._left_waist_xyz, self._right_waist_xyz)
        group = gz.Group([head, neck, body])
        return group


    @staticmethod
    def _yaw_matrix(degrees):
        #  http://planning.cs.uiuc.edu/node102.html
        radians = degrees * THETA_TO_RADIAN # -1 for anticlockwise to clockwise
        cos_angle = cos(radians)
        sin_angle = sin(radians)
        return np.array([[cos_angle, -sin_angle, 0],
                         [sin_angle, cos_angle, 0],
                         [0, 0 , 1]])

    @staticmethod
    def _pitch_matrix(degrees):
        radians = degrees * (PI / 180.)  # -1 for anticlockwise to clockwise
        cos_angle = cos(radians)
        sin_angle = sin(radians)
        return np.array([[cos_angle, 0, -sin_angle],
                        [0, 1, 0],
                        [-sin_angle, 0, cos_angle]])


if __name__ == '__main__':
    fig = StickFigure()
    group = fig.draw(body_yaw_degree=0, body_pitch_degree=90)
    surface = gz.Surface(width=300, height=300, bg_color=(1, 1, 1))
    group = group.translate((150, 100))
    group.draw(surface)
    surface.write_to_png(r'/Users/gman/Documents/code/test.gif')