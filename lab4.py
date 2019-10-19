import json
import pygame
import time
import rospy
import numpy as np

from enum import Enum
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray, Empty, Int16, String


SCREEN_WIDTH = 420
SCREEN_HEIGHT = 600
SPARKI_IMG_WIDTH = 50
SPARKI_IMG_HEIGHT = 64
ZERO_OFFSET_X = 40
ZERO_OFFSET_Y = 470
SCALE = 3

IR_THRESHOLD = 500
CYCLE_TIME = 0.05

WORLD_SIZE_X = 60
WORLD_SIZE_Y = 42

PING_COOLDOWN = 1


class Line(Enum):
    NONE = 0
    LEFT = 1
    RIGHT = 2
    CENTER = 3
    FULL = 4


class Sparki(object):
    def __init__(self):
        self.pose = None
        self.ir_distances = [0] * 5
        self._motor_speeds = [0] * 2
        self._servo_angle = 0
        self.us_distance = 0
        self._last_ping = time.time()

        self.pub_motor = rospy.Publisher('/sparki/motor_command', Float32MultiArray, queue_size=10)
        self.pub_odometry = rospy.Publisher('/sparki/set_odometry', Pose2D, queue_size=10)
        self.pub_ping = rospy.Publisher('/sparki/ping_command', Empty, queue_size=10)
        self.pub_servo = rospy.Publisher('sparki/set_servo', Int16, queue_size=10)
        self.sub_odometry = rospy.Subscriber('/sparki/odometry', Pose2D, self._update_odometry)
        self.sub_state = rospy.Subscriber('/sparki/state', String, self._update_state)

    def request_ultrasonic(self):
        if (time.time() - self._last_ping) < PING_COOLDOWN:
            return

        self.pub_ping.publish()
        self._last_ping = time.time()

    def reset_odometry(self):
        if self.pose == Pose2D():
            return

        self.pose = Pose2D()
        self.pub_odometry.publish(self.pose)
        rospy.loginfo('Loop Closure Triggered')

    @property
    def line_status(self):
        left = self.ir_distances[1] < IR_THRESHOLD
        center = self.ir_distances[2] < IR_THRESHOLD
        right = self.ir_distances[3] < IR_THRESHOLD

        if center:
            if left and right:
                return Line.FULL
            return Line.CENTER

        if left:
            return Line.LEFT

        if right:
            return Line.RIGHT

        return Line.NONE

    @property
    def motor_speeds(self):
        return self._motor_speeds

    @motor_speeds.setter
    def motor_speeds(self, value):
        if value == self.motor_speeds:
            return

        arr = Float32MultiArray()
        arr.data = value
        self.pub_motor.publish(arr)
        self._motor_speeds = value

    @property
    def servo_angle(self):
        return self._servo_angle

    @servo_angle.setter
    def servo_angle(self, angle):
        assert -80 < angle < 80

        self._servo_angle = angle
        self.pub_servo.publish(angle)

    def _update_odometry(self, pose):
        self.pose = pose

    def _update_state(self, data):
        state = json.loads(data.data)

        self.ir_distances = state.get('light_sensors', self.ir_distances)
        self._servo_angle = state.get('servo', self._servo_angle)
        self.us_distance = state.get('ping', self.us_distance)


class Map(object):
    def __init__(self):
        pygame.init()
        pygame.display.set_caption("Sparki's Map")

        self.world = [[False] * WORLD_SIZE_X for _ in range(WORLD_SIZE_Y)]

        self.sparki_img = pygame.transform.scale(pygame.image.load('sparki.png'), (SPARKI_IMG_WIDTH, SPARKI_IMG_HEIGHT))
        self.map_img = pygame.transform.scale(pygame.image.load('map.png'), (SCREEN_WIDTH, SCREEN_HEIGHT))

        self.screen = pygame.display.set_mode([SCREEN_WIDTH * SCALE, SCREEN_HEIGHT * SCALE])
        self.fake_screen = pygame.surface.Surface((SCREEN_WIDTH, SCREEN_HEIGHT))

    def draw(self, pose):
        self.fake_screen.blit(self.map_img, (0, 0))

        self.fake_screen.blit(
            pygame.transform.rotate(self.sparki_img, np.degrees(pose.theta)),
            (
                (-pose.y * 1000) + ZERO_OFFSET_X - (SPARKI_IMG_WIDTH // 2),
                (-pose.x * 1000) + ZERO_OFFSET_Y - (SPARKI_IMG_HEIGHT // 2),
            )
        )

        for row in range(WORLD_SIZE_Y):
            for col in range(WORLD_SIZE_X):
                if self.world[row][col]:
                    pygame.draw.rect(self.fake_screen, (255, 0, 0), (row * 10, col * 10, 10, 10))

        self.screen.blit(pygame.transform.scale(self.fake_screen, (SCREEN_WIDTH * SCALE, SCREEN_HEIGHT * SCALE)), (0, 0))
        pygame.display.update()

    def add_point(self, pose, servo_angle, dist):
        print('X:', pose.x)
        print('Y:', pose.y)
        print('T:', pose.theta)
        print('A:', servo_angle)
        print('D:', dist)

        rotation_matrix = np.empty(shape=(3, 3))
        rotation_matrix[0][0] = np.cos(pose.theta)
        rotation_matrix[0][1] = -(np.sin(pose.theta))
        rotation_matrix[0][2] = pose.x
        rotation_matrix[1][0] = np.sin(pose.theta)
        rotation_matrix[1][1] = np.cos(pose.theta)
        rotation_matrix[1][2] = pose.y
        rotation_matrix[2][0] = 0
        rotation_matrix[2][1] = 0
        rotation_matrix[2][2] = 1

        obj_x = dist * np.cos(servo_angle)
        obj_y = dist * np.sin(servo_angle)
        robot_coords = np.empty(shape=(3, 1))
        robot_coords[0][0] = obj_x
        robot_coords[1][0] = obj_y
        robot_coords[2][0] = 1

        homo_matrix = np.matmul(rotation_matrix, robot_coords)
        # print('Added:', homo_matrix)
        self.world[int(homo_matrix[0, 0])][int(homo_matrix[1, 0])] = True


class Lab4(object):
    def __init__(self):
        rospy.init_node('Lab4')

        self.sparki = Sparki()
        self.map = Map()

        self.sparki.reset_odometry()
        self.sparki.motor_speeds = 0, 0
        self.sparki.servo_angle = -40

    def _loop(self):
        self.sparki.request_ultrasonic()

        if self.sparki.line_status == Line.FULL:
            self.sparki.reset_odometry()
            self.sparki.motor_speeds = 1, 1
        elif self.sparki.line_status == Line.LEFT:
            self.sparki.motor_speeds = -1, 1
        elif self.sparki.line_status == Line.RIGHT:
            self.sparki.motor_speeds = 1, -1
        else:
            self.sparki.motor_speeds = 1, 1

        if 0 < self.sparki.us_distance < 30:
            self.map.add_point(self.sparki.pose, self.sparki.servo_angle, self.sparki.us_distance)
        self.map.draw(self.sparki.pose)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.sparki.motor_speeds = 0, 0
                exit(0)

    def start(self):
        while not rospy.is_shutdown():
            start = time.time()
            self._loop()
            end = time.time()

            diff = end - start
            if diff > CYCLE_TIME:
                diff = CYCLE_TIME

            rospy.sleep(CYCLE_TIME - diff)


if __name__ == '__main__':
    Lab4().start()
