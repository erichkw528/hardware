import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import pygame
import numpy as np
from cv_bridge import CvBridge
from typing import Optional
import cv2
from pygame import *
from roar_msgs.msg import EgoVehicleControl
from pydantic import BaseModel, validator
import time

MAX_SPEED = 1
MIN_ANGLE = -1
MAX_ANGLE = 1


class State(BaseModel):
    throttle: float = 0.0  # speed < 0 = brake
    steering_angle: float = 0.0
    brake: float = 0.0
    reverse: bool = False

    @validator("speed")
    def check_throttle(cls, v):
        assert v <= MAX_SPEED, f"throttle value {v} incorrect."

    @validator("steering_angle")
    def check_steering(cls, v):
        assert MIN_ANGLE <= v <= MAX_ANGLE, f"steering value {v} incorrect."


class ManualControllerNode(Node):
    def __init__(self):
        super().__init__("manual_controller_node")
        self.declare_parameter("loop_rate", 0.05)

        self.declare_parameter("rgb_topic", "/zed2i/center_camera/rgb/image_rect_color")
        self.declare_parameter("speed_increment", 0.1)
        self.declare_parameter("angle_increment", 0.1)

        self.timer = self.create_timer(
            self.get_parameter("loop_rate").get_parameter_value().double_value,
            self.timer_callback,
        )
        self.rgb_sub_ = self.create_subscription(
            Image,
            self.get_parameter("rgb_topic").get_parameter_value().string_value,
            self.on_image_recv,
            10,
        )
        self.publisher = self.create_publisher(EgoVehicleControl, "/manual_control", 10)
        self.bridge = CvBridge()
        self.image: Optional[np.ndarray] = None

        pygame.init()
        self.display = None
        self.surface = None
        pygame.display.set_caption(self.get_name())

        self.state = State()

    def on_image_recv(self, image: Image):
        self.image = cv2.rotate(
            np.array(self.bridge.imgmsg_to_cv2(image, "bgr8")),
            cv2.ROTATE_90_COUNTERCLOCKWISE,
        )

    def timer_callback(self):

        if self.image is not None:
            if self.surface is None or self.display == None:
                # first render
                self.display = pygame.display.set_mode(
                    (self.image.shape[0], self.image.shape[1]),
                    pygame.HWSURFACE | pygame.DOUBLEBUF,
                )
                self.surface = pygame.surfarray.make_surface(self.image)
            frame: np.ndarray = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
            pygame_image = pygame.surfarray.make_surface(frame)
            self.display.blit(pygame_image, (0, 0))
            pygame.display.flip()

        self.parse_event()
        msg: EgoVehicleControl = self.p_state_to_control(self.state)
        self.publisher.publish(msg)

    def p_state_to_control(self, state: State) -> EgoVehicleControl:
        msg: EgoVehicleControl = EgoVehicleControl()
        msg.throttle = float(state.throttle)
        msg.steer = float(state.steering_angle)
        msg.brake = float(state.brake)
        msg.reverse = False

        return msg

    def parse_event(self):
        speed_inc = (
            self.get_parameter("speed_increment").get_parameter_value().double_value
        )
        angle_inc = (
            self.get_parameter("angle_increment").get_parameter_value().double_value
        )
        for event in pygame.event.get():
            if event.type == KEYDOWN:
                if event.key == K_r:
                    self.state.reverse = not self.state.reverse

                if event.key == K_w or event.key == K_UP:
                    self.state.speed = min(self.state.speed + speed_inc, MAX_SPEED)
                if event.key == K_s or event.key == K_DOWN:
                    self.state.speed = max(0, self.state.speed - speed_inc)

                if event.key == K_SPACE:
                    self.state.speed = -1

                if event.key == K_d or event.key == K_RIGHT:
                    self.state.steering_angle = min(
                        self.state.steering_angle + angle_inc, MAX_ANGLE
                    )
                if event.key == K_a or event.key == K_LEFT:
                    self.state.steering_angle = max(
                        MIN_ANGLE, self.state.steering_angle - angle_inc
                    )

                # TODO: parse key for brake and reverse

    def destroy_node(self) -> bool:
        msg: EgoVehicleControl = EgoVehicleControl()
        msg.throttle = float(0)
        msg.steer = float(0)
        num_try = 3
        for i in range(num_try):
            self.publisher.publish(msg)
            self.get_logger().info(f"Safely shutting down: {i+1}/{num_try}")
            time.sleep(0.5)
        pygame.quit()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ManualControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)
    finally:
        node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
