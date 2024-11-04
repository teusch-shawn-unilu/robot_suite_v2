from tello_control_station.control_station import ControlStation
import rclpy


def main(args=None):
    rclpy.init(args=args)
    node = ControlStation()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
