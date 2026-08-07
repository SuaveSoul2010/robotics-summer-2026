import rclpy
from rclpy.node import Node
from calculator.srv import Calc


class CalculatorServer(Node):

    def __init__(self):
        super().__init__('calculator_server')
        self.create_service(Calc, 'calc_service', self.callback)
        self.get_logger().info("Service ready")

    def callback(self, request, response):

        a = request.a
        b = request.b
        op = request.operation

        if op == "add":
            response.result = a + b
        elif op == "subtract":
            response.result = a - b
        elif op == "multiply":
            response.result = a * b
        elif op == "divide":
            response.result = a / b if b != 0 else 0.0
        else:
            response.result = 0.0

        self.get_logger().info(f"{a} {op} {b} = {response.result}")
        return response


def main():
    rclpy.init()
    node = CalculatorServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()