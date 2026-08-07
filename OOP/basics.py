class Robot:
    
    def __init__(self, name, battery_level):
        self.name = name
        self.battery_level = battery_level
    
    def status(self):
        print(f"Name: {self.name}, Battery level: {self.battery_level}")

    def charge(self, amount):
        self.battery_level += amount

robot = Robot("Rover", 80)
robot.status()

robot2 = Robot("Buzzy", 60)
robot2.status()
robot2.charge(20)
robot2.status()