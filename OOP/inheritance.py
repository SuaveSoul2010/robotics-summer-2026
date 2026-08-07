from properties import Robot, Battery

class LightRobot(Robot):
    
    def __init__(self, name, battery, weight):
        super().__init__(name, battery)
        self.weight = weight

    def move(self):
        if self.battery.drain(5):
            print("Moving...")
        else:
            print("Not enough battery.")

class HeavyRobot(Robot):

    
    def __init__(self, name, battery, weight):
        super().__init__(name, battery)
        self.weight = weight
    

    def move(self):
        if self.battery.drain(20):
            print("Moving...")
        else:
            print("Not enough battery.")

    @property
    def weight(self):
        return self._weighta
    
    @weight.setter
    def weight(self, value):
        if value <= 20:
            raise ValueError("Wrong weight class")
        else:
            self._weight = value

gbattery = Battery(1000)
Gary = LightRobot("Gary", gbattery, 15)
Gary.status()
Gary.move()
Gary.status()

Gbattery = Battery(40)
Gerald = HeavyRobot("Gerald", Gbattery, 15)
Gerald.status()
Gerald.move()
Gerald.status()

robots = [Gary, Gerald]
for robot in robots:
    print(robot.weight)
