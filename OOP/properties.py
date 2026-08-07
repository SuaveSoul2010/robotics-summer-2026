class Battery:
    def __init__(self, level):
        self.level = level
        
    """
    @property
    def level(self):
        return self._level

    @level.setter
    def level(self, value):
        if value < 0 or value > 100:
            print("Invalid level!")
        else:
            self._level = value
    """  

    def status(self):
        print(f"Level: {self.level}")

    def drain(self, amount):
        if self.level < amount:
            self.level = 0
            return False
        else:
            self.level -= amount
            return True

class Robot:
    
    def __init__(self, name, battery):
        self.name = name
        self.battery = battery

    def status(self):
        print(self.name)
        self.battery.status()
       
    def move(self):
        if self.battery.drain(10):
            print("Moving...")
        else:
            print("Not enough battery to move.")

    @property
    def name(self):
        return self._name
    
    @name.setter
    def name(self, value):
        if value == "":
            raise ValueError("Invalid, empty name")
        elif len(value) > 15:
            raise ValueError("Invalid, name too long")
        else:
            self._name = value

if __name__ == "main":
    robot = Robot("rfuihuiwuhfiwuhfrirwuhfiwufh", 10)
    robot.status()