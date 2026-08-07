class Battery:

    def __init__(self, level):
        self.level = level

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

if __name__ == "__main__":
    battery = Battery(80)
    battery.status()
    battery.drain(100)
    battery.status()

    robot = Robot("Rover", battery)
    robot.status()
    robot.move()
    robot.status()