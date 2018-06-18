from geometry_msgs.msg import Twist

class Robot:
    def __init__(self, width = 1, height = 1, ID = 0, name="ground"):
        self.width = width
        self.height = height
        self.radius = int(max(width, height) / 2)
        self.ID = ID
        self.x = 0
        self.y = 0
        self.quaternion = [0,0,0,0]
        self.eularAngles = [0,0,0]
        self.speed = 0.7
        self.twist = Twist()
        self.currentState = ['calculate', 'turn', 'forward']
        self.currentStateCounter = 0
        self.clockWise = False
        self.goalTheta = 0
        self.originalTheta = 0
        self.forwardDistance = 0
        self.nameSpace = name


    def getPosition():
        return (self.x, self.y)

    def getOrientation():
        return (self.quaternion)