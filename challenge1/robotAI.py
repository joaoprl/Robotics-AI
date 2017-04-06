VELOCITY = 2

class robotAI:
    def __init__(self, p3dx):
        self.p3dx = p3dx
        self.state = self.moveForward

    def tick(self):
        self.checkStuck()
        self.state()

    def checkStopped(self):
        return self.p3dx.last_position == self.p3dx.position

    ## check last and current possition if robot is trying to move
    def checkStuck(self):
        if self.state == self.moveForward or self.state == self.moveBackward:
            if self.checkStopped():
                #print ('Robot possibly stuck at [' + str(self.p3dx.position[0]) + ', ' + \
                #        str(self.p3dx.last_position[0]) + ', ' + str(self.p3dx.orientation[2]) + ']')
                return True
        return False

    ## state: stop robot
    def stop(self):
        if self.checkStopped():
            self.state = self.moveForward()
        else:
            self.p3dx.move(0, 0)

    ## state: move robot forward
    def moveForward(self):
        self.p3dx.move(VELOCITY, VELOCITY)

    ## state: move robot backward
    def moveBackward(self):
        self.p3dx.move(-VELOCITY, -VELOCITY)

    ## state: rotate robot left
    def rotateLeft(self):
        self.p3dx.move(-VELOCITY, VELOCITY)

    ## state: rotate robot right
    def rotateRight(self):
        self.p3dx.move(VELOCITY, -VELOCITY)
    