import vrep

# response values
ERROR = -1
OK = 1

class simulator:
    def __init__(self, ip, port):
        self.id = -1
        self.ip = ip
        self.port = port

    def connect(self):
        self.id = vrep.simxStart(self.ip, self.port, True, True, 2000, 5)
        vrep.simxSynchronous(self.id, True)

        if self.id == ERROR:
            raise Exception('Unable to connect to V-REP Server!')

        return self.id

    def disconnect(self):
        if self.id is not ERROR:
            vrep.simxFinish(self.id)

    def pause(self):
        if self.id is not ERROR:
            vrep.simxPauseCommunication(self.id, 0)

    def resume(self):
        if self.id is not ERROR:
            vrep.simxPauseCommunication(self.id, 1)

    def update(self):
        if self.id is not ERROR:
            vrep.simxSynchronousTrigger(self.id)

    def get_handle(self, name):
        status, handle = vrep.simxGetObjectHandle(self.id, name, \
            vrep.simx_opmode_oneshot_wait)

        if status is ERROR:
            raise Exception('Unable to receive handle!')

        return handle

    def init_prox_sensor(self, handle):
        status, state, coord, _, _ = vrep.simxReadProximitySensor(self.id, \
                                        handle, vrep.simx_opmode_streaming)
        if status is ERROR:
            raise Exception('Unable to init sensor!')

        return state, coord

    def read_prox_sensor(self, handle):
        status, state, coord, _, _ = vrep.simxReadProximitySensor(self.id, \
                                        handle, vrep.simx_opmode_buffer)

        if status is ERROR:
            raise Exception('Unable to receive handle!')

        return state, coord

    def get_position(self, handle):
        # return absolute position
        status, pos = vrep.simxGetObjectPosition(self.id, handle, -1, \
                                        vrep.simx_opmode_streaming)

        if status is ERROR:
            raise Exception('Unable to receive handle!')

        return pos

    def get_orientation(self, handle):
        # return absolute position
        status, pos = vrep.simxGetObjectOrientation(self.id, handle, -1, \
                                        vrep.simx_opmode_streaming)

        if status is ERROR:
            raise Exception('Unable to receive handle!')

        return pos

    def get_velocity(self, handle):
        # return linear and angular velocity
        status, linear, angular = vrep.simxGetObjectVelocity(self.id, handle, \
                                        vrep.simx_opmode_streaming)

        if status is ERROR:
            raise Exception('Unable to receive handle!')

        return linear, angular

    def get_joint_position(self, handle):
        status, pos = vrep.simxGetJointPosition(self.id, handle, \
                                        vrep.simx_opmode_streaming)

        if status is ERROR:
            raise Exception('Unable to receive handle!')

        return pos

    def set_joint_target_velocity(self, handle, v):
        if self.id is not ERROR:
            vrep.simxSetJointTargetVelocity(self.id, handle, v, \
                                                vrep.simx_opmode_streaming)
