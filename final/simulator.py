import vrep

# response values
ERROR = -1
OK = 1

class Simulator(object):
    def __init__(self, ip, port):
        self.id = -1
        self.ip = ip
        self.port = port

    def connect(self):
        self.id = vrep.simxStart(self.ip, self.port, True, False, 2000, 5)
        vrep.simxSynchronous(self.id, True)

        if self.id == ERROR:
            raise Exception('Unable to connect to V-REP Server!')

        return self.id

    def disconnect(self):
        if self.id is not ERROR:
            vrep.simxFinish(self.id)

    def pause(self):
        if self.id is not ERROR:
            vrep.simxPauseCommunication(self.id, True)

    def resume(self):
        if self.id is not ERROR:
            vrep.simxPauseCommunication(self.id, False)

    def update(self):
        if self.id is not ERROR:
            vrep.simxSynchronousTrigger(self.id)

    def execute_script(self, object_name, function_name):
        if self.id is not ERROR:
            _, _, _, _, _ = vrep.simxCallScriptFunction(
                self.id, object_name, vrep.sim_scripttype_customizationscript,
                function_name, [], [], [], bytearray(), vrep.simx_opmode_blocking)

    def get_handle(self, name):
        status, handle = vrep.simxGetObjectHandle(
            self.id, name, vrep.simx_opmode_oneshot_wait)

        if status is ERROR:
            raise Exception('Unable to receive handle!')

        return handle

    def read_prox_sensor(self, handle, first_call=False):
        opmode = vrep.simx_opmode_streaming if first_call else vrep.simx_opmode_buffer
        status, state, coord, _, _ = vrep.simxReadProximitySensor(self.id, handle, opmode)

        if status is ERROR:
            raise Exception('Unable to receive handle!')

        return state, coord

    def get_position(self, handle, first_call=False, relative_to_parent=False):
        opmode = vrep.simx_opmode_streaming if first_call else vrep.simx_opmode_buffer
        relative = vrep.sim_handle_parent if relative_to_parent else -1
        # return absolute position
        status, pos = vrep.simxGetObjectPosition(self.id, handle, relative, opmode)

        if status is ERROR:
            raise Exception('Unable to receive handle!')

        return pos

    def get_orientation(self, handle, first_call=False, relative_to_parent=False):
        opmode = vrep.simx_opmode_streaming if first_call else vrep.simx_opmode_buffer
        relative = vrep.sim_handle_parent if relative_to_parent else -1
        # return absolute position
        status, pos = vrep.simxGetObjectOrientation(self.id, handle, relative, opmode)

        if status is ERROR:
            raise Exception('Unable to receive handle!')

        return pos

    def get_velocity(self, handle, first_call=False):
        opmode = vrep.simx_opmode_streaming if first_call else vrep.simx_opmode_buffer
        # return linear and angular velocity
        status, linear, angular = vrep.simxGetObjectVelocity(self.id, handle, opmode)

        if status is ERROR:
            raise Exception('Unable to receive handle!')

        return linear, angular

    def get_joint_position(self, handle, first_call=False):
        opmode = vrep.simx_opmode_streaming if first_call else vrep.simx_opmode_buffer
        status, pos = vrep.simxGetJointPosition(self.id, handle, opmode)

        if status is ERROR:
            raise Exception('Unable to receive handle!')

        return pos

    def set_position(self, handle, pos):
        if self.id is not ERROR:
            vrep.simxSetObjectPosition(
                self.id, handle, -1, pos, vrep.simx_opmode_oneshot)

    def set_orientation(self, handle, ori):
        if self.id is not ERROR:
            vrep.simxSetObjectOrientation(
                self.id, handle, -1, ori, vrep.simx_opmode_oneshot)

    def set_joint_position(self, handle, pos):
        if self.id is not ERROR:
            vrep.simxSetJointPosition(
                self.id, handle, pos, vrep.simx_opmode_oneshot)

    def set_joint_target_velocity(self, handle, vel):
        if self.id is not ERROR:
            vrep.simxSetJointTargetVelocity(
                self.id, handle, vel, vrep.simx_opmode_streaming)
