import numpy as np
from controller import Robot # type: ignore
from time import sleep

from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink

from tensorflow import keras

pi = np.pi


class Ur5Controller(Robot):
    def __init__(self, ur5):
        super(Ur5Controller, self).__init__()
        self.timeStep = 20
        self.ur5 = ur5
        self.chain = Chain.from_urdf_file("ur5e_with_gripper.urdf")

    def initUr5(self):
        # Init motor class for every joint
        self.elbow_joint = self.getDevice('elbow_joint')
        self.shoulder_lift_joint = self.getDevice('shoulder_lift_joint')
        self.shoulder_pan_joint = self.getDevice('shoulder_pan_joint')
        self.wrist_1_joint = self.getDevice('wrist_1_joint')
        self.wrist_2_joint = self.getDevice('wrist_2_joint')
        self.wrist_3_joint = self.getDevice('wrist_3_joint')

        # Global Positioning Sensor
        self.gps = self.getDevice('gps')

        # Init PositionSensor class for every joint
        self.elbow_joint_sensor = self.getDevice('elbow_joint_sensor')
        self.shoulder_lift_joint_sensor = self.getDevice(
            'shoulder_lift_joint_sensor')
        self.shoulder_pan_joint_sensor = self.getDevice(
            'shoulder_pan_joint_sensor')
        self.wrist_1_joint_sensor = self.getDevice('wrist_1_joint_sensor')
        self.wrist_2_joint_sensor = self.getDevice('wrist_2_joint_sensor')
        self.wrist_3_joint_sensor = self.getDevice('wrist_3_joint_sensor')

        # Activate sensors
        self.elbow_joint_sensor.enable(10)
        self.shoulder_lift_joint_sensor.enable(10)
        self.shoulder_pan_joint_sensor.enable(10)
        self.wrist_1_joint_sensor.enable(10)
        self.wrist_2_joint_sensor.enable(10)
        self.wrist_3_joint_sensor.enable(10)

        # Initial postion
        self.step(10)
        self.elbow_joint.setPosition(0)
        self.shoulder_lift_joint.setPosition(0)
        self.shoulder_pan_joint.setPosition(0)
        self.wrist_1_joint.setPosition(0)
        self.wrist_2_joint.setPosition(0)
        self.wrist_3_joint.setPosition(0)

    def dhParams(self):
        dh = [
                {
                    "alpha": 0,
                    "a": 0,
                    "distance": self.ur5.joint1.distance,
                    "theta": self.ur5.joint1.theta,
                },
                {
                    "alpha": pi / 2,
                    "a": 0,
                    "distance": 0,
                    "theta": self.ur5.joint2.theta,
                },
                {
                    "alpha": 0,
                    "a": self.ur5.joint2.a,
                    "distance": 0,
                    "theta": self.ur5.joint3.theta,
                },
                {
                    "alpha": 0,
                    "a": self.ur5.joint3.a,
                    "distance": self.ur5.joint4.distance,
                    "theta": self.ur5.joint4.theta,
                },
                {
                    "alpha": pi / 2,
                    "a": 0,
                    "distance": self.ur5.joint5.distance,
                    "theta": self.ur5.joint5.theta,
                },
                {
                    "alpha": -pi / 2,
                    "a": 0,
                    "distance": self.ur5.joint6.distance,
                    "theta": self.ur5.joint6.theta,
                },
            ]
        return dh

    def InverseKinematics(self, x, y, z):
        target = np.array([x, y, z])
        target_input = target[np.newaxis, :] # reshape to proper dimensions for the model
        angles_0 = self.chain.inverse_kinematics(target) # ikpy solution
        ANN = keras.models.load_model('../../ann_model/keras_model.keras')
        prediction = ANN.predict(target_input)
        angles = prediction[0]
        print("Predicted Angles: ")
        print(prediction[0])
        print("IKPY Angles: ")
        print(angles_0)
        return angles[1:] 
    
    def LSPB_trajectory(self, angles, sim_time):
        LSPB_results = []
        q0 = 0
        times = np.arange(0, sim_time, 0.01)
        for joint in angles:
            qf = joint
            V = 1.5*(qf - q0)/sim_time
            tb = (q0 - qf + V*sim_time)/V
            alpha = V/tb
            positions= []
            velocities = [] 
            accelarations = []
            for t in times:
                if t<=tb:
                    jointPosition = q0 + (alpha*(t**2))/2
                    jointVelocity = alpha*t
                    jointAcceleration = alpha
                    positions.append(jointPosition)
                    velocities.append(jointVelocity)
                    accelarations.append(jointAcceleration)
                elif tb < t and sim_time - tb >= t:
                    jointPosition = (qf+q0 - V*sim_time)/2 + V*t
                    jointVelocity = V
                    jointAcceleration = alpha
                    positions.append(jointPosition)
                    velocities.append(jointVelocity)
                    accelarations.append(jointAcceleration)

                else:
                    jointPosition = qf - alpha*(sim_time**2)/2 + alpha*sim_time*t - (alpha*(t**2))/2
                    jointVelocity = alpha*sim_time - alpha*t
                    jointAcceleration = -alpha
                    positions.append(jointPosition)
                    velocities.append(jointVelocity)
                    accelarations.append(jointAcceleration)
            LSPB_results.append(
                {
                    'positions': positions,
                    'velocities': velocities,
                    'acceletations': accelarations,
                    'time': np.arange(0, sim_time, 0.01),
                    'q0': q0,
                    }
                )
        return LSPB_results

    def run(self):
        i = 0
        while self.step(self.timeStep) != -1:
            i = i + 0.1
            self.shoulder_pan_joint.setPosition(i)
    
    

    def moveToTarget(self, point):
        x = point[0]
        y = point[1]
        z = point[2]
        target_angles = self.InverseKinematics(x, y, z)
        LSPB_results = self.LSPB_trajectory(target_angles, 1)
        i = 0
        for result in LSPB_results:
            for angle in result['positions']:
                if i == 0:
                    self.step(1)
                    self.shoulder_pan_joint.setPosition(angle)
                if i == 1:
                    self.step(1)
                    self.shoulder_lift_joint.setPosition(angle)
                if i == 2:
                    self.step(1)
                    self.elbow_joint.setPosition(angle)
                if i == 3:
                    self.step(1)
                    self.wrist_1_joint.setPosition(angle)
                if i == 4:
                    self.step(1)
                    self.wrist_2_joint.setPosition(angle)
                if i == 5:
                    self.step(1)
                    self.wrist_3_joint.setPosition(angle)
            i = i + 1
                
               
       

        