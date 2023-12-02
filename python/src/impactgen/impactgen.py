# pylint: disable = missing-module-docstring
# pylint: disable = missing-class-docstring
# pylint: disable = missing-function-docstring

import logging as log
from pathlib import Path
from scipy.spatial.transform import Rotation

from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.sensors import IMU, Electrics, Damage, State


class ImpactGenerator:

    gridmap = {
        'level': 'gridmap_v2',

        'a_spawn': (-268, 56, 100.5),
        'b_spawn': (-275, 56, 100.5),

        'pole_pos': (-285, 56, 100.5),

        'a_spawn_wall': (-269.604, 57.547, 100.5),
        'a_rot_wall':   (0.557, 0.118, -180.000),

        'linear_pos_a': (-269.604, 57.547, 100.5),
        'linear_pos_b': (-269.604, 75, 100.5),
        'linear_rot_b': (0, 0, -180.000),

        't_pos_a': (-269.604, 57.547, 100.5),
        't_pos_b': (-269.604, 75, 100.5),
        't_rot_b': (0, 0, 90),

        'ref_pos': (-268, 56, 100.5),
    }

    def __init__(self, bng_home, output, single=False):
        self.bng_home = bng_home
        self.output = Path(output)
        self.single = single

        self.bng = BeamNGpy('localhost', 64256, home=bng_home)

        self.scenario = None

        scenario_props = ImpactGenerator.gridmap

        self.vehicle_a = Vehicle('vehicle_a', model='etk800')
        self.vehicle_b = Vehicle('vehicle_b', model='etk800')

        self.setup_sensors()

        self.scenario = Scenario(scenario_props['level'], 'impactgen')
        self.scenario.add_vehicle(
            self.vehicle_a, pos=scenario_props['a_spawn'])
        self.scenario.add_vehicle(
            self.vehicle_b, pos=scenario_props['b_spawn'])

    def init_settings(self):
        self.bng.settings.set_particles_enabled(False)

    def setup(self):
        self.scenario.make(self.bng)
        log.debug('Loading scenario...')
        self.bng.scenario.load(self.scenario)
        log.debug('Setting steps per second...')
        self.bng.settings.set_deterministic(60)
        log.debug('Starting scenario...')
        self.bng.scenario.start()

        self.init_settings()

        log.debug('Setting annotation properties.')

    def setup_sensors(self):
        self.vehicle_a_imu = IMU((0, 0, 0), debug=True)
        self.vehicle_b_imu = IMU((0, 0, 0), debug=True)

        self.vehicle_a_damage = Damage()
        self.vehicle_b_damage = Damage()

        self.vehicle_a_electrics = Electrics()
        self.vehicle_b_electrics = Electrics()

        self.vehicle_a_state = State()
        self.vehicle_b_state = State()

        self.vehicle_a.sensors.attach(
            "vehicle_a_sensor", self.vehicle_a_imu)
        self.vehicle_b.sensors.attach(
            "vehicle_b_sensor", self.vehicle_b_imu)

        self.vehicle_a.sensors.attach(
            "vehicle_a_electrics", self.vehicle_a_electrics)
        self.vehicle_b.sensors.attach(
            "vehicle_b_electrics", self.vehicle_b_electrics)

        self.vehicle_a.sensors.attach(
            "vehicle_a_damage", self.vehicle_a_damage)
        self.vehicle_b.sensors.attach(
            "vehicle_b_damage", self.vehicle_b_damage)

        self.vehicle_a.sensors.attach(
            "vehicle_a_state", self.vehicle_a_state)
        self.vehicle_b.sensors.attach(
            "vehicle_b_state", self.vehicle_b_state)

    def teleport(self, vehicle, pos, rot_euler, reset=True):
        rot = Rotation.from_euler("xyz", rot_euler, True)
        rot_quat = rot.as_quat()
        rot_quat = (rot_quat[0], rot_quat[1], rot_quat[2], rot_quat[3])
        vehicle.teleport(pos, rot_quat, reset)

    def run_linear_crash(self):
        log.info('Running linear crash setting.')
        pos = (-277.275, 87.664, 100.303)
        targetpos = (-267.275, 200.059, 100.303)

        self.teleport(self.vehicle_a, pos, (0, 0, 180))
        self.teleport(self.vehicle_b, targetpos, (0, 0, 180))

        self.bng.control.step(60)

        self.vehicle_a.ai.set_target(self.vehicle_b.vid)
        self.vehicle_a.ai.set_speed(15)

    def stop_car(self, vehicle):
        vehicle.control(steering=0, throttle=0,
                        brake=1, parkingbrake=1, gear=0)
        self.bng.control.step(5*60)

    def run(self):
        log.info('Starting up BeamNG instance.')
        self.bng.open()
        try:
            log.info('Setting up BeamNG instance.')
            self.setup()

            # pylint: disable-next = unused-variable
            for i in range(3):
                self.run_linear_crash()

                while True:
                    self.vehicle_b.sensors.poll()
                    self.vehicle_a.sensors.poll()
                    if self.vehicle_a_damage['damage'] > 10:
                        self.vehicle_a.ai.set_mode("disabled")
                        self.stop_car(self.vehicle_a)
                        break
        finally:
            log.info('Closing BeamNG instance.')
            self.bng.close()
