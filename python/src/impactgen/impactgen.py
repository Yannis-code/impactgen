# pylint: disable = missing-module-docstring
# pylint: disable = missing-class-docstring
# pylint: disable = missing-function-docstring

import os, time
import logging as log
# import win32pipe
# import win32file
# import win32api
from pathlib import Path
from scipy.spatial.transform import Rotation

from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.sensors import GForces, Electrics, Damage, State, Timer


class ImpactGenerator:

    gridmap = {
        'level': 'gridmap_v2',

        'vehicle': [
            {
                'name': 'vehicle_a',
                'model': 'etk800',
                'position': (-268, 56, 100.5)
            },
            {
                'name': 'vehicle_b',
                'model': 'etk800',
                'position': (-275, 56, 100.5)
            }
        ],
        'crashs': {
            'linear': {
                'vehicle_a': {
                    'position': (-277.275, 87.664, 100.303),
                    'orientation': (0, 0, 180)
                },
                'vehicle_b': {
                    'position': (-267.275, 200.059, 100.303),
                    'orientation': (0, 0, 0)
                }
            }
        },
    }

    wca = {
        'level': 'west_coast_usa',

        'vehicle': [
            {
                'name': 'vehicle_a',
                'model': 'etk800',
                'position': (-712.760, 105.048, 118.650)
            },
            {
                'name': 'vehicle_b',
                'model': 'etk800',
                'position': (-719.422, 105.044, 100.5)
            }
        ],
    }

    def __init__(self, bng_home, output, single=False):
        self.bng_home = bng_home
        self.output = Path(output)
        self.single = single
        # self.dataPipeA = win32pipe.CreateNamedPipe(
        #     r'\\.\pipe\impactgenA', win32pipe.PIPE_ACCESS_OUTBOUND, win32pipe.PIPE_TYPE_MESSAGE | win32pipe.PIPE_NOWAIT, 2, 65536, 65536, 300, None)

        # self.dataPipeB = win32pipe.CreateNamedPipe(
        #     r'\\.\pipe\impactgenA', win32pipe.PIPE_ACCESS_OUTBOUND, win32pipe.PIPE_TYPE_MESSAGE | win32pipe.PIPE_NOWAIT, 2, 65536, 65536, 300, None)

        self.bng = BeamNGpy('localhost', 64256, home=bng_home)

        self.scenario = None

        scenario_props = ImpactGenerator.gridmap

        self.vehicle_a = Vehicle(
            scenario_props['vehicle'][0]['name'], model=scenario_props['vehicle'][0]['model'])
        self.vehicle_b = Vehicle(
            scenario_props['vehicle'][1]['name'], model=scenario_props['vehicle'][1]['model'])

        self.setup_sensors()

        self.scenario = Scenario(scenario_props['level'], 'impactgen')
        self.scenario.add_vehicle(
            self.vehicle_a, pos=scenario_props['vehicle'][0]['position'])
        self.scenario.add_vehicle(
            self.vehicle_b, pos=scenario_props['vehicle'][1]['position'])
        self.prev_frame_damage = 0

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
        self.vehicle_a_gforce = GForces()
        self.vehicle_b_gforce = GForces()

        self.vehicle_a_damage = Damage()
        self.vehicle_b_damage = Damage()

        self.vehicle_a_electrics = Electrics()
        self.vehicle_b_electrics = Electrics()

        self.vehicle_a_state = State()
        self.vehicle_b_state = State()

        self.vehicle_a_timer = Timer()
        self.vehicle_b_timer = Timer()

        self.vehicle_a.sensors.attach(
            "vehicle_a_sensor", self.vehicle_a_gforce)
        self.vehicle_b.sensors.attach(
            "vehicle_b_sensor", self.vehicle_b_gforce)

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

        self.vehicle_a.sensors.attach(
            "vehicle_a_timer", self.vehicle_a_timer)
        self.vehicle_b.sensors.attach(
            "vehicle_b_timer", self.vehicle_b_timer)

    def teleport(self, vehicle, pos, rot_euler, reset=True):
        rot = Rotation.from_euler("xyz", rot_euler, True)
        rot_quat = rot.as_quat()
        rot_quat = (rot_quat[0], rot_quat[1], rot_quat[2], rot_quat[3])
        vehicle.teleport(pos, rot_quat, reset)

    def run_linear_crash(self):
        log.info('Running linear crash setting.')
        with open(os.path.abspath("../../output_test.csv"), "w+") as self.output:
            log.info(f"File opened: {self.output.name}")
            self.log_header()

            veh_a_pos = ImpactGenerator.gridmap['crashs']['linear']["vehicle_a"]['position']
            veh_a_ang = ImpactGenerator.gridmap['crashs']['linear']["vehicle_a"]['orientation']
            self.teleport(self.vehicle_a, veh_a_pos, veh_a_ang)

            veh_a_pos = ImpactGenerator.gridmap['crashs']['linear']["vehicle_b"]['position']
            veh_a_ang = ImpactGenerator.gridmap['crashs']['linear']["vehicle_b"]['orientation']
            self.teleport(self.vehicle_b, veh_a_pos, veh_a_ang)

            self.bng.control.step(60)

            self.vehicle_a.ai.set_target(self.vehicle_b.vid)
            self.vehicle_a.ai.set_speed(15)

            while True:
                self.vehicle_b.sensors.poll()
                self.vehicle_a.sensors.poll()
                self.log_line()
                if self.vehicle_a_damage['damage'] > 10:
                    self.vehicle_a.ai.set_mode("disabled")
                    self.stop_car(self.vehicle_a)
                    break

    def run_no_crash(self):
        log.info('Running linear crash setting.')

        self.bng.control.step(60)

        self.vehicle_a.ai.set_mode("span")
        self.vehicle_a.ai.set_aggression(0.1)

    def stop_car(self, vehicle):
        vehicle.control(steering=0, throttle=0,
                        brake=1, parkingbrake=1, gear=0)
        # self.bng.control.step(5*60)
        # wait for 5 seconds
        timerNow=0
        while timerNow < 5*60:
            self.bng.control.step(1)
            timerNow += 1
            self.vehicle_b.sensors.poll()
            self.vehicle_a.sensors.poll()
            self.log_line()
            time.sleep(1/60)



    def log_header(self):
        self.output.write(f"time,airspeed,gx,gy,gz,damage,crash_flag\n")

    def get_csv_line(self):
        time = self.vehicle_b_timer["time"]
        airspeed = self.vehicle_b_electrics["airspeed"]
        damage = self.vehicle_b_damage["damage"]
        gx = self.vehicle_b_gforce["gx"]
        gy = self.vehicle_b_gforce["gy"]
        gz = self.vehicle_b_gforce["gz"]
        current_damage = self.vehicle_b_damage["damage"]
        log.info(f'Current damage {round(current_damage)}')
        crash_flag = current_damage > self.prev_frame_damage*1.0005
        self.prev_frame_damage = current_damage
        return f"{time},{airspeed},{gx},{gy},{gz},{round(damage)},{crash_flag}\n"

    def log_line(self):
        self.output.write(self.get_csv_line())

    def run(self):
        log.info('Starting up BeamNG instance.')
        self.bng.open()
        try:
            log.info('Setting up BeamNG instance.')
            self.setup()

            # win32pipe.ConnectNamedPipe(self.dataPipeA, None)
            # win32pipe.ConnectNamedPipe(self.dataPipeB, None)

            # pylint: disable-next = unused-variable
            for i in range(1):
                self.run_linear_crash()
        finally:
            # win32api.CloseHandle(self.dataPipeA)
            # win32api.CloseHandle(self.dataPipeB)
            log.info('Closing BeamNG instance.')
            self.bng.close()

    def log_to_pipe(self, pipe, data):
        # win32file.WriteFile(pipe, data)
        pass
