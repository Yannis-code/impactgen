# pylint: disable = missing-module-docstring
# pylint: disable = missing-class-docstring
# pylint: disable = missing-function-docstring

import os
import logging as log
from math import sin, cos, pi
# import win32pipe
# import win32file
# import win32api
from pathlib import Path
from scipy.spatial.transform import Rotation

from beamngpy import BeamNGpy, Scenario, Vehicle, StaticObject, angle_to_quat
from beamngpy.sensors import GForces, Electrics, Damage, State, Timer


class ImpactGenerator:

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

        self.vehicle_a = Vehicle("vehicle_a", model="covet")
        self.vehicle_b = Vehicle("vehicle_b", model="covet")

        self.setup_sensors()

        self.scenario = Scenario("smallgrid", 'impactgen')
        self.scenario.add_vehicle(self.vehicle_a, pos=(0, 0, 0.5))
        self.scenario.add_vehicle(self.vehicle_b, pos=(10, 0, 0.5))

        self.prev_frame_damage = 0

        self.sample_per_sec = 10
        self.last_sample_time = 0
        self.has_crash = False
        self.crash_time = 0
        self.delay_after_crash = 5

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

    def run_crash_360(self, speed, step):
        log.info('Running random crash setting.')
        for i in range(0, 360, step):
            with open(os.path.abspath(f"../data/360/{i}.csv"), "w+") as self.output:
                self.log_header()

                self.vehicle_a.teleport(
                    (0, 0, 0.2), angle_to_quat((0, 0, i)))
                self.vehicle_b.teleport(
                    (0, 150, 0.2), angle_to_quat((0, 0, 0)))

                self.bng.step(10)

                self.vehicle_b.sensors.poll()
                self.vehicle_a.sensors.poll()

                self.last_sample_time = self.vehicle_a_timer["time"]
                self.has_crash = False
                while True:
                    self.vehicle_a.sensors.poll()
                    self.vehicle_b.sensors.poll()
                    if self.vehicle_b_timer["time"] - self.last_sample_time >= 1 / self.sample_per_sec:
                        self.log_line()
                        self.last_sample_time = self.vehicle_a_timer["time"]
                    if self.has_crash:
                        if self.vehicle_a_timer["time"] - self.crash_time > self.delay_after_crash:
                            break
                    else:
                        self.vehicle_b.control(0, brake=0, parkingbrake=0)
                        self.vehicle_b.set_velocity(speed / 3.6, 1.0)
                        if self.vehicle_a_damage['damage'] > 10:
                            self.has_crash = True
                            self.last_sample_time = self.vehicle_a_timer["time"]
                            self.crash_time = self.vehicle_a_timer["time"]
                            self.stop_car(self.vehicle_a)
                            self.stop_car(self.vehicle_b)
                self.vehicle_a.recover()
                self.vehicle_b.recover()
                self.scenario.restart()

    def run_crash_wall(self, speed, step):
        log.info('Running random crash setting.')
        for i in range(0, 360, step):
            with open(os.path.abspath(f"../data/360/{i}.csv"), "w+") as self.output:
                self.log_header()

                self.vehicle_a.teleport(
                    (0, 0, 0.2), angle_to_quat((0, 0, i)))
                self.vehicle_b.teleport(
                    (0, 150, 0.2), angle_to_quat((0, 0, 0)))

                self.bng.step(10)

                self.vehicle_b.sensors.poll()
                self.vehicle_a.sensors.poll()

                self.last_sample_time = self.vehicle_a_timer["time"]
                self.has_crash = False
                while True:
                    self.vehicle_a.sensors.poll()
                    self.vehicle_b.sensors.poll()
                    if self.vehicle_b_timer["time"] - self.last_sample_time >= 1 / self.sample_per_sec:
                        self.log_line()
                        self.last_sample_time = self.vehicle_a_timer["time"]
                    if self.has_crash:
                        if self.vehicle_a_timer["time"] - self.crash_time > self.delay_after_crash:
                            break
                    else:
                        self.vehicle_b.control(0, brake=0, parkingbrake=0)
                        self.vehicle_b.set_velocity(speed / 3.6, 1.0)
                        if self.vehicle_a_damage['damage'] > 10:
                            self.has_crash = True
                            self.last_sample_time = self.vehicle_a_timer["time"]
                            self.crash_time = self.vehicle_a_timer["time"]
                            self.stop_car(self.vehicle_a)
                            self.stop_car(self.vehicle_b)
                self.vehicle_a.recover()
                self.vehicle_b.recover()
                self.scenario.restart()

    def stop_car(self, vehicle):
        vehicle.control(steering=0, throttle=0,
                        brake=1, parkingbrake=1, gear=0)

    def log_header(self):
        self.output.write(f"time,airspeed,gx,gy,gz,damage,crash_flag\n")

    def get_csv_line(self):
        time = self.vehicle_a_timer["time"]
        airspeed = self.vehicle_a_electrics["airspeed"]
        damage = self.vehicle_a_damage["damage"]
        gx = self.vehicle_a_gforce["gx"]
        gy = self.vehicle_a_gforce["gy"]
        gz = self.vehicle_a_gforce["gz"]
        current_damage = self.vehicle_a_damage["damage"]
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
            self.run_crash_wall(110, 45)
        finally:
            # win32api.CloseHandle(self.dataPipeA)
            # win32api.CloseHandle(self.dataPipeB)
            log.info('Closing BeamNG instance.')
            self.bng.close()

    def log_to_pipe(self, pipe, data):
        # win32file.WriteFile(pipe, data)
        pass
