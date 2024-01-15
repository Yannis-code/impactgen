# pylint: disable = missing-module-docstring
# pylint: disable = missing-class-docstring
# pylint: disable = missing-function-docstring
# pylint: disable = consider-using-with
# pylint: disable = line-too-long

import os
import logging as log
import numpy as np
# import win32pipe
# import win32file
# import win32api
from beamngpy import BeamNGpy, Scenario, StaticObject, angle_to_quat
from .vehicule import Vehicule


class ImpactGenerator:

    def __init__(self, bng_home, output, single=False):
        self.bng_home = bng_home
        self.single = single
        # self.dataPipeA = win32pipe.CreateNamedPipe(
        #     r'\\.\pipe\impactgenA', win32pipe.PIPE_ACCESS_OUTBOUND, win32pipe.PIPE_TYPE_MESSAGE | win32pipe.PIPE_NOWAIT, 2, 65536, 65536, 300, None)

        # self.dataPipeB = win32pipe.CreateNamedPipe(
        #     r'\\.\pipe\impactgenA', win32pipe.PIPE_ACCESS_OUTBOUND, win32pipe.PIPE_TYPE_MESSAGE | win32pipe.PIPE_NOWAIT, 2, 65536, 65536, 300, None)

        self.bng = BeamNGpy('localhost', 64256, home=bng_home)

        self.scenario = None

        self.vehicle_a = Vehicule("vehicle_a", model="vivace")
        self.vehicle_b = Vehicule("vehicle_b", model="vivace")
        self.vehicle_c = Vehicule("vehicle_c", model="vivace")

        self.walls = []

        j = 0
        for i in range(-70, 75, 5):
            pos_x = 100 + j * 20
            wall = StaticObject(name=f'wall-{i}', pos=(pos_x, 0, 0),
                                rot_quat=angle_to_quat((90, i, 0)), scale=(15, 3, 3),
                                shape='/art/shapes/objects/s_drywall.dae')
            self.walls.append(wall)
            j += 1

        self.scenario = Scenario("smallgrid", 'impactgen')
        self.scenario.add_vehicle(self.vehicle_a, pos=(50, 0, 0.5))
        self.scenario.add_vehicle(self.vehicle_b, pos=(60, 0, 0.5))
        self.scenario.add_vehicle(self.vehicle_c, pos=(70, 0, 0.5))
        for wall in self.walls:
            self.scenario.add_object(wall)

        self.sample_per_sec = 10
        self.last_sample_time = 0
        self.delay_after_crash = 2
        self.target_speed = 0

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

    def run_crash_360(self, speed, step):
        log.info('Running random crash setting.')
        for i in range(0, 360, step):
            dir_path = os.path.abspath(f"../data/360/{speed}")
            file_a = os.path.abspath(f"../data/360/{speed}/{i}_a.csv")
            file_b = os.path.abspath(f"../data/360/{speed}/{i}_b.csv")
            os.makedirs(dir_path, mode=777, exist_ok=True)
            self.vehicle_a.output = open(file_a, "w+", encoding="utf8")
            self.vehicle_b.output = open(file_b, "w+", encoding="utf8")

            self.vehicle_a.log_header()
            self.vehicle_b.log_header()

            self.vehicle_a.teleport(
                (0, 0, 0.2), angle_to_quat((0, 0, i)))
            self.vehicle_b.teleport(
                (0, 150, 0.2), angle_to_quat((0, 0, 0)))

            self.bng.step(10)

            self.vehicle_b.sensors.poll()
            self.vehicle_a.sensors.poll()

            self.last_sample_time = self.vehicle_a.vehicle_timer["time"]
            step_time = self.vehicle_a.vehicle_timer["time"]

            log.info('Driving')
            while self.vehicle_a.vehicle_damage['damage'] < 10:
                self.vehicle_b.control(0, brake=0, parkingbrake=0)
                self.vehicle_b.set_velocity(speed / 3.6, 1.0)
                self.vehicle_a.sensors.poll()
                self.vehicle_b.sensors.poll()
                if self.vehicle_a.vehicle_timer["time"] - self.last_sample_time >= 1 / self.sample_per_sec:
                    self.vehicle_a.log_line()
                    self.vehicle_b.log_line()
                    self.last_sample_time = self.vehicle_a.vehicle_timer["time"]

            log.info('Breaking')
            while self.vehicle_a.vehicle_electrics["airspeed"] > 1 or self.vehicle_b.vehicle_electrics["airspeed"] > 1:
                self.vehicle_a.sensors.poll()
                self.vehicle_b.sensors.poll()
                self.stop_car(self.vehicle_b)
                self.stop_car(self.vehicle_a)
                if self.vehicle_b.vehicle_timer["time"] - self.last_sample_time >= 1 / self.sample_per_sec:
                    self.vehicle_a.log_line()
                    self.vehicle_b.log_line()
                    self.last_sample_time = self.vehicle_a.vehicle_timer["time"]

            step_time = self.vehicle_a.vehicle_timer["time"]

            log.info('End Delay')
            while self.vehicle_b.vehicle_timer["time"] - step_time < self.delay_after_crash:
                self.vehicle_a.sensors.poll()
                self.vehicle_b.sensors.poll()
                self.stop_car(self.vehicle_b)
                self.stop_car(self.vehicle_a)
                if self.vehicle_b.vehicle_timer["time"] - self.last_sample_time >= 1 / self.sample_per_sec:
                    self.vehicle_a.log_line()
                    self.vehicle_b.log_line()
                    self.last_sample_time = self.vehicle_a.vehicle_timer["time"]

            self.vehicle_a.recover()
            self.vehicle_b.recover()
            self.vehicle_a.output.close()
            self.vehicle_b.output.close()
            self.bng.step(10)
            self.scenario.restart()

    def run_crash_360_break(self, speed, step, break_dist):
        log.info('Running random crash setting.')
        for i in range(0, 360, step):
            dir_path = os.path.abspath(f"../data/360_break/{speed}")
            file_a = os.path.abspath(f"../data/360_break/{speed}/{i}_a.csv")
            file_b = os.path.abspath(f"../data/360_break/{speed}/{i}_b.csv")
            os.makedirs(dir_path, mode=777, exist_ok=True)
            self.vehicle_a.output = open(file_a, "w+", encoding="utf8")
            self.vehicle_b.output = open(file_b, "w+", encoding="utf8")

            self.vehicle_a.log_header()
            self.vehicle_b.log_header()

            self.vehicle_a.teleport(
                (0, 0, 0.2), angle_to_quat((0, 0, i)))
            self.vehicle_b.teleport(
                (0, 150, 0.2), angle_to_quat((0, 0, 0)))

            self.bng.step(10)

            self.vehicle_b.sensors.poll()
            self.vehicle_a.sensors.poll()

            self.last_sample_time = self.vehicle_a.vehicle_timer["time"]

            log.info('Driving')
            while self.vehicle_a.vehicle_damage['damage'] < 10:
                self.vehicle_a.sensors.poll()
                self.vehicle_b.sensors.poll()

                pos = self.vehicle_a.vehicle_timer["pos"]
                if (np.sqrt(pos[0]**2 + pos[1]**2) > break_dist):
                    self.vehicle_b.control(0, brake=0, parkingbrake=0)
                    self.vehicle_b.set_velocity(speed / 3.6, 1.0)
                else:
                    self.vehicle_b.control(0, brake=1, parkingbrake=0)

                if self.vehicle_a.vehicle_timer["time"] - self.last_sample_time >= 1 / self.sample_per_sec:
                    self.vehicle_a.log_line()
                    self.vehicle_b.log_line()
                    self.last_sample_time = self.vehicle_a.vehicle_timer["time"]

            log.info('Breaking')
            while self.vehicle_a.vehicle_electrics["airspeed"] > 1 or self.vehicle_b.vehicle_electrics["airspeed"] > 1:
                self.vehicle_a.sensors.poll()
                self.vehicle_b.sensors.poll()
                self.stop_car(self.vehicle_b)
                self.stop_car(self.vehicle_a)
                if self.vehicle_b.vehicle_timer["time"] - self.last_sample_time >= 1 / self.sample_per_sec:
                    self.vehicle_a.log_line()
                    self.vehicle_b.log_line()
                    self.last_sample_time = self.vehicle_a.vehicle_timer["time"]

            step_time = self.vehicle_a.vehicle_timer["time"]

            log.info('End Delay')
            while self.vehicle_b.vehicle_timer["time"] - step_time < self.delay_after_crash:
                self.vehicle_a.sensors.poll()
                self.vehicle_b.sensors.poll()
                self.stop_car(self.vehicle_b)
                self.stop_car(self.vehicle_a)
                if self.vehicle_b.vehicle_timer["time"] - self.last_sample_time >= 1 / self.sample_per_sec:
                    self.vehicle_a.log_line()
                    self.vehicle_b.log_line()
                    self.last_sample_time = self.vehicle_a.vehicle_timer["time"]

            self.vehicle_a.recover()
            self.vehicle_b.recover()
            self.vehicle_a.output.close()
            self.vehicle_b.output.close()
            self.bng.step(10)
            self.scenario.restart()

    def run_crash_wall(self, speed, step, break_dist):
        log.info('Running random crash setting.')
        j = 0
        for i in range(-70, 70, step):
            dir_path = os.path.abspath(f"../data/wall/{speed}")
            file = os.path.abspath(f"../data/wall/{speed}/{i}_a.csv")
            os.makedirs(dir_path, mode=777, exist_ok=True)
            self.vehicle_b.output = open(file, "w+", encoding="utf8")

            self.vehicle_b.log_header()

            pos_x = 100 + j * 20
            self.vehicle_b.teleport(
                (pos_x, 150, 0.2), angle_to_quat((0, 0, 0)))

            self.bng.switch_vehicle(self.vehicle_b)

            self.bng.step(10)

            self.vehicle_b.sensors.poll()

            self.last_sample_time = self.vehicle_b.vehicle_timer["time"]

            log.info('Driving')
            while self.vehicle_b.vehicle_damage['damage'] < 10:
                self.vehicle_b.sensors.poll()
                pos = self.vehicle_b.vehicle_state["pos"]
                if np.sqrt(pos[0]**2 + pos[1]**2) > break_dist:
                    self.vehicle_b.control(0, brake=0, parkingbrake=0)
                    self.vehicle_b.set_velocity(speed / 3.6, 1.0)
                else:
                    self.vehicle_b.control(0, brake=1, parkingbrake=0)

                if self.vehicle_b.vehicle_timer["time"] - self.last_sample_time >= 1 / self.sample_per_sec:
                    self.vehicle_b.log_line()
                    self.last_sample_time = self.vehicle_b.vehicle_timer["time"]

            log.info('Breaking')
            while self.vehicle_b.vehicle_electrics["airspeed"] > 1:
                self.vehicle_b.sensors.poll()
                self.stop_car(self.vehicle_b)
                if self.vehicle_b.vehicle_timer["time"] - self.last_sample_time >= 1 / self.sample_per_sec:
                    self.vehicle_b.log_line()
                    self.last_sample_time = self.vehicle_b.vehicle_timer["time"]

            step_time = self.vehicle_b.vehicle_timer["time"]

            log.info('End Delay')
            while self.vehicle_b.vehicle_timer["time"] - step_time < self.delay_after_crash:
                self.vehicle_b.sensors.poll()
                self.stop_car(self.vehicle_b)
                if self.vehicle_b.vehicle_timer["time"] - self.last_sample_time >= 1 / self.sample_per_sec:
                    self.vehicle_b.log_line()
                    self.last_sample_time = self.vehicle_b.vehicle_timer["time"]
            j += 1
            self.vehicle_b.output.close()
            self.vehicle_b.recover()
            self.bng.step(10)
            self.scenario.restart()

    def run_abs(self, speed, break_delay):
        log.info('Running ABS')
        dir_path = os.path.abspath("../data/abs")
        file = os.path.abspath(f"../data/abs/{speed}_b.csv")
        os.makedirs(dir_path, mode=777, exist_ok=True)
        self.vehicle_b.output = open(file, "w+", encoding="utf8")

        self.vehicle_b.log_header()

        self.vehicle_b.teleport(
            (0, 150, 0.2), angle_to_quat((0, 0, 0)))

        self.vehicle_a.teleport(
            (-30, 150, 0.2), angle_to_quat((0, 0, 0)))

        self.bng.step(10)
        self.bng.switch_vehicle(self.vehicle_b)

        self.vehicle_b.sensors.poll()

        self.last_sample_time = self.vehicle_b.vehicle_timer["time"]
        step_time = self.vehicle_b.vehicle_timer["time"]

        log.info('Accelerating')
        while self.vehicle_b.vehicle_timer["time"] - step_time < 5:
            self.vehicle_b.sensors.poll()
            if self.vehicle_b.vehicle_timer["time"] - self.last_sample_time >= 1 / self.sample_per_sec:
                self.vehicle_b.log_line()
                self.last_sample_time = self.vehicle_b.vehicle_timer["time"]
            self.vehicle_b.control(0, brake=0, parkingbrake=0)
            self.vehicle_b.set_velocity(speed / 3.6, 1.0)

        step_time = self.vehicle_b.vehicle_timer["time"]

        log.info('Breaking delay')
        while self.vehicle_b.vehicle_timer["time"] - step_time < break_delay:
            self.vehicle_b.sensors.poll()
            if self.vehicle_b.vehicle_timer["time"] - self.last_sample_time >= 1 / self.sample_per_sec:
                self.vehicle_b.log_line()
                self.last_sample_time = self.vehicle_b.vehicle_timer["time"]
            self.vehicle_b.control(0, brake=0, parkingbrake=0)
            self.vehicle_b.set_velocity(speed / 3.6, 1.0)

        log.info('Breaking')
        while self.vehicle_b.vehicle_electrics["airspeed"] > 1:
            self.vehicle_b.sensors.poll()
            self.stop_car(self.vehicle_b)
            if self.vehicle_b.vehicle_timer["time"] - self.last_sample_time >= 1 / self.sample_per_sec:
                self.vehicle_b.log_line()
                self.last_sample_time = self.vehicle_b.vehicle_timer["time"]

        step_time = self.vehicle_b.vehicle_timer["time"]

        log.info('End Delay')
        while self.vehicle_b.vehicle_timer["time"] - step_time < self.delay_after_crash:
            self.vehicle_b.sensors.poll()
            self.stop_car(self.vehicle_b)
            if self.vehicle_b.vehicle_timer["time"] - self.last_sample_time >= 1 / self.sample_per_sec:
                self.vehicle_b.log_line()
                self.last_sample_time = self.vehicle_b.vehicle_timer["time"]

        self.vehicle_b.recover()
        self.vehicle_b.output.close()
        self.bng.step(10)
        self.scenario.restart()

    def run_abs_avoid(self, speed, step, break_dist):
        pass

    def stop_car(self, vehicle):
        vehicle.control(steering=0, throttle=0,
                        brake=1, parkingbrake=1, gear=0)

    def run(self):
        log.info('Starting up BeamNG instance.')
        self.bng.open()
        try:
            log.info('Setting up BeamNG instance.')
            self.setup()

            # win32pipe.ConnectNamedPipe(self.dataPipeA, None)
            # win32pipe.ConnectNamedPipe(self.dataPipeB, None)

            # pylint: disable-next = unused-variable
            for speed in [130, 110, 90, 50, 30]:
                self.target_speed = speed
                self.run_crash_wall(speed, 5, 0)
                self.run_crash_360(speed, 20)
                self.run_abs(speed, 2)
        finally:
            # win32api.CloseHandle(self.dataPipeA)
            # win32api.CloseHandle(self.dataPipeB)
            log.info('Closing BeamNG instance.')
            self.bng.close()

    def log_to_pipe(self, pipe, data):
        # win32file.WriteFile(pipe, data)
        pass
