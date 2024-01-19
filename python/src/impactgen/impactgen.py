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

    def sample_data(self, vehicle_list):
        if vehicle_list[0].vehicle_timer["time"] - self.last_sample_time >= 1 / self.sample_per_sec:
            for vehicle in vehicle_list:
                vehicle.log_line()
            self.last_sample_time = vehicle_list[0].vehicle_timer["time"]

    def sensors_poll(self, vehicle_list):
        for vehicle in vehicle_list:
            vehicle.sensors.poll()

    def stop_vehicles(self, vehicle_list):
        for vehicle in vehicle_list:
            vehicle.emmergency_break()

    def park_vehicles(self, vehicle_list):
        for vehicle in vehicle_list:
            vehicle.park()

    def recover_vehicles(self, vehicle_list):
        for vehicle in vehicle_list:
            vehicle.recover()

    def run_crash_360(self, speed, step):
        log.info('Running random crash setting.')
        for i in range(0, 360, step):
            dir_path = os.path.abspath(f"../data/360/{speed}")
            file_a = os.path.abspath(f"../data/360/{speed}/{i}_a.csv")
            file_b = os.path.abspath(f"../data/360/{speed}/{i}_b.csv")
            os.makedirs(dir_path, mode=777, exist_ok=True)
            self.vehicle_a.output = open(file_a, "w+", encoding="utf8")
            self.vehicle_b.output = open(file_b, "w+", encoding="utf8")
            vehicle_list = [self.vehicle_a, self.vehicle_b]

            self.vehicle_a.log_header()
            self.vehicle_b.log_header()

            self.vehicle_a.teleport(
                (0, 0, 0.2), angle_to_quat((0, 0, i)))
            self.vehicle_b.teleport(
                (0, 150, 0.2), angle_to_quat((0, 0, 0)))

            self.bng.step(10)

            self.sensors_poll(vehicle_list)

            self.last_sample_time = self.vehicle_a.vehicle_timer["time"]
            step_time = 0

            running = True

            while running:
                self.sensors_poll(vehicle_list)
                if self.vehicle_a.vehicle_damage['damage'] < 10:
                    self.vehicle_b.set_speed(speed)
                elif self.vehicle_a.vehicle_electrics["airspeed"] > 1 or self.vehicle_b.vehicle_electrics["airspeed"] > 1:
                    self.stop_vehicles(vehicle_list)
                    step_time = self.vehicle_b.vehicle_timer["time"]
                elif self.vehicle_b.vehicle_timer["time"] - step_time < self.delay_after_crash:
                    self.park_vehicles(vehicle_list)
                else:
                    running = False
                self.sample_data(vehicle_list)

            self.recover_vehicles(vehicle_list)
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
            vehicle_list = [self.vehicle_b]

            self.vehicle_b.log_header()

            pos_x = 100 + j * 20
            self.vehicle_b.teleport(
                (pos_x, 150, 0.2), angle_to_quat((0, 0, 0)))

            self.bng.switch_vehicle(self.vehicle_b)

            self.bng.step(10)
            self.sensors_poll(vehicle_list)
            self.last_sample_time = vehicle_list[0].vehicle_timer["time"]
            running = True

            while running:
                self.sensors_poll(vehicle_list)
                if vehicle_list[0].vehicle_damage['damage'] < 10:
                    self.vehicle_b.set_speed(speed)
                elif vehicle_list[0].vehicle_electrics["airspeed"] > 1:
                    self.stop_vehicles(vehicle_list)
                    step_time = vehicle_list[0].vehicle_timer["time"]
                elif vehicle_list[0].vehicle_timer["time"] - step_time < self.delay_after_crash:
                    self.park_vehicles(vehicle_list)
                else:
                    running = False
                self.sample_data(vehicle_list)

            j += 1
            self.vehicle_b.output.close()
            self.recover_vehicles(vehicle_list)
            self.bng.step(10)
            self.scenario.restart()

    def run_abs(self, speed, break_delay):
        log.info('Running ABS')
        dir_path = os.path.abspath("../data/abs")
        file = os.path.abspath(f"../data/abs/{speed}_b.csv")
        os.makedirs(dir_path, mode=777, exist_ok=True)
        self.vehicle_b.output = open(file, "w+", encoding="utf8")
        vehicle_list = [self.vehicle_b]

        self.vehicle_b.log_header()

        self.vehicle_b.teleport(
            (0, 150, 0.2), angle_to_quat((0, 0, 0)))

        self.vehicle_a.teleport(
            (-30, 150, 0.2), angle_to_quat((0, 0, 0)))

        self.bng.step(10)
        self.bng.switch_vehicle(self.vehicle_b)

        self.vehicle_b.sensors.poll()

        self.last_sample_time = self.vehicle_b.vehicle_timer["time"]
        start_time = vehicle_list[0].vehicle_timer["time"]
        stopped_time = 0

        self.bng.step(10)
        self.sensors_poll(vehicle_list)
        self.last_sample_time = vehicle_list[0].vehicle_timer["time"]
        running = True

        while running:
            self.sensors_poll(vehicle_list)
            if vehicle_list[0].vehicle_timer["time"] - start_time < 10:
                self.vehicle_b.set_speed(speed)
            elif vehicle_list[0].vehicle_electrics["airspeed"] > 1:
                self.stop_vehicles(vehicle_list)
                stopped_time = vehicle_list[0].vehicle_timer["time"]
            elif vehicle_list[0].vehicle_timer["time"] - stopped_time < self.delay_after_crash:
                self.park_vehicles(vehicle_list)
            else:
                running = False
            self.sample_data(vehicle_list)

        self.recover_vehicles(vehicle_list)
        self.vehicle_b.output.close()
        self.bng.step(10)
        self.scenario.restart()

    def run_abs_avoid(self, speed, step, break_dist):
        pass

    def run(self):
        log.info('Starting up BeamNG instance.')
        self.bng.open()
        try:
            log.info('Setting up BeamNG instance.')
            self.setup()

            # win32pipe.ConnectNamedPipe(self.dataPipeA, None)
            # win32pipe.ConnectNamedPipe(self.dataPipeB, None)

            # pylint: disable-next = unused-variable
            for speed in [130]:
                self.target_speed = speed
                # self.run_crash_wall(speed, 5, 0)
                # self.run_crash_360(speed, 20)
                self.run_abs(speed, 2)
        finally:
            # win32api.CloseHandle(self.dataPipeA)
            # win32api.CloseHandle(self.dataPipeB)
            log.info('Closing BeamNG instance.')
            self.bng.close()

    def log_to_pipe(self, pipe, data):
        # win32file.WriteFile(pipe, data)
        pass
