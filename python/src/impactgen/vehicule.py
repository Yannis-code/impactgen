# pylint: disable = missing-module-docstring
# pylint: disable = missing-class-docstring
# pylint: disable = missing-function-docstring

from beamngpy import Vehicle
from beamngpy.sensors import GForces, Electrics, Damage, State, Timer


class Vehicule (Vehicle):
    def __init__(self, vid: str, model: str):
        super().__init__(vid, model)

        self.vehicle_gforce = GForces()
        self.vehicle_damage = Damage()
        self.vehicle_electrics = Electrics()
        self.vehicle_state = State()
        self.vehicle_timer = Timer()

        self.sensors.attach("vehicule_sensor", self.vehicle_gforce)
        self.sensors.attach("vehicule_electrics", self.vehicle_electrics)
        self.sensors.attach("vehicule_damage", self.vehicle_damage)
        self.sensors.attach("vehicule_state", self.vehicle_state)
        self.sensors.attach("vehicule_timer", self.vehicle_timer)

        self.output = None

        self.prev_frame_damage = 0

    def log_header(self):
        self.output.write("time,airspeed,gx,gy,gz,damage,crash_flag\n")

    def get_csv_line(self):
        time = self.vehicle_timer["time"]
        airspeed = self.vehicle_electrics["airspeed"]
        damage = self.vehicle_damage["damage"]
        gx = self.vehicle_gforce["gx"]
        gy = self.vehicle_gforce["gy"]
        gz = self.vehicle_gforce["gz"]
        current_damage = self.vehicle_damage["damage"]
        crash_flag = 1 if current_damage > self.prev_frame_damage else 0
        self.prev_frame_damage = current_damage
        return f"{time},{airspeed},{gx},{gy},{gz},{round(damage)},{crash_flag}\n"

    def set_speed(self, target_speed):
        target_speed_ms = target_speed / 3.6
        self.control(0, brake=0, parkingbrake=0)
        self.set_velocity(target_speed_ms, 1.0)

    def emmergency_break(self):
        self.control(steering=0, throttle=0, brake=1, gear=0)

    def park(self):
        self.control(steering=0, throttle=0, brake=1, parkingbrake=1, gear=0)

    def log_line(self):
        self.output.write(self.get_csv_line())
