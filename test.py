'''
.. module:: checkpoints
    :platform: Windows
    :synopsis: Simple demo on how to use checkpoints in a scenario.

.. moduleauthor:: Pascale Maul <pmaul@beamng.gmbh>
'''
from beamngpy import BeamNGpy, Scenario, Vehicle

if __name__ == '__main__':
    with BeamNGpy('localhost', 64256, home="C:/Users/yanni/Documents/Games/BeamNG.tech/BeamNG.tech.v0.30.6.0") as bng:
        scenario = Scenario('smallgrid', 'waypoint demo')
        vehicle = Vehicle('test_car', model='etk800', rot_quat=(0, 0, 0, 1))
        scenario.add_vehicle(vehicle)

        positions = [(0, -10, 0), (0, -20, 0), (0, -30, 0)]
        scales = [(1.0, 1.0, 1.0)] * 3
        scenario.add_checkpoints(positions, scales)
        scenario.make(bng)

        bng.scenario.load(scenario)
        bng.scenario.start()
        wp = scenario.find_waypoints()[-1]

        bng.step(60)

        vehicle.ai_set_speed(50 * 3.6)
        vehicle.ai_set_mode("manual")
        print(wp.id, wp.name)
        vehicle.ai_set_waypoint(wp.id)
        input('press \'Enter\' to exit demo')
