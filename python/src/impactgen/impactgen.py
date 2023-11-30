# pylint: disable=unused-argument
# pylint: disable=unused-import
# pylint: disable=missing-module-docstring
# pylint: disable=missing-class-docstring
# pylint: disable=missing-function-docstring
# pylint: disable=dangerous-default-value
# pylint: disable=too-many-arguments
# pylint: disable=unspecified-encoding
# pylint: disable=attribute-defined-outside-init

import copy
import hashlib
import io
import json
import logging as log
import random
import time
import zipfile
from collections import defaultdict
from pathlib import Path

import numpy as np
from beamngpy import BeamNGpy, Scenario, Vehicle
from PIL import Image

from . import materialmngr
from .api import ImpactGenAPI

PART_ANNOTATIONS = 'tech/part_annotation_config.json'
OBJ_ANNOTATIONS = 'tech/annotations.json'


class OptionSpace:

    def __init__(self, options):
        self.options = options
        self.count = 1

        for col in options:
            self.count *= len(col)

        self.sampled = set()

    def exhausted(self):
        return len(self.sampled) == self.count - 1

    def sample_new(self):
        if self.exhausted():
            return None

        unique = random.randint(0, self.count)
        while unique in self.sampled:
            unique = random.randint(0, self.count)
        self.sampled.add(unique)

        ret = []
        for col in self.options:
            idx = unique % len(col)
            ret.append(col[idx])
            unique //= len(col)

        return tuple(ret)


class ImpactGenerator:
    parts = [
        'etk800_bumper_F',
        'etk800_bumperbar_F',
        'etk800_bumper_R',
        'etk800_fender_L',
        'etk800_fender_R',
        'etk800_hood',
        'etk800_towhitch',
        'etk800_steer',
        'etk800_radiator',
        'etk800_roof_wagon',
        'wheel_F_5',
    ]

    emptyable = {
        'etk800_bumperbar_F',
        'etk800_towhitch',
    }

    wca = {
        'level': 'west_coast_usa',

        'a_spawn': (-270.75, 678, 74.9),
        'b_spawn': (-260.25, 678, 74.9),

        'pole_pos': (-677.15, 848, 75.1),

        'linear_pos_a': (-630, 65, 103.4),
        'linear_pos_b': (-619, 77, 102.65),
        'linear_rot_b': (0, 0, 45.5),

        't_pos_a': (-440, 688, 75.1),
        't_pos_b': (-453, 700, 75.1),
        't_rot_b': (0, 0, 315),

        'ref_pos': (-18, 610, 75),
    }

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

    smallgrid = {
        'level': 'smallgrid',

        'a_spawn': (-270.75, 678, 0.1),
        'b_spawn': (-260.25, 678, 0.1),

        'pole_pos': (-677.15, 848, 0.1),
        'pole': (-682, 842, 0),

        'linear_pos_a': (-630, 65, 0.1),
        'linear_pos_b': (-619, 77, 0.1),
        'linear_rot_b': (0, 0, 45.5),

        't_pos_a': (-440, 688, 0.1),
        't_pos_b': (-453, 700, 0.1),
        't_rot_b': (0, 0, 315),

        'ref_pos': (321, 321, 0.1),
    }

    def __init__(self, bng_home, output, config,
                 poolsize=2, smallgrid=False, sim_mtx=None, similarity=0.5,
                 random_select=False, single=False):
        self.bng_home = bng_home
        self.output = Path(output)
        self.config = config
        self.smallgrid = smallgrid
        self.single = single

        self.impactgen_mats = None
        if smallgrid:
            mats = materialmngr.get_impactgen_materials(bng_home)
            self.impactgen_mats = sorted(list(mats))

        self.poolsize = poolsize
        self.similarity = similarity
        self.sim_mtx = sim_mtx
        self.random_select = random_select

        self.pole_space = None
        self.t_bone_space = None
        self.linear_space = None
        self.nocrash_space = None

        self.total_possibilities = 0

        self.bng = BeamNGpy('localhost', 64256, home=bng_home)

        self.scenario = None

        scenario_props = ImpactGenerator.wca
        if smallgrid:
            scenario_props = ImpactGenerator.smallgrid

        self.vehicle_a = Vehicle('vehicle_a', model='etk800')
        self.vehicle_b = Vehicle('vehicle_b', model='etk800')

        self.scenario = Scenario(scenario_props['level'], 'impactgen')
        self.scenario.add_vehicle(
            self.vehicle_a, pos=scenario_props['a_spawn'])
        self.scenario.add_vehicle(
            self.vehicle_b, pos=scenario_props['b_spawn'])

        self.vehicle_a_parts = defaultdict(set)
        self.vehicle_a_config = None
        self.vehicle_b_config = None

        self.api = ImpactGenAPI(self.bng)

    def generate_nocrash_space(self, props):
        nocrash_options = []
        for part in ImpactGenerator.parts:  # Vary each configurable part
            nocrash_options.append(self.vehicle_a_parts[part])
        self.nocrash_space = OptionSpace(nocrash_options)

    def generate_pole_space(self, props):
        pole_options = [(False, True)]  # Vehicle facing forward/backward
        pole_options.append(np.linspace(-0.75, 0.75, 5))  # Position offset
        pole_options.append(np.linspace(0.15, 0.5, 4))  # Throttle intensity
        for part in ImpactGenerator.parts:  # Vary each configurable part
            pole_options.append(self.vehicle_a_parts[part])
        self.pole_space = OptionSpace(pole_options)

    def generate_t_bone_space(self, props):
        t_options = [(False, True)]  # Vehicle hit left/right
        t_options.append(np.linspace(-30, 30, 11))  # A rotation offset
        t_options.append(np.linspace(-1.5, 1.5, 5))  # B pos. offset
        t_options.append(np.linspace(0.2, 0.5, 4))  # B throttle
        for part in ImpactGenerator.parts:
            t_options.append(self.vehicle_a_parts[part])
        self.t_bone_space = OptionSpace(t_options)

    def generate_linear_space(self, props):
        linear_options = [(False, True)]  # Vehicle hit front/back
        linear_options.append(np.linspace(-15, 15, 5))  # A rot. offset
        linear_options.append(np.linspace(-1.33, 1.33, 5))  # B pos. offset
        linear_options.append(np.linspace(0.25, 0.5, 4))  # B throttle
        for part in ImpactGenerator.parts:
            linear_options.append(self.vehicle_a_parts[part])
        self.linear_space = OptionSpace(linear_options)

    def generate_spaces(self):
        props = ImpactGenerator.wca
        if self.smallgrid:
            props = ImpactGenerator.smallgrid

        self.generate_nocrash_space(props)
        self.generate_t_bone_space(props)
        self.generate_linear_space(props)
        self.generate_pole_space(props)

    def scan_parts(self, parts, known=set()):
        with open('out.json', 'w') as outfile:
            outfile.write(json.dumps(parts, indent=4, sort_keys=True))

        for part_type in ImpactGenerator.parts:
            options = parts[part_type]
            self.vehicle_a_parts[part_type].update(options)

    def init_parts(self):
        self.vehicle_a_config = self.vehicle_a.get_part_config()
        self.vehicle_b_config = self.vehicle_b.get_part_config()

        b_parts = self.vehicle_b_config['parts']
        b_parts['etk800_licenseplate_R'] = 'etk800_licenseplate_R_EU'
        b_parts['etk800_licenseplate_F'] = 'etk800_licenseplate_F_EU'
        b_parts['licenseplate_design_2_1'] = 'license_plate_germany_2_1'

        options = self.vehicle_a.get_part_options()
        self.scan_parts(options)

        for k in self.vehicle_a_parts.keys():
            self.vehicle_a_parts[k] = list(self.vehicle_a_parts[k])
            if k in ImpactGenerator.emptyable:
                self.vehicle_a_parts[k].append('')

    def init_settings(self):
        self.bng.settings.set_particles_enabled(False)

        self.generate_spaces()

        log.info('%s pole crash possibilities.', self.pole_space.count)
        log.info('%s T-Bone crash possibilities.', self.t_bone_space.count)
        log.info('%s parallel crash possibilities.', self.linear_space.count)
        log.info('%s no crash possibilities.', self.nocrash_space.count)

        self.total_possibilities = \
            self.pole_space.count + \
            self.t_bone_space.count + \
            self.linear_space.count + \
            self.nocrash_space.count
        log.info('%s total incidents possible.', self.total_possibilities)

    def get_vehicle_config(self, setting):
        parts = dict()
        for idx, part in enumerate(ImpactGenerator.parts):
            parts[part] = setting[idx]
        refwheel = parts['wheel_F_5']
        parts['wheel_R_5'] = refwheel.replace('_F', '_R')

        # Force licence plate to always be German
        parts['etk800_licenseplate_R'] = 'etk800_licenseplate_R_EU'
        parts['etk800_licenseplate_F'] = 'etk800_licenseplate_F_EU'
        parts['licenseplate_design_2_1'] = 'license_plate_germany_2_1'

        config = copy.deepcopy(self.vehicle_a_config)
        config['parts'] = parts
        return config

    def set_image_properties(self):
        self.api.set_image_properties(
            self.config['imageWidth'], self.config['imageHeight'], self.config['colorFormat'], self.config['annotFormat'],
            self.config['cameraRadius'], self.config['cameraHeight'], self.config['fov'])

    def setup(self):
        self.scenario.make(self.bng)
        log.debug('Loading scenario...')
        self.bng.scenario.load(self.scenario)
        log.debug('Setting steps per second...')
        self.bng.settings.set_deterministic(50)
        log.debug('Starting scenario...')
        self.bng.scenario.start()

        self.init_parts()
        self.init_settings()

        log.debug('Setting annotation properties.')
        self.part_path = (Path(self.bng_home) / PART_ANNOTATIONS).absolute()
        self.obj_path = (Path(self.bng_home, OBJ_ANNOTATIONS)).absolute()
        self.set_image_properties()

    def settings_exhausted(self):
        return self.t_bone_space.exhausted() and \
            self.linear_space.exhausted() and \
            self.pole_space.exhausted() and \
            self.nocrash_space.exhausted()

    def run_t_bone_crash(self):
        log.info('Running t-bone crash setting.')
        if self.t_bone_space.exhausted():
            log.debug('T-Bone crash setting exhausted.')
            return None

        props = ImpactGenerator.wca
        if self.smallgrid:
            props = ImpactGenerator.smallgrid

        setting = self.t_bone_space.sample_new()
        side, angle, offset, throttle = setting[:4]
        config = setting[4:]
        config = self.get_vehicle_config(config)

        if side:
            angle += 225
        else:
            angle += 45

        pos_a = props['t_pos_a']

        rot_b = props['t_rot_b']
        pos_b = list(props['t_pos_b'])
        pos_b[0] += offset

        log.debug('Sending t-bone crash config.')
        self.api.run_t_bone_crash(
            self.vehicle_a.vid, self.vehicle_b.vid, config, pos_a, angle, pos_b, rot_b, throttle)
        log.debug('T-Bone crash response received.')

        return setting

    def run_linear_crash(self):
        log.info('Running linear crash setting.')
        if self.linear_space.exhausted():
            log.debug('Linear crash settings exhausted.')
            return None

        props = ImpactGenerator.wca
        if self.smallgrid:
            props = ImpactGenerator.smallgrid

        setting = self.linear_space.sample_new()
        back, angle, offset, throttle = setting[:4]
        config = setting[4:]
        config = self.get_vehicle_config(config)

        if back:
            angle += 225
        else:
            offset += 1.3
            angle += 45

        pos_a = props['linear_pos_a']

        rot_b = props['linear_rot_b']
        pos_b = list(props['linear_pos_b'])
        pos_b[0] += offset

        log.debug('Sending linear crash config.')
        self.api.run_linear_crash(
            self.vehicle_a.vid, self.vehicle_b.vid, config, pos_a, angle, pos_b, rot_b, throttle)
        log.debug('Linear crash response received.')

        return setting

    def run_pole_crash(self):
        log.info('Running pole crash setting.')
        if self.pole_space.exhausted():
            log.debug('Pole crash settings exhausted.')
            return None

        props = ImpactGenerator.wca
        if self.smallgrid:
            props = ImpactGenerator.smallgrid

        setting = self.pole_space.sample_new()
        back, offset, throttle = setting[:3]
        config = setting[3:]
        config = self.get_vehicle_config(config)

        angle = 45
        if back:
            angle = 225
            offset += 0.85
            throttle = -throttle

        pos = list(props['pole_pos'])
        pos[0] += offset

        pole = props['pole'] if self.smallgrid else None
        log.debug('Sending pole crash config.')
        self.api.run_pole_crash(
            self.vehicle_a.vid, config, pos, angle, throttle, pole)
        log.debug('Got pole crash response.')

        return setting

    def run_no_crash(self):
        log.info('Running non-crash scenario.')
        if self.nocrash_space.exhausted():
            return None

        setting = self.nocrash_space.sample_new()
        log.info('Got new setting: %s', setting)

        vehicle_config = self.get_vehicle_config(setting)
        log.info('Got new vehicle config: %s', vehicle_config)

        log.info('Sending Non-Crash request')
        self.api.run_no_crash(self.vehicle_a.vid, vehicle_config)
        log.info('Non-crash finished.')

        return setting

    def run_incident(self, incident):
        log.info('Setting up next incident.')
        self.bng.ui.display_message('Setting up next incident...')
        setting = incident()
        return setting

    def run_incidents(self):
        log.info('Enumerating possible incidents.')
        count = 1

        incidents = [
            self.run_t_bone_crash,
            self.run_linear_crash,
            self.run_pole_crash,
            self.run_no_crash,
        ]

        while not self.settings_exhausted():
            log.info('Running incident %s of %s...', count,
                     self.total_possibilities)
            self.bng.scenario.restart()
            log.info('Scenario restarted.')
            time.sleep(5.0)
            self.vehicle_b.set_part_config(self.vehicle_b_config)
            log.info('Vehicle B config set.')

            incident = incidents[count % len(incidents)]
            if self.run_incident(incident) is None:
                log.info('Ran out of options for: %s', incident)
                incidents.remove(incident)  # Possibility space exhausted

            count += 1

    def run(self):
        log.info('Starting up BeamNG instance.')
        self.bng.open(['tech/impactgen/crashOutput'])
        try:
            log.info('Setting up BeamNG instance.')
            self.setup()
            self.run_incidents()
        finally:
            log.info('Closing BeamNG instance.')
            self.bng.close()
