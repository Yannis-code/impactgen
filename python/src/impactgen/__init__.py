# pylint: disable = missing-module-docstring
# pylint: disable = missing-function-docstring

import os

from .impactgen import ImpactGenerator


def read(fil):
    fil = os.path.join(os.path.dirname(__file__), fil)
    with open(fil, encoding='utf-8') as f:
        return f.read()


__version__ = read('version.txt')
