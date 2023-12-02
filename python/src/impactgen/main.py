# pylint: disable = missing-module-docstring
# pylint: disable = missing-function-docstring
# pylint: disable = unused-argument

import logging as log
import os
import os.path
import shutil
import sys

import click

from . import ImpactGenerator

LOG = 'impactgen.log'
CFG = 'impactgen.yml'


def log_exception(extype, value, trace):
    log.exception('Uncaught exception:', exc_info=(extype, value, trace))


def setup_logging(log_file=None):
    handlers = []
    if log_file:
        if os.path.exists(log_file):
            # pylint: disable-next = consider-using-f-string
            backup = '{}.1'.format(log_file)
            shutil.move(log_file, backup)
        file_handler = log.FileHandler(log_file, 'w', 'utf-8')
        handlers.append(file_handler)

    term_handler = log.StreamHandler()
    handlers.append(term_handler)
    fmt = '%(asctime)s %(levelname)-8s %(message)s'
    log.basicConfig(handlers=handlers, format=fmt, level=log.INFO)

    sys.excepthook = log_exception

    log.info('Started impactgen logging.')


@click.group()
@click.option('--log-file', type=click.Path(dir_okay=False), default=LOG)
@click.pass_context
def cli(ctx=None, log_file=None, **opts):
    setup_logging(log_file=log_file)


@cli.command()
@click.argument('bng-home', type=click.Path(file_okay=False, exists=True))
@click.argument('output', type=click.Path(file_okay=False))
@click.pass_context
def generate(ctx, bng_home, output):
    if not os.path.exists(output):
        os.makedirs(output)

    gen = ImpactGenerator(bng_home, output,)
    gen.run()


if __name__ == '__main__':
    cli(obj={})
