# from __future__ import annotations
from datetime import datetime
import click
from click import Context
from click_aliases import ClickAliasedGroup
import builtins

import logging
from rich.logging import RichHandler

logging.basicConfig(
    level=logging.INFO,
    format="%(message)s",
    datefmt="[%X]",
    handlers=[RichHandler()]
)

CONTEXT_SETTINGS = dict(help_option_names=['-h', '--help'], max_content_width=800)

@click.group(cls=ClickAliasedGroup, context_settings=CONTEXT_SETTINGS)
@click.pass_context
def cli(ctx: Context, **kwargs):
    pass


@cli.command(aliases=['i'])
@click.argument("output_file", nargs=1, required=False, default=f"rosmetasys_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.json")
@click.option("-v", "--verbose", is_flag=True, default=False, help='Print more output.')
# @click.option("-p", "--publish", is_flag=True, default=False, help='Publish the .')

@click.pass_context
def export(ctx: Context, **kwargs):
    """
    Exports the system meta information to an [OUTPUT_FILE].
    
    """
    from rosmetasys.export import export

    try:
        export(**kwargs)
    except Exception as e:
        from pakk.logger import Logger
        Logger.get_console().print_exception()
    except KeyboardInterrupt:
        from pakk.logger import Logger
        Logger.get_console().print("Keyboard interrupt at CLI...")
    #     pass
    # finally:
    #     ctx.exit(0)
