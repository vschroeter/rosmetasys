from __future__ import annotations
import click
from click import Context
from click_aliases import ClickAliasedGroup

import logging
from rich.logging import RichHandler

from InquirerPy import inquirer

from rosmetasys.console import console


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


@cli.command(aliases=['e'])
@click.argument("system_name", nargs=1, required=False, default=f"rosmetasys")
@click.option("-v", "--verbose", is_flag=True, default=False, help='Print more output.')
@click.option("-i", "--interactive", is_flag=True, default=False, help='Interactive version asking you the options.')
@click.option("-a", "--anonymize", is_flag=True, default=False, help='Anonymizing of node names and topics.')
@click.option("-z", "--zip", is_flag=True, default=False, help='Create the zip file.')
@click.option("-p", "--pretty", is_flag=True, default=False, help='Pretty formatting for the json.')
@click.option("--author", help='Author of the dataset.', default="")
@click.option("--description", help='Description of the dataset.', default="")
@click.option("--no-consent", is_flag=True, default=False, help='No consent to the use of the data set under the CC BY-SA license.')
@click.pass_context
def export(ctx: Context, **kwargs):
    """
    Export the system meta information, named [SYSTEM_NAME].
    
    """
    from rosmetasys.export import export

    if kwargs["interactive"]:
        kwargs["system_name"] = inquirer.text(message="System name:", default=kwargs["system_name"]).execute()
        kwargs["anonymize"] = inquirer.confirm(message="Anonymizing of node names and topics:", default=kwargs["anonymize"]).execute()
        kwargs["author"] = inquirer.text(message="Author of the dataset:", default=kwargs["author"]).execute()
        kwargs["description"] = inquirer.text(message="Description of the dataset:", default=kwargs["description"]).execute()
        kwargs["no_consent"] = not inquirer.confirm(message="Consent to the use of the data set under the CC BY-SA license:", default=not kwargs["no_consent"]).execute()
        kwargs["zip"] = inquirer.confirm(message="Create the zip file:", default=True).execute()


    if kwargs["verbose"]:
        logging.getLogger().setLevel(logging.DEBUG)

    try:
        export(**kwargs)
    except KeyboardInterrupt:
        console.print("Aborting.")

