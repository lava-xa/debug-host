#!/usr/bin/env python3
"""Launch the Aero Hand desktop control GUI.

Run this file from the ``debug-host`` directory with::

    uv run python3 main.py
"""

from gui import main as run_gui


def main() -> None:
    """Start the GUI application."""
    run_gui()


if __name__ == "__main__":
    main()
