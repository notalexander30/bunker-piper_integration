#!/usr/bin/env python3
"""Start the complete Bunker Nav-Man workflow.

The implementation remains in the legacy launcher so existing deployments keep
working while this clearer command becomes the documented entry point.
"""

import importlib.util
from pathlib import Path
import sys


implementation_path = Path(__file__).with_name("start_iliyas_abot_in_trystan.py")
spec = importlib.util.spec_from_file_location("nav_man_workflow", implementation_path)
if spec is None or spec.loader is None:
    raise RuntimeError(f"Unable to load workflow implementation: {implementation_path}")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
raise SystemExit(module.main(sys.argv[1:]))
