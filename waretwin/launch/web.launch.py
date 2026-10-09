import importlib.util
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    path = Path(get_package_share_directory('waretwin_web')) / 'runtime/launch_support.py'
    spec = importlib.util.spec_from_file_location('waretwin_launch_support', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.description()
