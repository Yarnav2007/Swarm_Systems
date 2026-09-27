#!/usr/bin/env python3
"""Shadow OakD-Lite with per-model default Gazebo sensor topic names.

The stock depth sensor can publish to one global /depth_camera topic across
all x500_depth models. A temporary copy without that explicit topic lets Gazebo
generate a model-scoped topic so depth frames cannot be misattributed.
"""

import os
import shutil
import sys
from pathlib import Path
from xml.etree import ElementTree as ET


def find_model(px4_root, destination):
    configured = os.getenv('SWARM_OAKD_MODEL_DIR')
    if configured:
        path = Path(configured).expanduser()
        if (path / 'model.sdf').is_file():
            return path
        raise FileNotFoundError(f'SWARM_OAKD_MODEL_DIR has no model.sdf: {path}')
    home = Path.home()
    roots = [px4_root / 'Tools/simulation/gz/models',
             px4_root / 'build/px4_sitl_default/rootfs/models',
             home / '.gz/fuel', home / '.ignition/fuel']
    roots.extend(Path(p) for p in os.getenv('GZ_SIM_RESOURCE_PATH', '').split(':') if p)
    for root in roots:
        if not root.is_dir() or root.resolve() == destination.resolve():
            continue
        candidate = root / 'OakD-Lite'
        if (candidate / 'model.sdf').is_file():
            return candidate
        # Fuel caches are nested by host/owner/model/version.
        for folder, dirs, files in os.walk(root):
            dirs[:] = [d for d in dirs if d != 'camera_model_override']
            if (Path(folder).name.lower() == 'oakd-lite' or
                    Path(folder).parent.name.lower() == 'oakd-lite') and 'model.sdf' in files:
                return Path(folder)
    return None


def patch_camera_model(source, destination):
    width = max(80, min(640, int(os.getenv('SWARM_CAMERA_WIDTH', '320'))))
    height = max(60, min(480, int(os.getenv('SWARM_CAMERA_HEIGHT', '240'))))
    fps = max(1, min(10, int(os.getenv('SWARM_CAMERA_FPS', '3'))))
    target = destination / 'OakD-Lite'
    if target.exists():
        shutil.rmtree(target)
    shutil.copytree(source, target)
    sdf = target / 'model.sdf'
    tree = ET.parse(sdf)
    removed = 0
    for sensor in tree.findall('.//sensor'):
        if sensor.get('type') not in ('camera', 'depth_camera', 'rgbd_camera'):
            continue
        rate = sensor.find('update_rate')
        if rate is None:
            rate = ET.SubElement(sensor, 'update_rate')
        rate.text = str(fps)
        for camera in sensor.findall('camera'):
            for image in camera.findall('image'):
                for name, value in (('width', width), ('height', height)):
                    element = image.find(name)
                    if element is None:
                        element = ET.SubElement(image, name)
                    element.text = str(value)
        topic = sensor.find('topic')
        if topic is not None:
            # Gazebo's default topic includes world/model/link/sensor names.
            sensor.remove(topic)
            removed += 1
    if removed < 1:
        raise RuntimeError(f'No explicit camera topic found in {source / "model.sdf"}; '
                           'verify topic names with gz topic -l before attributing depth')
    tree.write(sdf, encoding='utf-8', xml_declaration=True)
    return removed, width, height, fps


def main():
    destination = Path(sys.argv[1]).resolve()
    px4_root = Path(sys.argv[2]).resolve()
    source = find_model(px4_root, destination)
    if source is None:
        raise FileNotFoundError('OakD-Lite model not cached; provide '
                                'SWARM_OAKD_MODEL_DIR=/path/to/OakD-Lite')
    destination.mkdir(parents=True, exist_ok=True)
    removed, width, height, fps = patch_camera_model(source, destination)
    print(f'Camera model: {removed} shared topic override(s) removed; '
          f'{width}x{height} at {fps} Hz to reduce renderer load. '
          'Sensor topics should include their drone model ID.', flush=True)


if __name__ == '__main__':
    try:
        main()
    except (FileNotFoundError, RuntimeError, ValueError, ET.ParseError, OSError) as exc:
        print(f'Camera setup: {exc}', file=sys.stderr)
        raise SystemExit(1)
