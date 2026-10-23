"""Flight YAML must pass the same XML-RPC marshalling used by roslaunch."""

from pathlib import Path
import xml.etree.ElementTree as ET
from xmlrpc.client import dumps

import numpy as np
import pytest
import yaml

from slung_circle_reference import CircleTrajectoryConfig, PayloadCircleReference


PACKAGE = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("launch_name", ["hover_swarm.launch", "ctbr_controller.launch"])
def test_launch_yaml_parameters_can_be_marshaled_without_allow_none(launch_name):
    launch = ET.parse(PACKAGE / "launch" / launch_name)
    values = []
    for element in launch.iter("rosparam"):
        path = element.get("file")
        if path:
            path = Path(path.replace("$(find crazyswarm)", str(PACKAGE)))
            value = yaml.safe_load(path.read_text())
        else:
            value = yaml.safe_load(element.text or "{}")
        values.append(value)
    # A standard ROS1 master does not enable the XML-RPC None extension.
    dumps((values,), methodname="system.multicall", allow_none=False)


def test_automatic_circle_center_survives_ros_parameter_roundtrip():
    from xmlrpc.client import loads

    payload = yaml.safe_load((PACKAGE / "config" / "slung_payload.yaml").read_text())["slung_payload"]
    encoded = dumps((payload,), allow_none=False)
    decoded = loads(encoded)[0][0]
    config = CircleTrajectoryConfig.from_mapping(decoded["trajectory"])
    # Automatic circle centering is selected by omission even if the live
    # mission currently uses an explicit figure-eight crossing.
    circle_config = dict(decoded["trajectory"], mode="circle")
    circle_config.pop("center_xy", None)
    circle = PayloadCircleReference(np.array([1., 2., .6]), 20.,
                                   CircleTrajectoryConfig.from_mapping(circle_config))
    np.testing.assert_allclose(circle.center_position, [1.5, 2., .6])
    np.testing.assert_allclose(circle.entry_position, [1., 2., .6])
