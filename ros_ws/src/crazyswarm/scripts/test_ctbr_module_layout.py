"""Public entry points and control math must survive the module split."""

import ast
import importlib
from pathlib import Path


def test_independent_and_transport_math_import_without_ros():
    independent=importlib.import_module("ctbr_independent_controller")
    transport=importlib.import_module("ctbr_transport_vehicle_controller")
    geometric=importlib.import_module("ctbr_geometric_controller")
    assert independent.IndependentVehicleController
    assert transport.TransportVehicleController
    assert geometric.GeometricCtbrController
    for name in ["ctbr_independent_controller","ctbr_transport_vehicle_controller",
                 "ctbr_control_math","ctbr_geometric_controller"]:
        tree=ast.parse(Path(__file__).with_name(name+".py").read_text())
        for node in ast.walk(tree):
            if isinstance(node,ast.Import):
                assert all(alias.name!="rospy" for alias in node.names)
            if isinstance(node,ast.ImportFrom):
                assert node.module not in ["rospy","ctbr_controller"]


def test_ros_entry_reexports_the_real_node_classes_without_defining_them():
    from test_ctbr_controller_v2 import _controller_module
    module=_controller_module()
    assert module.CtbrControllerNode.__module__=="ctbr_vehicle_node"
    assert module.MultiCtbrControllerNode.__module__=="ctbr_fleet_node"
    assert module.GeometricCtbrController.__module__=="ctbr_geometric_controller"
    tree=ast.parse(Path(module.__file__).read_text())
    assert not any(isinstance(node,ast.ClassDef) for node in tree.body)
