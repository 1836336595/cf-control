"""Global, trajectory, and per-aircraft ROS parameter routing."""




GLOBAL_CTBR_PARAMETER_ROOT = "/ctbr_controller"
TRAJECTORY_PARAMETER_ROOT = "/ctbr_trajectory"
PAYLOAD_PARAMETER_ROOT = "/slung_payload"
_PARAMETER_MISSING = object()

def vehicle_parameter_namespace(vehicle_id):
    """Return the dedicated absolute controller-parameter root for one CF."""
    try:
        normalized_id = int(vehicle_id)
    except (TypeError, ValueError) as error:
        raise ValueError("Crazyflie ID 必须是正整数: %r" % (vehicle_id,)) from error
    if isinstance(vehicle_id, bool) or normalized_id <= 0:
        raise ValueError("Crazyflie ID 必须是正整数: %r" % (vehicle_id,))
    return "/ctbr_controller_cf%d" % normalized_id


class CtbrParameterResolver:
    """Read CTBR parameters from global, trajectory, and per-CF namespaces."""

    def __init__(self, get_param, has_param, vehicle_id):
        if not callable(get_param) or not callable(has_param):
            raise TypeError("get_param 和 has_param 必须可调用")
        self._get_param = get_param
        self._has_param = has_param
        self.vehicle_id = int(vehicle_id)
        self.vehicle_root = vehicle_parameter_namespace(self.vehicle_id)
        if not self._has_param(self.vehicle_root):
            raise ValueError(
                "CF%d 缺少专属参数块：%s" % (self.vehicle_id, self.vehicle_root)
            )
        root_value = self._get_param(self.vehicle_root)
        if not isinstance(root_value, dict):
            raise ValueError(
                "CF%d 专属参数块必须是字典：%s" % (
                    self.vehicle_id, self.vehicle_root
                )
            )

    @staticmethod
    def _path(root, name):
        name = str(name).strip("/")
        if not name:
            raise ValueError("参数名不能为空")
        return root + "/" + name

    def _read(self, root, name, default=_PARAMETER_MISSING):
        path = self._path(root, name)
        if default is _PARAMETER_MISSING:
            return self._get_param(path)
        return self._get_param(path, default)

    def global_param(self, name, default=_PARAMETER_MISSING):
        return self._read(GLOBAL_CTBR_PARAMETER_ROOT, name, default)

    def trajectory_param(self, name, default=_PARAMETER_MISSING):
        return self._read(TRAJECTORY_PARAMETER_ROOT, name, default)

    def vehicle_param(self, name, default=_PARAMETER_MISSING):
        return self._read(self.vehicle_root, name, default)
