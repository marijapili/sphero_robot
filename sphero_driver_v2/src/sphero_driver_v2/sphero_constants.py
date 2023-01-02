from spherov2.commands.power import BatteryStates
from diagnostic_msgs.msg import DiagnosticStatus

BATT_STATE_MAP = {
    BatteryStates.CHARGED: DiagnosticStatus.OK,
    BatteryStates.CHARGING: DiagnosticStatus.OK,
    BatteryStates.NOT_CHARGING: DiagnosticStatus.STALE,
    BatteryStates.OK: DiagnosticStatus.OK,
    BatteryStates.LOW: DiagnosticStatus.WARN,
    BatteryStates.CRITICAL: DiagnosticStatus.ERROR,
    BatteryStates.UNKNOWN: DiagnosticStatus.STALE
}

# TODO: These are made up.
ODOM_POSE_COVARIANCE = [1e-3, 0, 0, 0, 0, 0,
                        0, 1e-3, 0, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 1e3]

ODOM_TWIST_COVARIANCE = [1e-3, 0, 0, 0, 0, 0,
                         0, 1e-3, 0, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e3]

IMU_ORIENTATION_COVARIANCE = [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]
IMU_ANG_VEL_COVARIANCE = [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]
IMU_LIN_ACC_COVARIANCE = [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]
