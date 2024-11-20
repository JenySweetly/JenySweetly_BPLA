# tests/test_uav_control.py

import pytest
from unittest.mock import MagicMock, patch
from uav_control2 import UAVControl
from pymavlink import mavutil


def test_init_success(mocker):
    mock_connection = mocker.patch('pymavlink.mavutil.mavlink_connection')
    mock_master = MagicMock()
    mock_connection.return_value = mock_master
    mock_master.wait_heartbeat.return_value = None

    uav = UAVControl('tcp:127.0.0.1:5760')
    assert uav.master == mock_master
    mock_master.wait_heartbeat.assert_called_once()


def test_arm_success(mocker):
    mock_master = MagicMock()
    uav = UAVControl.__new__(UAVControl)
    uav.master = mock_master

    uav.arm()
    mock_master.arducopter_arm.assert_called_once()
    mock_master.motors_armed_wait.assert_called_once()


def test_disarm_success(mocker):
    mock_master = MagicMock()
    uav = UAVControl.__new__(UAVControl)
    uav.master = mock_master

    uav.disarm()
    mock_master.arducopter_disarm.assert_called_once()
    mock_master.motors_disarmed_wait.assert_called_once()


def test_takeoff_success(mocker):
    mock_master = MagicMock()
    mock_master.recv_match.return_value = MagicMock(
        lat=500000000, lon=500000000)
    uav = UAVControl.__new__(UAVControl)
    uav.master = mock_master
    uav.seq = 0
    uav.wait_command_ack = MagicMock(return_value=True)
    uav.set_mode = MagicMock()
    uav.arm = MagicMock()

    uav.takeoff(10)
    uav.set_mode.assert_called_with('GUIDED')
    uav.arm.assert_called_once()
    mock_master.mav.command_long_send.assert_called_once()
    uav.wait_command_ack.assert_called_with(
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)


def test_takeoff_invalid_altitude():
    uav = UAVControl.__new__(UAVControl)
    with pytest.raises(ValueError):
        uav.takeoff(-5)


def test_set_mode_success(mocker):
    mock_master = MagicMock()
    mock_master.mode_mapping.return_value = {'GUIDED': 4}
    uav = UAVControl.__new__(UAVControl)
    uav.master = mock_master

    uav.set_mode('GUIDED')
    mock_master.set_mode.assert_called_with(4)


def test_get_telemetry_global_position(mocker):
    mock_msg = MagicMock()
    mock_msg.get_type.return_value = 'GLOBAL_POSITION_INT'
    mock_msg.lat = 500000000
    mock_msg.lon = 500000000
    mock_msg.alt = 10000

    mock_master = MagicMock()
    mock_master.recv_match.return_value = mock_msg

    uav = UAVControl.__new__(UAVControl)
    uav.master = mock_master

    telemetry = uav.get_telemetry()
    assert telemetry == {'lat': 50.0, 'lon': 50.0, 'alt': 10.0}


def test_init_failure(mocker):
    mock_connection = mocker.patch('pymavlink.mavutil.mavlink_connection')
    mock_connection.side_effect = Exception("Connection error")

    with pytest.raises(ConnectionError):
        UAVControl('tcp:127.0.0.1:5760')


def test_state_transition_connection(mocker):
    mock_connection = mocker.patch('pymavlink.mavutil.mavlink_connection')
    mock_master = MagicMock()
    mock_connection.return_value = mock_master
    mock_master.wait_heartbeat.return_value = None

    uav = UAVControl('tcp:127.0.0.1:5760')
    assert uav.master == mock_master


def test_state_transition_arm(mocker):
    mock_master = MagicMock()
    uav = UAVControl.__new__(UAVControl)
    uav.master = mock_master

    uav.arm()
    mock_master.arducopter_arm.assert_called_once()


def test_state_transition_landing(mocker):
    mock_master = MagicMock()
    uav = UAVControl.__new__(UAVControl)
    uav.master = mock_master
    uav.set_mode = MagicMock()

    uav.set_mode('LAND')
    uav.set_mode.assert_called_with('LAND')


if __name__ == "__main__":
    unittest.main()
