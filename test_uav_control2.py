# test_uav_control.py
import unittest

import pytest
from unittest.mock import MagicMock
from uav_control2 import UAVControl
from pymavlink import mavutil
import logging

# Отключаем логирование во время тестов для более чистого вывода
logging.getLogger('uav_control').disabled = True


class TestUAVControl:
    def test_init_success(self, mocker):
        # Мокаем mavutil.mavlink_connection и wait_heartbeat
        mock_connection = mocker.patch('pymavlink.mavutil.mavlink_connection')
        mock_master = MagicMock()
        mock_connection.return_value = mock_master
        mock_master.wait_heartbeat.return_value = None

        uav = UAVControl('tcp:127.0.0.1:5760')
        assert uav.master == mock_master
        mock_master.wait_heartbeat.assert_called_once()

    def test_init_failure(self, mocker):
        mock_connection = mocker.patch('pymavlink.mavutil.mavlink_connection')
        mock_connection.side_effect = Exception("Ошибка подключения")

        with pytest.raises(ConnectionError, match="Failed to connect to UAV: Ошибка подключения"):
            UAVControl('tcp:127.0.0.1:5760')

    def test_arm_success(self):
        mock_master = MagicMock()
        uav = UAVControl.__new__(UAVControl)
        uav.master = mock_master

        uav.arm()

        mock_master.arducopter_arm.assert_called_once()
        mock_master.motors_armed_wait.assert_called_once()

    def test_arm_failure(self):
        mock_master = MagicMock()
        mock_master.arducopter_arm.side_effect = Exception(
            "Ошибка армирования")
        uav = UAVControl.__new__(UAVControl)
        uav.master = mock_master

        with pytest.raises(RuntimeError, match="Failed to arm UAV: Ошибка армирования"):
            uav.arm()

    def test_disarm_success(self):
        mock_master = MagicMock()
        uav = UAVControl.__new__(UAVControl)
        uav.master = mock_master

        uav.disarm()

        mock_master.arducopter_disarm.assert_called_once()
        mock_master.motors_disarmed_wait.assert_called_once()

    def test_disarm_failure(self):
        mock_master = MagicMock()
        mock_master.arducopter_disarm.side_effect = Exception("Ошибка disarm")
        uav = UAVControl.__new__(UAVControl)
        uav.master = mock_master

        with pytest.raises(RuntimeError, match="Failed to disarm UAV: Ошибка disarm"):
            uav.disarm()

    def test_takeoff_success(self, mocker):
        mock_master = MagicMock()
        mock_msg = MagicMock()
        mock_msg.lat = 500000000  # 50 градусов
        mock_msg.lon = 500000000
        mock_master.recv_match.return_value = mock_msg

        uav = UAVControl.__new__(UAVControl)
        uav.master = mock_master
        uav.set_mode = MagicMock()
        uav.wait_command_ack = MagicMock(return_value=True)
        uav.arm = MagicMock()
        uav.seq = 0  # Инициализируем seq

        uav.takeoff(10)

        uav.set_mode.assert_called_with('GUIDED')
        uav.arm.assert_called_once()
        mock_master.recv_match.assert_called_with(
            type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        mock_master.mav.command_long_send.assert_called()
        uav.wait_command_ack.assert_called_with(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)

    def test_takeoff_invalid_altitude(self):
        uav = UAVControl.__new__(UAVControl)
        with pytest.raises(ValueError, match="Высота должна быть положительной"):
            uav.takeoff(-5)

    def test_takeoff_no_gps(self):
        mock_master = MagicMock()
        mock_master.recv_match.return_value = None  # Нет GPS данных

        uav = UAVControl.__new__(UAVControl)
        uav.master = mock_master
        uav.set_mode = MagicMock()
        uav.arm = MagicMock()

        with pytest.raises(RuntimeError, match="Failed to take off: Не удалось получить текущие координаты для взлёта"):
            uav.takeoff(10)

    def test_takeoff_command_ack_failure(self):
        mock_master = MagicMock()
        mock_msg = MagicMock()
        mock_msg.lat = 500000000
        mock_msg.lon = 500000000
        mock_master.recv_match.return_value = mock_msg

        uav = UAVControl.__new__(UAVControl)
        uav.master = mock_master
        uav.set_mode = MagicMock()
        uav.arm = MagicMock()
        uav.wait_command_ack = MagicMock(return_value=False)
        uav.seq = 0  # Инициализируем seq

        with pytest.raises(RuntimeError, match="Failed to take off: Команда взлёта не подтверждена"):
            uav.takeoff(10)

    def test_set_mode_success(self):
        mock_master = MagicMock()
        mock_master.mode_mapping.return_value = {'GUIDED': 4}

        uav = UAVControl.__new__(UAVControl)
        uav.master = mock_master

        uav.set_mode('GUIDED')
        mock_master.set_mode.assert_called_with(4)

    def test_set_mode_unknown_mode(self):
        mock_master = MagicMock()
        mock_master.mode_mapping.return_value = {'GUIDED': 4}

        uav = UAVControl.__new__(UAVControl)
        uav.master = mock_master

        with pytest.raises(ValueError, match="Неизвестный режим: UNKNOWN"):
            uav.set_mode('UNKNOWN')

    def test_set_mode_mapping_failure(self):
        mock_master = MagicMock()
        mock_master.mode_mapping.return_value = None

        uav = UAVControl.__new__(UAVControl)
        uav.master = mock_master

        with pytest.raises(RuntimeError, match="Не удалось получить список режимов полёта"):
            uav.set_mode('GUIDED')

    def test_get_telemetry_global_position(self):
        mock_msg = MagicMock()
        mock_msg.get_type.return_value = 'GLOBAL_POSITION_INT'
        mock_msg.lat = 500000000
        mock_msg.lon = 500000000
        mock_msg.alt = 10000  # в мм

        mock_master = MagicMock()
        mock_master.recv_match.return_value = mock_msg

        uav = UAVControl.__new__(UAVControl)
        uav.master = mock_master

        telemetry = uav.get_telemetry()
        assert telemetry == {'lat': 50.0, 'lon': 50.0, 'alt': 10.0}

    def test_get_telemetry_attitude(self):
        mock_msg = MagicMock()
        mock_msg.get_type.return_value = 'ATTITUDE'
        mock_msg.roll = 0.1
        mock_msg.pitch = 0.2
        mock_msg.yaw = -0.3

        mock_master = MagicMock()
        mock_master.recv_match.return_value = mock_msg

        uav = UAVControl.__new__(UAVControl)
        uav.master = mock_master

        telemetry = uav.get_telemetry()
        assert telemetry == {'roll': 0.1, 'pitch': 0.2, 'yaw': -0.3}

    def test_get_telemetry_no_message(self):
        mock_master = MagicMock()
        mock_master.recv_match.return_value = None

        uav = UAVControl.__new__(UAVControl)
        uav.master = mock_master

        telemetry = uav.get_telemetry()
        assert telemetry is None

    def test_get_telemetry_invalid_latitude(self):
        mock_msg = MagicMock()
        mock_msg.get_type.return_value = 'GLOBAL_POSITION_INT'
        mock_msg.lat = 1000000000  # Некорректная широта
        mock_msg.lon = 500000000
        mock_msg.alt = 10000

        mock_master = MagicMock()
        mock_master.recv_match.return_value = mock_msg

        uav = UAVControl.__new__(UAVControl)
        uav.master = mock_master

        telemetry = uav.get_telemetry()
        assert telemetry is None  # Ожидаем, что метод вернёт None из-за ошибки

    def test_get_telemetry_vfr_hud(self):
        mock_msg = MagicMock()
        mock_msg.get_type.return_value = 'VFR_HUD'
        mock_msg.groundspeed = 15.0
        mock_msg.airspeed = 16.0
        mock_msg.heading = 90

        mock_master = MagicMock()
        mock_master.recv_match.return_value = mock_msg

        uav = UAVControl.__new__(UAVControl)
        uav.master = mock_master

        telemetry = uav.get_telemetry()
        assert telemetry == {
            'groundspeed': 15.0,
            'airspeed': 16.0,
            'heading': 90
        }

    def test_get_telemetry_sys_status(self):
        mock_msg = MagicMock()
        mock_msg.get_type.return_value = 'SYS_STATUS'
        mock_msg.voltage_battery = 11000  # в мВ
        mock_msg.battery_remaining = 80  # в процентах

        mock_master = MagicMock()
        mock_master.recv_match.return_value = mock_msg

        uav = UAVControl.__new__(UAVControl)
        uav.master = mock_master

        telemetry = uav.get_telemetry()
        assert telemetry == {
            'battery_voltage': 11.0,
            'battery_remaining': 80
        }

    def test_wait_command_ack_success(self):
        mock_master = MagicMock()
        ack_msg = MagicMock()
        ack_msg.command = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
        ack_msg.result = mavutil.mavlink.MAV_RESULT_ACCEPTED
        mock_master.recv_match.return_value = ack_msg

        uav = UAVControl.__new__(UAVControl)
        uav.master = mock_master

        result = uav.wait_command_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)
        assert result is True

    def test_wait_command_ack_failure(self):
        mock_master = MagicMock()
        ack_msg = MagicMock()
        ack_msg.command = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
        ack_msg.result = mavutil.mavlink.MAV_RESULT_DENIED
        mock_master.recv_match.return_value = ack_msg

        uav = UAVControl.__new__(UAVControl)
        uav.master = mock_master

        result = uav.wait_command_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)
        assert result is False

    def test_wait_command_ack_timeout(self):
        mock_master = MagicMock()
        mock_master.recv_match.return_value = None  # Не получаем ack

        uav = UAVControl.__new__(UAVControl)
        uav.master = mock_master

        result = uav.wait_command_ack(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, timeout=1)
        assert result is False

    def test_goto_success(self):
        mock_master = MagicMock()
        uav = UAVControl.__new__(UAVControl)
        uav.master = mock_master
        uav.wait_command_ack = MagicMock(return_value=True)
        uav.seq = 0  # Инициализируем seq

        uav.goto(50.0, 50.0, 10.0)

        mock_master.mav.mission_count_send.assert_called_with(
            uav.master.target_system,
            uav.master.target_component,
            1,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        )
        mock_master.mav.mission_item_send.assert_called()
        uav.wait_command_ack.assert_called_with(
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT)

    def test_goto_command_ack_failure(self):
        mock_master = MagicMock()
        uav = UAVControl.__new__(UAVControl)
        uav.master = mock_master
        uav.wait_command_ack = MagicMock(return_value=False)
        uav.seq = 0  # Инициализируем seq

        with pytest.raises(RuntimeError, match="Failed to go to waypoint: Команда полёта к точке не подтверждена"):
            uav.goto(50.0, 50.0, 10.0)

    def test_goto_exception(self):
        mock_master = MagicMock()
        mock_master.mav.mission_count_send.side_effect = Exception(
            "Ошибка отправки миссии")

        uav = UAVControl.__new__(UAVControl)
        uav.master = mock_master
        uav.seq = 0  # Инициализируем seq

        with pytest.raises(RuntimeError, match="Failed to go to waypoint: Ошибка отправки миссии"):
            uav.goto(50.0, 50.0, 10.0)

    def test_land_success(self):
        mock_master = MagicMock()
        mock_master.mode_mapping.return_value = {'LAND': 9}
        uav = UAVControl.__new__(UAVControl)
        uav.master = mock_master
        uav.wait_command_ack = MagicMock(return_value=True)

        uav.land()

        mock_master.set_mode.assert_called_with(9)
        uav.wait_command_ack.assert_called_with(
            mavutil.mavlink.MAV_CMD_NAV_LAND)
        assert uav.wait_command_ack.call_count == 1

    def test_land_command_ack_failure(self):
        mock_master = MagicMock()
        mock_master.mode_mapping.return_value = {'LAND': 9}
        uav = UAVControl.__new__(UAVControl)
        uav.master = mock_master
        uav.wait_command_ack = MagicMock(return_value=False)

        with pytest.raises(RuntimeError, match="Failed to land: Команда посадки не подтверждена"):
            uav.land()


if __name__ == '__main__':
    unittest.main()
