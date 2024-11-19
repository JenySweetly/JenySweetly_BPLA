# %%
# test_mission_planner.py

import unittest
from unittest.mock import MagicMock, patch, call
from mission_planner import MissionPlanner


# %%
class TestMissionPlanner(unittest.TestCase):
    def setUp(self):
        # Создание mock-объекта для UAVControl
        self.patcher = patch('mission_planner.UAVControl')
        self.mock_uav_control_class = self.patcher.start()
        self.mock_uav = MagicMock()
        self.mock_uav_control_class.return_value = self.mock_uav
        self.planner = MissionPlanner('udp:127.0.0.1:14550')

    def tearDown(self):
        # Остановка патчера
        self.patcher.stop()

    def test_execute_mission_success(self):
        # Тест успешного выполнения миссии
        waypoints = [
            (55.0, 37.0, 10.0),
            (55.0001, 37.0001, 20.0),
            (55.0002, 37.0002, 15.0)
        ]

        # Настройка side_effect для get_telemetry
        telemetry_data = iter([
            {'lat': 55.0, 'lon': 37.0, 'alt': 10.0},
            {'lat': 55.0001, 'lon': 37.0001, 'alt': 20.0},
            {'lat': 55.0002, 'lon': 37.0002, 'alt': 15.0}
        ])

        self.mock_uav.get_telemetry.side_effect = lambda: next(
            telemetry_data, None)

        self.planner.execute_mission(waypoints)

        self.mock_uav.arm.assert_called_once()
        self.mock_uav.set_mode.assert_any_call('GUIDED')
        self.mock_uav.takeoff.assert_called_once_with(waypoints[0][2])

        expected_calls = [call(wp[0], wp[1], wp[2]) for wp in waypoints]
        self.assertEqual(self.mock_uav.goto.call_count, len(waypoints))
        self.mock_uav.goto.assert_has_calls(expected_calls)

        self.mock_uav.set_mode.assert_any_call('RTL')
        self.mock_uav.disarm.assert_called_once()

    def test_execute_mission_failure(self):
        # Тест провала выполнения миссии из-за недостижения точки
        waypoints = [
            (55.0, 37.0, 10.0),
            (55.0001, 37.0001, 20.0)
        ]

        # Настройка get_telemetry для возвращения неизменных координат
        self.mock_uav.get_telemetry.return_value = {
            'lat': 55.0,
            'lon': 37.0,
            'alt': 10.0
        }

        with self.assertRaises(Exception) as context:
            self.planner.execute_mission(waypoints)

        self.assertIn('Не удалось достичь точки 1', str(context.exception))
        self.mock_uav.disarm.assert_called_once()


# %%
if __name__ == "__main__":
    unittest.main()
