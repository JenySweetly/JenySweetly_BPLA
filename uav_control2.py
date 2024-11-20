"""
Модуль для управления БПЛА с использованием MAVLink.
"""

import time
import math
import logging
from typing import Optional, Dict

from pymavlink import mavutil

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class UAVControl:
    """
    Класс для управления БПЛА через MAVLink.
    """

    def __init__(self, connection_string: str):
        """
        Инициализация подключения к БПЛА.

        Args:
            connection_string (str): Строка подключения MAVLink.
        """
        try:
            self.master = mavutil.mavlink_connection(connection_string)
            self.master.wait_heartbeat()
            logger.info("Соединение установлено")
            self.seq = 0  # Инициализация последовательного номера миссии
        except Exception as e:
            logger.error("Ошибка подключения: %s", e)
            raise ConnectionError(f"Failed to connect to UAV: {e}") from e

    def arm(self) -> None:
        """
        Arm БПЛА для начала работы двигателей.
        """
        try:
            self.master.arducopter_arm()
            self.master.motors_armed_wait()
            logger.info("БПЛА армирован")
        except Exception as e:
            logger.error("Ошибка армирования БПЛА: %s", e)
            raise RuntimeError(f"Failed to arm UAV: {e}") from e

    def disarm(self) -> None:
        """
        Disarm БПЛА для остановки двигателей.
        """
        try:
            self.master.arducopter_disarm()
            self.master.motors_disarmed_wait()
            logger.info("БПЛА disarmed")
        except Exception as e:
            logger.error("Ошибка disarm БПЛА: %s", e)
            raise RuntimeError(f"Failed to disarm UAV: {e}") from e

    def takeoff(self, altitude: float) -> None:
        """
        Команда на взлёт до заданной высоты.

        Args:
            altitude (float): Целевая высота взлёта в метрах.
        """
        if altitude <= 0:
            raise ValueError("Высота должна быть положительной")

        try:
            self.set_mode('GUIDED')
            self.arm()

            # Получение текущих координат
            msg = self.master.recv_match(
                type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
            if msg:
                current_lat = msg.lat / 1e7
                current_lon = msg.lon / 1e7
            else:
                raise RuntimeError(
                    "Не удалось получить текущие координаты для взлёта")

            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,
                0, 0, 0, 0,
                current_lat,  # param5: Широта взлёта
                current_lon,  # param6: Долгота взлёта
                altitude      # param7: Высота взлёта
            )

            if not self.wait_command_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF):
                raise RuntimeError("Команда взлёта не подтверждена")
            logger.info("Взлёт на высоту %s метров", altitude)
        except Exception as e:
            logger.error("Ошибка взлёта: %s", e)
            raise RuntimeError(f"Failed to take off: {e}") from e

    def set_mode(self, mode: str) -> None:
        """
        Установка режима полёта БПЛА.

        Args:
            mode (str): Название режима (например, 'GUIDED', 'LAND').
        """
        mode_mapping = self.master.mode_mapping()
        if not isinstance(mode_mapping, dict):
            logger.error("Ошибка: mode_mapping() не вернул словарь")
            raise RuntimeError("Не удалось получить список режимов полёта")

        mode_id = mode_mapping.get(mode)
        if mode_id is None:
            raise ValueError(f"Неизвестный режим: {mode}")

        try:
            self.master.set_mode(mode_id)
            logger.info("Режим установлен: %s", mode)
        except Exception as e:
            logger.error("Ошибка установки режима %s: %s", mode, e)
            raise RuntimeError(f"Failed to set mode {mode}: {e}") from e

    def get_telemetry(self) -> Optional[Dict[str, float]]:
        """
        Получение телеметрических данных от БПЛА.

        Returns:
            Optional[Dict[str, float]]: Словарь с телеметрическими данными или None.
        """
        try:
            msg = self.master.recv_match(
                type=['GLOBAL_POSITION_INT', 'ATTITUDE'], blocking=True, timeout=5)
            if msg:
                telemetry = {}
                if msg.get_type() == 'GLOBAL_POSITION_INT':
                    telemetry['lat'] = msg.lat / 1e7
                    telemetry['lon'] = msg.lon / 1e7
                    telemetry['alt'] = msg.alt / 1000
                    # Проверка диапазонов значений
                    assert - \
                        90.0 <= telemetry['lat'] <= 90.0, "Некорректная широта"
                    assert - \
                        180.0 <= telemetry['lon'] <= 180.0, "Некорректная долгота"
                elif msg.get_type() == 'ATTITUDE':
                    telemetry['roll'] = msg.roll
                    telemetry['pitch'] = msg.pitch
                    telemetry['yaw'] = msg.yaw
                    # Проверка диапазонов углов
                    assert - \
                        math.pi <= telemetry['roll'] <= math.pi, "Некорректный крен"
                    assert -math.pi / \
                        2 <= telemetry['pitch'] <= math.pi / \
                        2, "Некорректный тангаж"
                    assert - \
                        math.pi <= telemetry['yaw'] <= math.pi, "Некорректное рыскание"
                return telemetry
            logger.warning("Телеметрия недоступна")
            return None
        except AssertionError as e:
            logger.error("Ошибка в телеметрии: %s", e)
            return None
        except Exception as e:
            logger.error("Ошибка получения телеметрии: %s", e)
            return None

    def wait_command_ack(self, command: int, timeout: int = 10) -> bool:
        """
        Ожидание подтверждения выполнения команды.

        Args:
            command (int): Код команды MAVLink.
            timeout (int): Время ожидания в секундах.

        Returns:
            bool: True, если команда подтверждена, False в противном случае.
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            ack_msg = self.master.recv_match(
                type='COMMAND_ACK', blocking=True, timeout=1)
            if ack_msg and ack_msg.command == command:
                if ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    logger.info("Команда %s подтверждена", command)
                    return True
                logger.error("Команда %s отклонена с кодом %s",
                             command, ack_msg.result)
                return False
        logger.error("Не получено подтверждение для команды %s", command)
        return False

    def goto(self, lat: float, lon: float, alt: float) -> None:
        """
        Команда на полёт к заданным координатам.

        Args:
            lat (float): Широта целевой точки.
            lon (float): Долгота целевой точки.
            alt (float): Высота целевой точки в метрах.
        """
        try:
            self.seq += 1  # Increment mission sequence number

            # Отправка количества миссий (1 пункт)
            self.master.mav.mission_count_send(
                self.master.target_system,
                self.master.target_component,
                1,  # Количество пунктов миссии
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION
            )
            time.sleep(1)  # Задержка для обработки

            self.master.mav.mission_item_send(
                self.master.target_system,
                self.master.target_component,
                self.seq,  # Последовательный номер миссии
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,  # current (0 - не текущая точка)
                1,  # autocontinue (1 - продолжать автоматически)
                0, 0, 0, 0,
                int(lat * 1e7),
                int(lon * 1e7),
                alt
            )

            if not self.wait_command_ack(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT):
                raise RuntimeError("Команда полёта к точке не подтверждена")

            logger.info("Летим к точке (%s, %s, %s)", lat, lon, alt)
        except Exception as e:
            logger.error("Ошибка при полёте к точке: %s", e)
            raise RuntimeError(f"Failed to go to waypoint: {e}") from e

    def land(self) -> None:
        """
    Команда на посадку БПЛА.
    """
        try:
            self.set_mode('LAND')
            if not self.wait_command_ack(mavutil.mavlink.MAV_CMD_NAV_LAND):
                raise RuntimeError("Команда посадки не подтверждена")
            logger.info("БПЛА выполняет посадку")
        except Exception as e:
            logger.error("Ошибка при посадке: %s", e)
            raise RuntimeError(f"Failed to land: {e}") from e
