# %%
from pymavlink import mavutil
import time
import math
from typing import Optional, Dict, Any
import logging

# %%
# uav_control.py
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
            logger.error(f"Ошибка подключения: {e}")
            raise

    def arm(self) -> None:
        """
        Взведение (Arm) БПЛА для начала работы двигателей.
        """
        try:
            self.master.arducopter_arm()
            self.master.motors_armed_wait()
            logger.info("БПЛА взведён")
        except Exception as e:
            logger.error(f"Ошибка взведения БПЛА: {e}")
            raise

    def disarm(self) -> None:
        """
        Разоружение (Disarm) БПЛА для остановки двигателей.
        """
        try:
            self.master.arducopter_disarm()
            self.master.motors_disarmed_wait()
            logger.info("БПЛА разоружён")
        except Exception as e:
            logger.error(f"Ошибка разоружения БПЛА: {e}")
            raise

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

            # Получение текущих координат
            msg = self.master.recv_match(
                type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
            if msg:
                current_lat = msg.lat / 1e7
                current_lon = msg.lon / 1e7
            else:
                raise Exception(
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
                raise Exception("Команда взлёта не подтверждена")
            logger.info(f"Взлёт на высоту {altitude} метров")
        except Exception as e:
            logger.error(f"Ошибка взлёта: {e}")
            raise

    def set_mode(self, mode: str) -> None:
        """
        Установка режима полёта БПЛА.

        Args:
            mode (str): Название режима (например, 'GUIDED', 'LAND').
        """
        mode_mapping = self.master.mode_mapping()
        if not isinstance(mode_mapping, dict):
            logger.error("Ошибка: mode_mapping() не вернул словарь")
            raise Exception("Не удалось получить список режимов полёта")

        mode_id = mode_mapping.get(mode)
        if mode_id is None:
            raise ValueError(f"Неизвестный режим: {mode}")

        try:
            self.master.set_mode(mode_id)
            logger.info(f"Режим установлен: {mode}")
        except Exception as e:
            logger.error(f"Ошибка установки режима {mode}: {e}")
            raise

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
                    if not -90.0 <= telemetry['lat'] <= 90.0:
                        raise ValueError("Некорректная широта")
                    if not -180.0 <= telemetry['lon'] <= 180.0:
                        raise ValueError("Некорректная долгота")
                elif msg.get_type() == 'ATTITUDE':
                    telemetry['roll'] = msg.roll
                    telemetry['pitch'] = msg.pitch
                    telemetry['yaw'] = msg.yaw
                    if not -math.pi <= telemetry['roll'] <= math.pi:
                        raise ValueError("Некорректный крен")
                    if not -math.pi/2 <= telemetry['pitch'] <= math.pi/2:
                        raise ValueError("Некорректный тангаж")
                    if not -math.pi <= telemetry['yaw'] <= math.pi:
                        raise ValueError("Некорректное рыскание")
                return telemetry
            else:
                logger.warning("Телеметрия недоступна")
                return None
        except Exception as e:
            logger.error(f"Ошибка получения телеметрии: {e}")
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
                    logger.info(f"Команда {command} подтверждена")
                    return True
                else:
                    logger.error(
                        f"Команда {command} отклонена с кодом {ack_msg.result}")
                    return False
        logger.error(f"Не получено подтверждение для команды {command}")
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
                0,  # Последовательный номер миссии
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Исправленный фрейм
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,  # current
                1,  # autocontinue
                0, 0, 0, 0,
                lat, lon, alt
            )

            if not self.wait_command_ack(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT):
                raise Exception("Команда полёта к точке не подтверждена")

            logger.info(f"Летим к точке ({lat}, {lon}, {alt})")
        except Exception as e:
            logger.error(f"Ошибка при полёте к точке: {e}")
            raise
