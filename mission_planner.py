# %%
# mission_planner.py

from uav_control import UAVControl
import time
from typing import List, Tuple
import logging

logger = logging.getLogger(__name__)


# %%


class MissionPlanner:
    """
    Класс для планирования и выполнения миссий БПЛА.
    """

    def __init__(self, connection_string: str):
        """
        Инициализация планировщика миссий.

        Args:
            connection_string (str): Строка подключения MAVLink.
        """
        self.uav = UAVControl(connection_string)

    def execute_mission(self, waypoints: List[Tuple[float, float, float]]) -> None:
        """
        Выполнение миссии по заданным точкам.

        Args:
            waypoints (List[Tuple[float, float, float]]): Список точек (lat, lon, alt).
        """
        try:
            self.uav.arm()
            self.uav.set_mode('GUIDED')
            self.uav.takeoff(waypoints[0][2])

            # Ожидание набора высоты
            time.sleep(5)

            for idx, waypoint in enumerate(waypoints):
                logger.info(f"Переходим к точке {idx+1}: {waypoint}")
                self.uav.goto(*waypoint)

                # Ожидание достижения точки с проверкой телеметрии
                reached = False
                for _ in range(5):  # Максимум 5 проверок
                    telemetry = self.uav.get_telemetry()
                    if telemetry:
                        lat_diff = abs(telemetry.get('lat', 0.0) - waypoint[0])
                        lon_diff = abs(telemetry.get('lon', 0.0) - waypoint[1])
                        alt_diff = abs(telemetry.get('alt', 0.0) - waypoint[2])
                        if lat_diff < 0.0001 and lon_diff < 0.0001 and alt_diff < 1.0:
                            reached = True
                            logger.info(f"Достигнута точка {idx+1}")
                            break
                    time.sleep(1)
                if not reached:
                    logger.error(f"Не удалось достичь точки {idx+1}")
                    raise Exception(f"Не удалось достичь точки {idx+1}")

            # Возвращение и посадка
            self.uav.set_mode('RTL')
            logger.info("Возвращение домой и посадка")

            # Ожидание посадки
            time.sleep(5)
            self.uav.disarm()
        except Exception as e:
            logger.error(f"Ошибка во время выполнения миссии: {e}")
            self.uav.disarm()
            raise
