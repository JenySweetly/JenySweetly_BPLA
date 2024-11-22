import numpy as np
import unittest

# Константы для параметров БЛА
MAX_PAYLOAD = 1500  # максимальная взлетная масса в граммах
MAX_FLIGHT_TIME = 30 * 60  # максимальное время полета в секундах
MAX_CONTROL_DISTANCE = 1000  # максимальная дальность управления в метрах
POSITIONING_ACCURACY = 1  # точность позиционирования в метрах


def data_acs(accel_data, weight, flight_time, control_distance):
    """
    Обработка данных с акселерометра с учетом параметров БЛА.

    Args:
        accel_data: Список значений ускорения по осям X, Y, Z (g).
        weight: Масса БЛА в граммах.
        flight_time: Время полета в секундах.
        control_distance: Дальность управления в метрах.

    Returns:
        Словарь с обработанными данными:
        - acceleration: Список значений ускорения (g).
        - inclination: Список углов наклона по осям X, Y (град).
    """

    if weight > MAX_PAYLOAD:
        raise ValueError("Превышена максимальная взлетная масса БЛА.")
    if flight_time > MAX_FLIGHT_TIME:
        raise ValueError("Превышено максимальное время полета.")
    if control_distance > MAX_CONTROL_DISTANCE:
        raise ValueError("Превышена максимальная дальность управления.")

    acceleration_x = np.mean(accel_data[0])
    acceleration_y = np.mean(accel_data[1])
    acceleration_z = np.mean(accel_data[2])

    inclination_x = np.arctan2(acceleration_x, np.sqrt(
        np.square(acceleration_y) + np.square(acceleration_z))) * 180 / np.pi
    inclination_y = np.arctan2(acceleration_y, np.sqrt(
        np.square(acceleration_x) + np.square(acceleration_z))) * 180 / np.pi

    return {
        "acceleration": [acceleration_x, acceleration_y, acceleration_z],
        "inclination": [inclination_x, inclination_y]
    }


if __name__ == "__main__":
    accel_data = [[2, 2, 2], [5, 5, 5], [8, 8, 8]]
    weight = 1200  # масса БЛА в граммах
    flight_time = 1600  # время полета в секундах
    control_distance = 800  # дальность управления в метрах

    try:
        data_acs_obj = data_acs(
            accel_data, weight, flight_time, control_distance)
        print("Обработка данных:")
        print(f"Ускорение: {data_acs_obj['acceleration']}")
        print(f"Углы наклона: {data_acs_obj['inclination']}")
    except ValueError as e:
        print(e)


class TestDataAcs(unittest.TestCase):
    def test_data_processing(self):
        accel_data = [[2, 2, 2], [5, 5, 5], [8, 8, 8]]
        weight = 1200  # допустимая масса БЛА
        flight_time = 1600  # допустимое время полета
        control_distance = 800  # допустимая дальность управления

        result = data_acs(accel_data, weight, flight_time, control_distance)

        self.assertAlmostEqual(result['acceleration'][0], 2.0)
        self.assertAlmostEqual(result['acceleration'][1], 5.0)
        self.assertAlmostEqual(result['acceleration'][2], 8.0)
        self.assertAlmostEqual(
            result['inclination'][0], 11.96946312460731, places=2)
        self.assertAlmostEqual(
            result['inclination'][1], 31.23006977557103, places=2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
