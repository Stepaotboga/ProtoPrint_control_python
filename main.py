import serial
import re
import time
import os
from typing import List, Tuple, Dict, Optional
import numpy as np
from scipy import interpolate
import csv
import json
from datetime import datetime
import math

import gcode_analysis
import interpolator
import types_code_run

SERIAL_PORT = "COM6"
GRID_POINTS = 7
SCAN_Z = -23.0
CORNER_POINT = (107.0, 317.0)
OFFSET_PROBE = (59.0, 18.95, 6.05) # Сдвиг по координатам щупа датчика относительно фрезы гравера
INSTRUMENT_HEIGH_POS = (263.0, 294.0, -20.0) # позиция датчика высоты инструмента
PROBE_ATTEMPS = 2

FILENAME = "denis.gcode"
OUTPUT_FILE = "output_main.gcode"
MAP_FILE = f"map.txt"
MAP_FILE_2 = f"map_2.txt"
PRINT_MSG = 0

DEFAULT = types_code_run.DEFAULT
DEBUG = types_code_run.DEBUG
RUN_TYPE = DEFAULT # DEFAULT


start_code_time = time.time()
if 1 or PRINT_MSG:
    print_message = lambda inp: print(str(round(time.time() - start_code_time, 3)) + ":\t" + str(inp))
else:
    print_message = lambda inp: 1


class Probe:
    def __init__(self, other, cur_pos="undefined"):
        self.deploy_command = "M42 P26 S0"
        self.retract_command = "M42 P26 S255"
        self.deploy_wait = 0.5 # задержка после опускания щупа
        self.current_position = cur_pos # текущее состояние щупа (втянуто/вытянуто)
        self.mapper = other

    def probe(self):
        response = self.mapper.send_command("M327", timeout=15)
        return response

    def down(self):
        if self.current_position != "down":
            self.mapper.send_command(self.deploy_command)
            self.current_position = "down"
            time.sleep(self.deploy_wait)
        return 1

    def up(self):
        if self.current_position != "up":
            self.mapper.send_command(self.retract_command)
            self.current_position = "up"
            print_message(self.retract_command)
            time.sleep(self.deploy_wait)
        return 1


class PCBHeightMapper:
    def __init__(self, port='COM9', baudrate=115200):
        """
        Инициализация подключения к станку
        """
        self.probe = None
        self.port = port
        self.baudrate = baudrate
        self.serial = None

        #параметры станка
        self.x_min_pos = 0.0
        self.y_min_pos = 0.0
        self.x_max_pos = 295.0
        self.y_max_pos = 350.0

        # Параметры сканирования
        self.scan_z = SCAN_Z
        self.scan_feedrate = 1000
        self.probe_feedrate = 300
        self.grid_points = GRID_POINTS
        self.trigger_prefix = "Triggered: "

        # Параметры компенсации
        self.probe_attempts = PROBE_ATTEMPS  # Количество попыток измерения высоты
        self.probe_retry_delay = 0.1  # Задержка между попытками (сек)
        self.offset_probe_coords = OFFSET_PROBE[:2] # Сдвиг по координатам щупа датчика относительно фрезы гравера
        self.z_offset = OFFSET_PROBE[2] # Сдвиг по высоте. Чем больше значение, тем глубже заглубляется фреза

        #рабочая область
        self.corner_position = CORNER_POINT # позиция угла зажима печатной платы
        self.corner_x = self.corner_position[0]
        self.corner_y = self.corner_position[1]
        self.x_border = 1.0
        self.y_border = 1.0

        # Результаты измерений
        self.height_map = {}  # (x, y) -> height
        self.height_map_3d = []  # Для экспорта
        self.interpolator = None



        if RUN_TYPE == DEFAULT:
            self.connect()
            self.probe = Probe(self)
            self.probe.up() # поднимаем щуп вверх              -убрать после debug

    def connect(self):
        """Подключение к станку"""
        print_message('connecting...')
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=2,
                write_timeout=2
            )
            time.sleep(2)
            self.serial.reset_input_buffer()
            print_message(f"Подключено к {self.port}")
        except Exception as e:
            print_message(f"Ошибка подключения: {e}")
            raise

    def send_command(self, command: str, wait_for_response=True, timeout=10, expected_response=None):
        """
        Отправка команды станку
        """
        if not self.serial:
            raise ConnectionError("Нет подключения к станку")

        # Очистка буфера
        self.serial.reset_input_buffer()

        # Отправка команды
        cmd = command.strip() + '\n'
        self.serial.write(cmd.encode())
        print_message(f"Отправка: {command}")

        if not wait_for_response:
            time.sleep(0.1)
            return None

        # Чтение ответа
        response_lines = []
        start_time = time.time()
        time_processing = start_time
        got_expected = False

        while time.time() - time_processing < timeout:
            if self.serial.in_waiting:
                try:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        print_message(f"  Ответ: {line}")
                        response_lines.append(line)

                        # Проверяем на триггер
                        if self.trigger_prefix in line:
                            return line

                        # Проверяем ожидаемый ответ
                        if expected_response and expected_response in line:
                            got_expected = True
                            return line

                        # проверка стандартного ответа станка
                        if line == "echo:busy: processing":
                            time_processing = time.time()
                            continue

                        # Проверяем на ошибку
                        if 'error' in line.lower() or 'alarm' in line.lower():
                            print_message(f"  ОШИБКА СТАНКА: {line}")
                            return line

                        # OK ответ
                        if 'ok' in line.lower():
                            return '\n'.join(response_lines)
                except Exception as e:
                    print_message(f"  Ошибка чтения: {e}")
                    continue
            time.sleep(0.01)

        # Таймаут
        if response_lines:
            print("ТАЙМАУТ!!!!")
            return '\n'.join(response_lines)
        else:
            print_message("  Таймаут ожидания ответа")
            return None


    def read_data(self, filename):
        #  чтение gcode из файла
        with open(filename, 'r', encoding='utf-8') as f:
            lines = f.readlines()
        return lines


    def get_height_at_point_with_retry(self, x: float, y: float) -> Optional[float]:
        """
        Измерение высоты с несколькими попытками
        """
        heights = []

        x = max(self.x_min_pos, x - self.offset_probe_coords[0]) # компенсация сдвига по оси X
        y = max(self.y_min_pos, y - self.offset_probe_coords[1])  # компенсация сдвига по оси X

        for attempt in range(self.probe_attempts):
            try:
                print_message(f"  Попытка {attempt + 1}/{self.probe_attempts}")

                z = self.scan_z + self.z_offset # отход на высоту + сдвиг

                # Перемещение в точку
                self.send_command(f"G1 X{x:.4f} Y{y:.4f} F{self.scan_feedrate}")
                self.send_command(f"M400")
                self.send_command(f"G1 Z{z:.4f} F{self.scan_feedrate}")
                self.send_command(f"M400")

                # Запуск измерения
                self.probe.down()
                response = self.probe.probe()

                if response and self.trigger_prefix in response:
                    height_str = response.split(self.trigger_prefix)[1].strip()
                    height = float(height_str) - self.z_offset # высота
                    heights.append(height)
                    print_message(f"    Измерено: {height:.4f} мм")

                    # Если получили разумное значение, проверяем
                    if -50 < height < -30:  # Разумный диапазон для платы
                        if len(heights) > 1:
                            # Проверяем стабильность измерений
                            if abs(height - heights[-2]) < 0.1:  # Разница менее 0.1 мм
                                print_message(f"    Измерение стабильно")
                                return height
                        elif len(heights) == self.probe_attempts:
                            # Берем медиану всех попыток
                            heights_sorted = sorted(heights)
                            median_height = heights_sorted[len(heights_sorted) // 2]
                            print_message(f"    Используем медиану: {median_height:.4f} мм")
                            return median_height

                # Задержка между попытками
                if attempt < self.probe_attempts - 1:
                    time.sleep(self.probe_retry_delay)

            except Exception as e:
                print_message(f"    Ошибка попытки {attempt + 1}: {e}")
                if attempt < self.probe_attempts - 1:
                    time.sleep(self.probe_retry_delay)
                continue

        # Если все попытки неудачны
        if heights:
            avg_height = sum(heights) / len(heights)
            print_message(f"    Используем среднее после ошибок: {avg_height:.4f} мм")
            return avg_height

        print_message(f"    ВСЕ ПОПЫТКИ НЕУДАЧНЫ!")
        return None

    def scan_pcb_surface(self, min_x: float, max_x: float, min_y: float, max_y: float): #старый алгоритм сканирования
        """
        Сканирование поверхности платы
        """
        global total_points
        if PRINT_MSG: print(f"\n{'=' * 60}")
        if PRINT_MSG: print("НАЧАЛО СКАНИРОВАНИЯ ПОВЕРХНОСТИ ПЛАТЫ")
        if PRINT_MSG: print(f"Диапазон X: {min_x:.2f} - {max_x:.2f} мм")
        if PRINT_MSG: print(f"Диапазон Y: {min_y:.2f} - {max_y:.2f} мм")
        if PRINT_MSG: print(f"Сетка: {self.grid_points}×{self.grid_points} точек")
        if PRINT_MSG: print(f"{'=' * 60}")

        # Генерируем сетку
        x_points = np.linspace(min_x, max_x, self.grid_points)
        y_points = np.linspace(min_y, max_y, self.grid_points)

        self.height_map = {}
        self.height_map_3d = []
        self.grid = np.zeros((len(x_points),len(y_points)))

        successful_points = 0

        for i, x in enumerate(x_points):
            if PRINT_MSG: print(f"\nСтрока {i + 1}/{self.grid_points}:")

            for j, y in enumerate(y_points):
                point_num = i * self.grid_points + j + 1
                total_points = self.grid_points ** 2

                if PRINT_MSG: print(f"  Точка {point_num}/{total_points}: X={x:.2f}, Y={y:.2f}")

                height = self.get_height_at_point_with_retry(x, y)

                if height is not None:
                    self.height_map[(round(x, 4), round(y, 4))] = height
                    self.height_map_3d.append({
                        'x': round(x, 4),
                        'y': round(y, 4),
                        'z': round(height, 4)
                    })
                    self.grid[i, j] = round(height, 4)
                    print(self.grid)
                    successful_points += 1
                    if PRINT_MSG: print(f"    ✓ Сохранено: {height:.4f} мм")
                else:
                    if PRINT_MSG: print(f"    ✗ Пропуск точки")

                # Небольшая пауза между точками
                time.sleep(0.2)

        if PRINT_MSG: print(f"\n{'=' * 60}")
        print(f"СКАНИРОВАНИЕ ЗАВЕРШЕНО")
        if PRINT_MSG: print(f"Успешных измерений: {successful_points}/{total_points}")
        if PRINT_MSG: print(f"{'=' * 60}")

        return successful_points > 0

    def scan_pcb_surface_snake(self, min_x: float, max_x: float, min_y: float, max_y: float): # сканирование карты змейкой
        """
        Сканирование поверхности платы
        """'''
        global total_points
        if PRINT_MSG: print(f"\n{'=' * 60}")
        if PRINT_MSG: print("НАЧАЛО СКАНИРОВАНИЯ ПОВЕРХНОСТИ ПЛАТЫ")
        if PRINT_MSG: print(f"Диапазон X: {min_x:.2f} - {max_x:.2f} мм")
        if PRINT_MSG: print(f"Диапазон Y: {min_y:.2f} - {max_y:.2f} мм")
        if PRINT_MSG: print(f"Сетка: {self.grid_points}×{self.grid_points} точек")
        if PRINT_MSG: print(f"{'=' * 60}")'''

        # Генерируем сетку
        x_points = np.linspace(min_x, max_x, self.grid_points)
        y_points = np.linspace(min_y, max_y, self.grid_points)
        self.xp = x_points
        self.yp = y_points
        self.grid = np.zeros((len(x_points),len(y_points)))

        successful_points = 0

        for j, y in enumerate(y_points):
            if j % 2 == 0:
                x_iter = range(len(x_points))
            else:
                x_iter = reversed(range(len(x_points)))

            for i in x_iter:
                x = x_points[i]

                height = self.get_height_at_point_with_retry(x, y)

                if height is not None:
                    self.grid[j, i] = round(height, 4)
                    print(self.grid)
                    successful_points += 1

                else:
                    if PRINT_MSG: print(f"    ✗ Пропуск точки")

                # Небольшая пауза между точками
                time.sleep(0.2)

        print(f"СКАНИРОВАНИЕ ЗАВЕРШЕНО")

        return successful_points > 0

    def export_height_map_snake(self, filename=MAP_FILE):
        """
        Экспорт карты высот в файлы
        """
        if not self.grid.all():
            print("Нет данных для экспорта")
            return


        txt_file = filename
        with open(txt_file, 'w', encoding='utf-8') as f:

            for j, y in enumerate(self.yp):
                for i, x in enumerate(self.xp):
                    f.write(f"{x} {y} {self.grid[i, j]}\n")

        print(f"\nКарта высот экспортирована:")
        if PRINT_MSG: print(f"  TXT: {txt_file}")


    def export_height_map(self, filename: str):
        """
        Экспорт карты высот в файлы
        """
        if not self.height_map_3d:
            print("Нет данных для экспорта")
            return

        base_name = os.path.splitext(filename)[0]
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        # Текстовый файл для визуализации
        txt_file = MAP_FILE
        with open(txt_file, 'w', encoding='utf-8') as f:
            f.write("=" * 60 + "\n")
            f.write(f"КАРТА ВЫСОТ ПЛАТЫ\n")
            f.write(f"Время: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Всего точек: {len(self.height_map_3d)}\n")
            f.write("=" * 60 + "\n\n")

            # Группируем по Y для удобного отображения
            y_groups = {}
            for point in self.height_map_3d:
                y = point['y']
                if y not in y_groups:
                    y_groups[y] = []
                y_groups[y].append(point)

            # Сортируем Y
            sorted_y = sorted(y_groups.keys())

            for y in sorted_y:
                f.write(f"\nY = {y:.2f} мм:\n")
                points = sorted(y_groups[y], key=lambda p: p['x'])

                for point in points:
                    f.write(f"  X={point['x']:6.2f} мм: Z={point['z']:7.4f} мм\n")

        print(f"\nКарта высот экспортирована:")
        if PRINT_MSG: print(f"  TXT: {txt_file}")

    def process_gcode_file(self, input_file, output_file):
        """
        Основная функция обработки G-code
        """
        if PRINT_MSG: print(f"\n{'=' * 60}")
        print(f"ОБРАБОТКА ФАЙЛА: {input_file}")
        if PRINT_MSG: print(f"{'=' * 60}")

        # Чтение кода
        lines = self.read_data(input_file)
        lines = gcode_analysis.offset_zero_position(lines) # берем минимальные xy

        min_x, max_x, min_y, max_y = gcode_analysis.find_gcode_bounds_margins(lines)

        # Поиск границ для снятия карты

        if PRINT_MSG: print("min_x, max_x, min_y, max_y")
        if PRINT_MSG: print(min_x, max_x, min_y, max_y)
        if PRINT_MSG: print()

        x_range = max_x - min_x
        y_range = max_y - min_y

        #print(lines)

        zero_point = [self.corner_x, self.corner_y - y_range]
        if PRINT_MSG: print("zero_point", zero_point)

        if PRINT_MSG: print()

        x_margin = 2.5
        y_margin = 2.5

        zero_point = [zero_point[0] + x_margin, zero_point[1] + y_margin]

        lines = gcode_analysis.added_xy_shift(lines, zero_point)

        min_x_scan, max_x_scan, min_y_scan, max_y_scan = gcode_analysis.find_gcode_bounds_margins(lines) # добавление полей

        if PRINT_MSG: print("min_x, max_x, min_y, max_y")
        if PRINT_MSG: print(min_x, max_x, min_y, max_y)
        if PRINT_MSG: print()


        # Сканирование поверхности
        if RUN_TYPE == DEFAULT:
            self.send_command(f"G28")
            #self.probe.up()
            if not self.scan_pcb_surface_snake(min_x_scan, max_x_scan, min_y_scan, max_y_scan):
                print("Ошибка: не удалось получить данные высот")
                return
            self.probe.up()
            self.send_command(f"G1 Z0")

        # Экспорт карты высот
        #self.export_height_map(input_file)
        self.export_height_map_snake(MAP_FILE_2)


        # Обработка G-code
        processed_lines = interpolator.process_gcode_2(lines, MAP_FILE_2)

        if PRINT_MSG: print(f"\n{'=' * 60}")
        print(f"ОБРАБОТКА ЗАВЕРШЕНА")






        # Сохраняем результат
        with open(output_file, 'w', encoding='utf-8') as f:
            f.writelines(processed_lines)

        print(f"\nКомпенсация завершена:")









def main():
    mapper = PCBHeightMapper(port=SERIAL_PORT, baudrate=115200)
    mapper.grid_points = GRID_POINTS
    mapper.probe_attempts = PROBE_ATTEMPS

    mapper.process_gcode_file(FILENAME, OUTPUT_FILE)



if __name__ == "__main__":
    main()