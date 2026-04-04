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

"""ПАРАМЕТРЫ ДЛЯ СКАНИРОВАНИЯ"""
SCAN_MAP = 0 # сканировать ли карту
AUTO_SCAN_Z = 1 # автоматическое измерение средней высоты автоматического сканирования
GRID_POINTS = 6
PROBE_ATTEMPS = 1
SCAN_Z = -23.0 # Высота сканирования по умолчанию
MIN_SCAN_Z = -28.0 # Минимально возможная высота автоматического поиска высоты сканирования
MAX_SCAN_Z = -1.0 # Максимально возможная высота автоматического поиска высоты сканирования
HOP_SCAN_Z = -1.0 # Подъем для сканирования карты

"""ПАРАМЕТРЫ ДЛЯ ОБРАБОТКИ"""
CORNER_POINT = (93.0, 285.0) # угол закрепления заготовки
OFFSET_PROBE = (60.0, 23.0, 5.05) # Сдвиг по координатам щупа датчика относительно фрезы гравера
OFFSET_EXTRUDER = (10.0, -22.2, -12.65) # Сдвиг сопла экструдера относительно фрезы
INSTRUMENT_HEIGH_POS = (263.0, 294.0, -18.0) # позиция датчика высоты инструмента
USE_BOUNDS = 0 # проихводится анализ минимальных и максимальных координат, производится сдвиг к минимальным
if not USE_BOUNDS:
    GCODE_WIDTH = 35.0      # X axis
    GCODE_HEIGHT = 43.0     # Y axis

"""ПАРАМЕТРЫ ФАЙЛОВ"""
FILENAME = "rdfe.gcode"
OUTPUT_FILE = "output_main.gcode"
MAP_FILE = f"map.txt" # нерабочий
MAP_FILE_2 = f"map_2.txt"

"""ТИП РАБОТЫ КОДА"""
DEFAULT = types_code_run.DEFAULT
DEBUG = types_code_run.DEBUG
RUN_TYPE = DEFAULT # DEFAULT
PRINT_MSG = 0
USE_EXTRUDER = 1 # используется экструдер вместо фрезера

start_code_time = time.time()
if 1 or PRINT_MSG:
    print_message = lambda inp: print(str(round(time.time() - start_code_time, 3)) + ":\t" + str(inp))
else:
    print_message = lambda inp: 1

AUTO_SCAN_Z = AUTO_SCAN_Z and SCAN_MAP

"""КОНЕЦ ПАРАМЕТРОВ"""


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

    def find_h_instrument(self, x: float, y: float, z: float, cmd="M329"):
        try:
            #self.send_command(f"G1 Z0 F{self.scan_feedrate}")
            #self.send_command(f"M400")
            self.send_command(f"G1 X{x:.4f} Y{y:.4f} F{self.scan_feedrate}")
            self.send_command(f"M400")
            self.send_command(f"G1 Z{z:.4f} F{self.scan_feedrate}")
            self.send_command(f"M400")
            if cmd == "M327":
                self.probe.down()

            response = self.send_command(cmd, timeout=15)
            if response and self.trigger_prefix in response:
                height_str = response.split(self.trigger_prefix)[1].strip()
                h_instrument = float(height_str)
                return round(h_instrument, 6)
            raise Exception
        except Exception as exc:
            print("Ошибка")
            return "err"


    def auto_find_delta_instrument(self): # автоматический поиск высоты инструмента
        try:
            self.probe.up()
            self.send_command(f"G1 Z0 F{self.scan_feedrate}")
            self.send_command(f"M400")

            if not USE_EXTRUDER:
                x, y, z = INSTRUMENT_HEIGH_POS
            else:
                x = INSTRUMENT_HEIGH_POS[0] + OFFSET_EXTRUDER[0]
                y = INSTRUMENT_HEIGH_POS[1] + OFFSET_EXTRUDER[1]
                z = INSTRUMENT_HEIGH_POS[2] + OFFSET_EXTRUDER[2]

            h_instr = self.find_h_instrument(x, y, z)
            if h_instr == "err":
                raise Exception

            x_p = INSTRUMENT_HEIGH_POS[0] - OFFSET_PROBE[0]
            y_p = INSTRUMENT_HEIGH_POS[1] - OFFSET_PROBE[1]
            z_p = INSTRUMENT_HEIGH_POS[2] + OFFSET_PROBE[2] + 3.0

            self.send_command(f"G1 Z0 F{self.scan_feedrate}")
            self.send_command(f"M400")

            h_probe = self.find_h_instrument(x_p, y_p, z_p, "M327")

            self.probe.up()
            self.send_command(f"G1 Z0 F{self.scan_feedrate}")
            self.send_command(f"M400")

            if h_probe == "err":
                raise Exception
            delta = round(h_probe - h_instr, 6)
            self.z_offset = delta
            return delta

        except Exception:
            print('err')
            return 'err'






    def read_data(self, filename):
        #  чтение gcode из файла
        with open(filename, 'r', encoding='utf-8') as f:
            lines = f.readlines()
        return lines

    def automotive_find_scan_z(self, x: float, y: float):
        '''
        автоматические нахождение высоты сканирования на средней точке у gcode
        '''
        cycles = 3
        heights = []
        range_cycles = round(abs(MAX_SCAN_Z - MIN_SCAN_Z), 2)
        delta_cycles = round(range_cycles/cycles, 2)
        for i in range(cycles):
            z = MAX_SCAN_Z - delta_cycles * i
            heights.append(z)

        # Перемещение в точку
        x = max(self.x_min_pos, x - self.offset_probe_coords[0])  # компенсация сдвига по оси X
        y = max(self.y_min_pos, y - self.offset_probe_coords[1])  # компенсация сдвига по оси Y

        self.send_command(f"G1 X{x:.4f} Y{y:.4f} F{self.scan_feedrate}")
        self.send_command(f"M400")
        self.send_command(f"G1 Z0 F{self.scan_feedrate}")
        self.send_command(f"M400")

        is_found_z = 0
        for z_scan in heights:
            self.send_command(f"G1 Z{z_scan:.4f} F{self.scan_feedrate}")
            self.send_command(f"M400")

            # Запуск измерения
            self.probe.down()
            response = self.probe.probe()
            if response and self.trigger_prefix in response:
                height_str = response.split(self.trigger_prefix)[1].strip()
                height = round(float(height_str) + HOP_SCAN_Z, 2)
                self.scan_z = height
                is_found_z = 1
                break

        if not is_found_z:
            #self.scan_z = self.scan_z
            return False

        return True

    def find_midpoint_gcode(self, lines):
        min_x, max_x, min_y, max_y = gcode_analysis.find_gcode_bounds_margins(lines)
        mid_x = round((max_x + min_x)/2, 3)
        mid_y = round((max_y + min_y)/2, 3)
        return (mid_x, mid_y)




    def get_height_at_point_with_retry(self, x: float, y: float) -> Optional[float]:
        """
        Измерение высоты с несколькими попытками
        """
        heights = []

        x = max(self.x_min_pos, x - self.offset_probe_coords[0]) # компенсация сдвига по оси X
        y = max(self.y_min_pos, y - self.offset_probe_coords[1])  # компенсация сдвига по оси Y

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
                    #height = float(height_str) - self.z_offset # высота            ------------------ НЕРАБОЧАЯ ХРЕЕНЬ
                    height = float(height_str) # высота
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

    def process_gcode_file(self, input_file, output_file):
        """
        Основная функция обработки G-code
        """
        if PRINT_MSG: print(f"\n{'=' * 60}")
        print(f"ОБРАБОТКА ФАЙЛА: {input_file}")
        if PRINT_MSG: print(f"{'=' * 60}")

        # Чтение кода
        lines = self.read_data(input_file)
        if USE_BOUNDS:
            lines = gcode_analysis.offset_to_zero_position(lines) # берем минимальные xy
            min_x, max_x, min_y, max_y = gcode_analysis.find_gcode_bounds_margins(lines)
        else:
            min_x, max_x, min_y, max_y = (0, GCODE_WIDTH, 0, GCODE_HEIGHT)



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

        if USE_BOUNDS:
            x_margin = gcode_analysis.MARGIN_X
            y_margin = gcode_analysis.MARGIN_Y
        else:
            x_margin = 0.0
            y_margin = 0.0


        zero_point = [zero_point[0] + x_margin, zero_point[1] + y_margin]


        lines = gcode_analysis.added_xy_shift(lines, zero_point)

        if USE_BOUNDS:
            min_x_scan, max_x_scan, min_y_scan, max_y_scan = gcode_analysis.find_gcode_bounds_margins(lines) # добавление полей
        else:
            #min_x_scan, max_x_scan, min_y_scan, max_y_scan = gcode_analysis.find_gcode_bounds(lines)  # без добавление полей
            min_x_scan, max_x_scan, min_y_scan, max_y_scan = zero_point[0], zero_point[0] + GCODE_WIDTH, zero_point[1], zero_point[1] + GCODE_HEIGHT

        if PRINT_MSG: print("min_x, max_x, min_y, max_y")
        if PRINT_MSG: print(min_x, max_x, min_y, max_y)
        if PRINT_MSG: print()


        # Сканирование поверхности
        if SCAN_MAP:
            if RUN_TYPE == DEFAULT:
                self.send_command(f"G28")
                #self.probe.up()

                if AUTO_SCAN_Z:
                    mid_x, mid_y = self.find_midpoint_gcode(lines)
                    if self.automotive_find_scan_z(mid_x, mid_y):
                        print("найден нулевой уровень сканирования")
                    else:
                        print('не найден нулевой уровень, используем по умолчанию')


                if not self.scan_pcb_surface_snake(min_x_scan, max_x_scan, min_y_scan, max_y_scan):
                    print("Ошибка: не удалось получить данные высот")
                    return
                self.probe.up()
                self.send_command(f"G1 Z0")

            # Экспорт карты высот
            #self.export_height_map(input_file)
            self.export_height_map_snake(MAP_FILE_2)

        #датчик высоты инструмента
        if self.auto_find_delta_instrument() == "err":
            if not USE_EXTRUDER:
                self.z_offset = OFFSET_PROBE[2]
            else:
                self.z_offset = OFFSET_PROBE[2] - OFFSET_EXTRUDER[2]
        print(self.z_offset)
        # Обработка G-code
        processed_lines = interpolator.process_gcode_2(lines, MAP_FILE_2)
        #print(processed_lines[:100])
        if not USE_EXTRUDER:
            shift_z = self.z_offset * -1 - 0.4
        else:
            shift_z = self.z_offset * -1 - 0.4 + 0.5
        processed_lines = gcode_analysis.added_z_offset(processed_lines, shift_z)
        #print(processed_lines[:100])

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