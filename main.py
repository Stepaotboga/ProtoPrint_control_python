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



class Probe:
    def __init__(self, other, cur_pos="undefined"):
        self.deploy_command = "M42 P26 S0"
        self.retract_command = "M42 P26 S1"
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
            time.sleep(self.deploy_wait)
        return 1





class PCBHeightMapper:
    def __init__(self, port='COM9', baudrate=115200):
        """
        Инициализация подключения к станку
        """
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.connect()
        self.probe = Probe(self)
        self.probe.up()

        # Параметры сканирования
        self.scan_z = -40.0
        self.scan_feedrate = 800
        self.probe_feedrate = 100
        self.grid_points = 10
        self.trigger_prefix = "Triggered: "

        # Параметры компенсации
        self.max_segment_length = 5.0  # Максимальная длина сегмента для интерполяции (мм)
        self.probe_attempts = 3  # Количество попыток измерения высоты
        self.probe_retry_delay = 0.5  # Задержка между попытками (сек)
        self.offset_probe_coords = (59.0, 18.95) # Сдвиг по координатам щупа датчика относительно фрезы гравера

        #рабочая область
        self.corner_position = (111.0, 289.0) # позиция угла зажима печатной платы
        self.corner_x = self.corner_position[0]
        self.corner_y = self.corner_position[1]



        # Результаты измерений
        self.height_map = {}  # (x, y) -> height
        self.height_map_3d = []  # Для экспорта
        self.interpolator = None

    def connect(self):
        """Подключение к станку"""
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
            print(f"Подключено к {self.port}")
        except Exception as e:
            print(f"Ошибка подключения: {e}")
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
        print(f"Отправка: {command}")

        if not wait_for_response:
            time.sleep(0.1)
            return None

        # Чтение ответа
        response_lines = []
        start_time = time.time()
        got_expected = False

        while time.time() - start_time < timeout:
            if self.serial.in_waiting:
                try:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        print(f"  Ответ: {line}")
                        response_lines.append(line)

                        # Проверяем на триггер
                        if self.trigger_prefix in line:
                            return line

                        # Проверяем ожидаемый ответ
                        if expected_response and expected_response in line:
                            got_expected = True
                            return line

                        # Проверяем на ошибку
                        if 'error' in line.lower() or 'alarm' in line.lower():
                            print(f"  ОШИБКА СТАНКА: {line}")
                            return line

                        # OK ответ
                        if 'ok' in line.lower():
                            return '\n'.join(response_lines)
                except Exception as e:
                    print(f"  Ошибка чтения: {e}")
                    continue
            time.sleep(0.01)

        # Таймаут
        if response_lines:
            return '\n'.join(response_lines)
        else:
            print("  Таймаут ожидания ответа")
            return None

    def get_height_at_point_with_retry(self, x: float, y: float) -> Optional[float]:
        """
        Измерение высоты с несколькими попытками
        """
        heights = []

        for attempt in range(self.probe_attempts):
            try:
                print(f"  Попытка {attempt + 1}/{self.probe_attempts}")

                # Перемещение в точку
                self.send_command(f"G1 Z{self.scan_z:.4f} F{self.scan_feedrate}")
                self.send_command(f"G1 X{x:.4f} Y{y:.4f} F{self.scan_feedrate}")

                # Запуск измерения
                response = self.send_command("M327", timeout=15)

                if response and self.trigger_prefix in response:
                    height_str = response.split(self.trigger_prefix)[1].strip()
                    height = float(height_str)
                    heights.append(height)
                    print(f"    Измерено: {height:.4f} мм")

                    # Если получили разумное значение, проверяем
                    if -50 < height < -30:  # Разумный диапазон для платы
                        if len(heights) > 1:
                            # Проверяем стабильность измерений
                            if abs(height - heights[-2]) < 0.1:  # Разница менее 0.1 мм
                                print(f"    Измерение стабильно")
                                return height
                        elif len(heights) == self.probe_attempts:
                            # Берем медиану всех попыток
                            heights_sorted = sorted(heights)
                            median_height = heights_sorted[len(heights_sorted) // 2]
                            print(f"    Используем медиану: {median_height:.4f} мм")
                            return median_height

                # Задержка между попытками
                if attempt < self.probe_attempts - 1:
                    time.sleep(self.probe_retry_delay)

            except Exception as e:
                print(f"    Ошибка попытки {attempt + 1}: {e}")
                if attempt < self.probe_attempts - 1:
                    time.sleep(self.probe_retry_delay)
                continue

        # Если все попытки неудачны
        if heights:
            avg_height = sum(heights) / len(heights)
            print(f"    Используем среднее после ошибок: {avg_height:.4f} мм")
            return avg_height

        print(f"    ВСЕ ПОПЫТКИ НЕУДАЧНЫ!")
        return None

    def scan_pcb_surface(self, min_x: float, max_x: float, min_y: float, max_y: float):
        """
        Сканирование поверхности платы
        """
        print(f"\n{'=' * 60}")
        print("НАЧАЛО СКАНИРОВАНИЯ ПОВЕРХНОСТИ ПЛАТЫ")
        print(f"Диапазон X: {min_x:.2f} - {max_x:.2f} мм")
        print(f"Диапазон Y: {min_y:.2f} - {max_y:.2f} мм")
        print(f"Сетка: {self.grid_points}×{self.grid_points} точек")
        print(f"{'=' * 60}")

        # Генерируем сетку
        x_points = np.linspace(min_x, max_x, self.grid_points)
        y_points = np.linspace(min_y, max_y, self.grid_points)

        self.height_map = {}
        self.height_map_3d = []

        successful_points = 0

        for i, x in enumerate(x_points):
            print(f"\nСтрока {i + 1}/{self.grid_points}:")

            for j, y in enumerate(y_points):
                point_num = i * self.grid_points + j + 1
                total_points = self.grid_points ** 2

                print(f"  Точка {point_num}/{total_points}: X={x:.2f}, Y={y:.2f}")

                height = self.get_height_at_point_with_retry(x, y)

                if height is not None:
                    self.height_map[(round(x, 4), round(y, 4))] = height
                    self.height_map_3d.append({
                        'x': round(x, 4),
                        'y': round(y, 4),
                        'z': round(height, 4)
                    })
                    successful_points += 1
                    print(f"    ✓ Сохранено: {height:.4f} мм")
                else:
                    print(f"    ✗ Пропуск точки")

                # Небольшая пауза между точками
                time.sleep(0.2)

        print(f"\n{'=' * 60}")
        print(f"СКАНИРОВАНИЕ ЗАВЕРШЕНО")
        print(f"Успешных измерений: {successful_points}/{total_points}")
        print(f"{'=' * 60}")

        return successful_points > 0

    def create_height_interpolator(self):
        """
        Создание интерполятора для оценки высоты
        """
        if len(self.height_map) < 4:
            raise ValueError(f"Недостаточно точек для интерполяции: {len(self.height_map)}")

        # Подготовка данных
        points = []
        values = []

        for (x, y), z in self.height_map.items():
            points.append([x, y])
            values.append(z)

        points = np.array(points)
        values = np.array(values)

        # Создаем интерполятор (кубическая интерполяция)
        try:
            self.interpolator = interpolate.CloughTocher2DInterpolator(points, values, fill_value=np.mean(values))
            print(f"Создан интерполятор на основе {len(points)} точек")
        except Exception as e:
            print(f"Ошибка создания интерполятора: {e}, используем линейную интерполяцию")
            self.interpolator = interpolate.LinearNDInterpolator(points, values, fill_value=np.mean(values))

    def estimate_height(self, x: float, y: float) -> float:
        """
        Оценка высоты в точке
        """
        if self.interpolator is None:
            self.create_height_interpolator()

        try:
            height = float(self.interpolator(x, y))
            return height
        except Exception as e:
            # Если интерполяция не удалась, используем ближайшую точку
            print(f"Ошибка интерполяции в ({x:.2f}, {y:.2f}): {e}")

            # Находим ближайшую измеренную точку
            min_dist = float('inf')
            nearest_height = 0

            for (px, py), h in self.height_map.items():
                dist = math.sqrt((x - px) ** 2 + (y - py) ** 2)
                if dist < min_dist:
                    min_dist = dist
                    nearest_height = h

            print(f"  Используем ближайшую точку: {nearest_height:.4f} мм (расстояние: {min_dist:.2f} мм)")
            return nearest_height

    def interpolate_movement(self, x1: float, y1: float, x2: float, y2: float,
                             start_z: float, end_z: float) -> List[Tuple[float, float, float]]:
        """
        Разбивает движение на сегменты с интерполяцией высоты
        """
        segments = []

        # Вычисляем длину движения
        distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

        if distance <= self.max_segment_length:
            # Не разбиваем, просто используем интерполированные высоты
            mid_z = self.estimate_height((x1 + x2) / 2, (y1 + y2) / 2)
            segments.append((x1, y1, start_z))
            segments.append((x2, y2, end_z))
        else:
            # Разбиваем на сегменты
            num_segments = max(2, int(math.ceil(distance / self.max_segment_length)))

            for i in range(num_segments + 1):
                t = i / num_segments
                x = x1 + (x2 - x1) * t
                y = y1 + (y2 - y1) * t

                if i == 0:
                    z = start_z
                elif i == num_segments:
                    z = end_z
                else:
                    # Интерполируем высоту для промежуточной точки
                    z = self.estimate_height(x, y)

                segments.append((x, y, z))

        return segments

    def export_height_map(self, filename: str):
        """
        Экспорт карты высот в файлы
        """
        if not self.height_map_3d:
            print("Нет данных для экспорта")
            return

        base_name = os.path.splitext(filename)[0]
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        # CSV файл
        csv_file = f"{base_name}_height_map_{timestamp}.csv"
        with open(csv_file, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(['X (mm)', 'Y (mm)', 'Height (mm)', 'Timestamp'])

            for point in self.height_map_3d:
                writer.writerow([
                    point['x'],
                    point['y'],
                    point['z'],
                    datetime.now().isoformat()
                ])

        # JSON файл
        json_file = f"{base_name}_height_map_{timestamp}.json"
        with open(json_file, 'w', encoding='utf-8') as f:
            json.dump({
                'metadata': {
                    'timestamp': timestamp,
                    'points_count': len(self.height_map_3d),
                    'grid_points': self.grid_points,
                    'scan_z': self.scan_z
                },
                'height_map': self.height_map_3d
            }, f, indent=2)

        # Текстовый файл для визуализации
        txt_file = f"{base_name}_height_map_{timestamp}.txt"
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
        print(f"  CSV: {csv_file}")
        print(f"  JSON: {json_file}")
        print(f"  TXT: {txt_file}")

        # Статистика
        heights = [p['z'] for p in self.height_map_3d]
        print(f"\nСтатистика высот:")
        print(f"  Минимум: {min(heights):.4f} мм")
        print(f"  Максимум: {max(heights):.4f} мм")
        print(f"  Среднее: {np.mean(heights):.4f} мм")
        print(f"  Стандартное отклонение: {np.std(heights):.4f} мм")

    def process_gcode_file(self, input_file: str, output_file: str = None):
        """
        Основная функция обработки G-code
        """
        print(f"\n{'=' * 60}")
        print(f"ОБРАБОТКА ФАЙЛА: {input_file}")
        print(f"{'=' * 60}")

        # Поиск границ
        min_x, max_x, min_y, max_y = self.find_gcode_bounds(input_file)

        # Сканирование поверхности
        if not self.scan_pcb_surface(min_x, max_x, min_y, max_y):
            print("Ошибка: не удалось получить данные высот")
            return

        # Экспорт карты высот
        self.export_height_map(input_file)

        # Создание интерполятора
        self.create_height_interpolator()

        # Создание выходного файла
        if output_file is None:
            base, ext = os.path.splitext(input_file)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_file = f"{base}_compensated_{timestamp}{ext}"

        # Обработка G-code
        self.compensate_gcode_with_interpolation(input_file, output_file)

        print(f"\n{'=' * 60}")
        print(f"ОБРАБОТКА ЗАВЕРШЕНА")
        print(f"Входной файл: {input_file}")
        print(f"Выходной файл: {output_file}")
        print(f"{'=' * 60}")

    def find_gcode_bounds(self, filename: str) -> Tuple[float, float, float, float]:
        """
        Находит границы обработки в G-code
        """
        print(f"\nАнализ G-code файла...")

        min_x = float('inf')
        max_x = float('-inf')
        min_y = float('inf')
        max_y = float('-inf')

        patterns = {
            'x': re.compile(r'X([-+]?\d*\.?\d+)'),
            'y': re.compile(r'Y([-+]?\d*\.?\d+)'),
            'z': re.compile(r'Z([-+]?\d*\.?\d+)')
        }

        with open(filename, 'r', encoding='utf-8') as f:
            lines = f.readlines()

        for line_num, line in enumerate(lines, 1):
            # Пропускаем комментарии
            clean_line = line.split(';')[0].split('(')[0].strip()
            if not clean_line:
                continue

            # Ищем координаты
            x_match = patterns['x'].search(clean_line)
            y_match = patterns['y'].search(clean_line)

            if x_match:
                x_val = float(x_match.group(1))
                min_x = min(min_x, x_val)
                max_x = max(max_x, x_val)

            if y_match:
                y_val = float(y_match.group(1))
                min_y = min(min_y, y_val)
                max_y = max(max_y, y_val)

        # Если не нашли координаты
        if min_x == float('inf'):
            min_x, max_x = 0, 100
            print("  Координаты X не найдены, используем 0-100 мм")
        else:
            print(f"  X: {min_x:.2f} - {max_x:.2f} мм")

        if min_y == float('inf'):
            min_y, max_y = 0, 100
            print("  Координаты Y не найдены, используем 0-100 мм")
        else:
            print(f"  Y: {min_y:.2f} - {max_y:.2f} мм")

        # Добавляем поля
        margin_x = (max_x - min_x) * 0.1 + 5
        margin_y = (max_y - min_y) * 0.1 + 5

        min_x -= margin_x
        max_x += margin_x
        min_y -= margin_y
        max_y += margin_y

        print(f"  Скан с полями: X={min_x:.2f}-{max_x:.2f}, Y={min_y:.2f}-{max_y:.2f}")

        return min_x, max_x, min_y, max_y

    def compensate_gcode_with_interpolation(self, input_file: str, output_file: str):
        """
        Обработка G-code с интерполяцией высоты по траектории
        """
        print(f"\nКомпенсация высоты в G-code...")

        patterns = {
            'x': re.compile(r'X([-+]?\d*\.?\d+)'),
            'y': re.compile(r'Y([-+]?\d*\.?\d+)'),
            'z': re.compile(r'Z([-+]?\d*\.?\d+)'),
            'f': re.compile(r'F([-+]?\d*\.?\d+)'),
            'g': re.compile(r'G(\d+)'),
            'm': re.compile(r'M(\d+)')
        }

        with open(input_file, 'r', encoding='utf-8') as f:
            lines = f.readlines()

        processed_lines = []
        last_x = None
        last_y = None
        last_z = None
        feedrate = 1000  # Значение по умолчанию

        line_count = len(lines)
        compensated_moves = 0

        for line_num, line in enumerate(lines, 1):
            original_line = line.rstrip('\n')
            clean_line = original_line.split(';')[0].split('(')[0].strip()

            # Сохраняем комментарии и пустые строки
            if not clean_line:
                processed_lines.append(original_line + '\n')
                continue

            # Проверяем на команды, которые нужно оставить как есть
            if any(cmd in clean_line.upper() for cmd in ['M30', 'M2', 'M0', 'M1', 'M6', 'M3', 'M4', 'M5']):
                processed_lines.append(original_line + '\n')
                continue

            # Ищем G-код
            g_match = patterns['g'].search(clean_line)
            g_code = int(g_match.group(1)) if g_match else None

            # Ищем скорость
            f_match = patterns['f'].search(clean_line)
            if f_match:
                feedrate = float(f_match.group(1))

            # Если это не G0/G1, оставляем как есть
            if g_code not in [0, 1]:
                processed_lines.append(original_line + '\n')
                continue

            # Ищем координаты в текущей строке
            x_match = patterns['x'].search(clean_line)
            y_match = patterns['y'].search(clean_line)
            z_match = patterns['z'].search(clean_line)

            target_x = float(x_match.group(1)) if x_match else last_x
            target_y = float(y_match.group(1)) if y_match else last_y
            target_z = float(z_match.group(1)) if z_match else last_z

            # Если нет координат X/Y, оставляем как есть
            if target_x is None or target_y is None:
                processed_lines.append(original_line + '\n')
                if x_match:
                    last_x = target_x
                if y_match:
                    last_y = target_y
                if z_match:
                    last_z = target_z
                continue

            # Если нет предыдущих координат (начало)
            if last_x is None or last_y is None:
                last_x = target_x
                last_y = target_y
                last_z = target_z if target_z is not None else 0
                processed_lines.append(original_line + '\n')
                continue

            # Определяем целевую высоту Z
            if target_z is None:
                # Если Z не указан, используем последний
                target_z = last_z
            else:
                # Компенсируем целевую высоту
                surface_height = self.estimate_height(target_x, target_y)
                target_z = target_z - surface_height

            # Компенсируем начальную высоту
            start_surface_height = self.estimate_height(last_x, last_y)
            if last_z is not None:
                start_z = last_z - start_surface_height
            else:
                start_z = 0

            # Разбиваем движение на сегменты
            segments = self.interpolate_movement(
                last_x, last_y, target_x, target_y,
                start_z, target_z
            )

            # Генерируем G-код для сегментов
            for i, (seg_x, seg_y, seg_z) in enumerate(segments):
                if i == 0 and seg_x == last_x and seg_y == last_y and abs(seg_z - start_z) < 0.001:
                    # Пропускаем первую точку если она совпадает с текущей
                    continue

                # Формируем команду
                cmd_parts = [f"G1"]

                # Добавляем координаты с округлением до 4 знаков
                cmd_parts.append(f"X{seg_x:.4f}")
                cmd_parts.append(f"Y{seg_y:.4f}")
                cmd_parts.append(f"Z{seg_z:.4f}")

                # Добавляем скорость, если она была в оригинальной команде
                if f_match or i == 0:
                    cmd_parts.append(f"F{feedrate:.1f}")

                cmd = " ".join(cmd_parts)

                # Добавляем комментарий для отладки
                if i == 0 or i == len(segments) - 1:
                    comment = f" ; сегмент {i + 1}/{len(segments)}"
                    if i == len(segments) - 1:
                        comment += f" (конечная точка)"
                else:
                    comment = f" ; интерполяция {i + 1}/{len(segments)}"

                processed_lines.append(cmd + comment + '\n')
                compensated_moves += 1

            # Обновляем последние координаты
            last_x = target_x
            last_y = target_y
            last_z = target_z

            # Прогресс
            if line_num % 10 == 0 or line_num == line_count:
                print(f"  Обработано: {line_num}/{line_count} строк")

        # Сохраняем результат
        with open(output_file, 'w', encoding='utf-8') as f:
            f.writelines(processed_lines)

        print(f"\nКомпенсация завершена:")
        print(f"  Всего строк: {line_count}")
        print(f"  Скомпенсировано перемещений: {compensated_moves}")
        print(f"  Файл сохранен: {output_file}")

    def close(self):
        """Закрытие соединения"""
        if self.serial:
            try:
                # Поднимаем фрезу перед завершением
                self.send_command("G1 Z5 F500", wait_for_response=False)
                time.sleep(1)
                self.serial.close()
                print("\nСоединение закрыто, фреза поднята")
            except:
                self.serial.close()
                print("\nСоединение закрыто")


def main():
    """
    Основная функция
    """
    import argparse

    parser = argparse.ArgumentParser(
        description='Компенсация высоты платы в G-code для фрезерного станка',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
    Примеры использования:
      python pcb_compensator.py board.gcode
      python pcb_compensator.py board.gcode --port COM3 --points 8
      python pcb_compensator.py board.gcode --scan-z -38 --segment-length 3
            """
    )

    parser.add_argument('gcode_file', help='Входной G-code файл')
    parser.add_argument('--port', default='COM9', help='COM порт станка (по умолчанию: COM9)')
    parser.add_argument('--baud', type=int, default=115200, help='Скорость обмена (по умолчанию: 115200)')
    parser.add_argument('--points', type=int, default=10, help='Количество точек сканирования по каждой оси')
    parser.add_argument('--scan-z', type=float, default=-40.0, help='Начальная высота сканирования (мм)')
    parser.add_argument('--segment-length', type=float, default=5.0, help='Макс. длина сегмента интерполяции (мм)')
    parser.add_argument('--attempts', type=int, default=3, help='Количество попыток измерения')
    parser.add_argument('--output', help='Имя выходного файла (по умолчанию: имя_файла_compensated_дата.gcode)')

    args = parser.parse_args()

    # Проверка файла
    if not os.path.exists(args.gcode_file):
        print(f"Ошибка: файл не найден - {args.gcode_file}")
        return

    print(f"\n{'=' * 60}")
    print("PCB HEIGHT COMPENSATOR")
    print(f"{'=' * 60}")

    mapper = None
    try:
        # Создание маппера
        mapper = PCBHeightMapper(port=args.port, baudrate=args.baud)

        # Настройка параметров
        mapper.grid_points = args.points
        mapper.scan_z = args.scan_z
        mapper.max_segment_length = args.segment_length
        mapper.probe_attempts = args.attempts

        print(f"Параметры:")
        print(f"  Файл: {args.gcode_file}")
        print(f"  Порт: {args.port}")
        print(f"  Точки сканирования: {args.points}×{args.points}")
        print(f"  Высота сканирования: {args.scan_z} мм")
        print(f"  Длина сегмента: {args.segment_length} мм")
        print(f"  Попыток измерения: {args.attempts}")

        # Обработка файла
        mapper.process_gcode_file(args.gcode_file, args.output)

    except KeyboardInterrupt:
        print("\n\nПрограмма прервана пользователем")
    except Exception as e:
        print(f"\nОшибка: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if mapper:
            mapper.close()


if __name__ == "__main__":
    main()