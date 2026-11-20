from ugv_line.line_math import sensor_positions

from ugv_simulation.line_camera import channel_levels, sensor_columns, to_millivolts

POSITIONS = sensor_positions(8, 0.009525)


def test_left_sensor_is_on_the_left_of_the_image():
    columns = sensor_columns(96, POSITIONS, 0.04)
    assert columns == sorted(columns)                # D1 (left, +y) .. D8 (right)
    assert columns[0] < 48 < columns[-1]
    # symmetric about the image centre
    assert columns[0] + columns[-1] == 95


def test_sensor_columns_stay_inside_the_image():
    columns = sensor_columns(8, [1.0, 0.0, -1.0], 0.04)
    assert columns == [0, 4, 7]


def test_dark_stripe_under_one_sensor():
    width, height = 96, 8
    columns = sensor_columns(width, POSITIONS, 0.04)
    data = bytearray([200] * width * height)
    for row in range(height):
        for col in range(columns[3] - 2, columns[3] + 3):
            data[row * width + col] = 10
    levels = channel_levels(bytes(data), width, height, width, columns)
    assert levels[3] < 0.1
    assert all(level > 0.7 for k, level in enumerate(levels) if k != 3)


def test_dark_reads_high_like_the_deneyap_adc():
    assert to_millivolts([0.0, 1.0]) == [3300, 0]
