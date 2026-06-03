import cv2
import numpy as np

def generate_chessboard(squares_x=9, squares_y=7, square_size_px=200):
    # Размеры итогового изображения
    width = squares_x * square_size_px
    height = squares_y * square_size_px
    
    # Создаем белое изображение
    image = np.ones((height, width), dtype=np.uint8) * 255
    
    # Рисуем черные квадраты в шахматном порядке
    for y in range(squares_y):
        for x in range(squares_x):
            if (x + y) % 2 == 1:
                top_left_x = x * square_size_px
                top_left_y = y * square_size_px
                bottom_right_x = top_left_x + square_size_px
                bottom_right_y = top_left_y + square_size_px
                
                # Заполняем черный квадрат
                image[top_left_y:bottom_right_y, top_left_x:bottom_right_x] = 0
                
    return image

if __name__ == '__main__':
    # Генерируем доску 9x7 квадратов (8x6 внутренних углов)
    # Размер квадрата 200 пикселей для высокой четкости при печати
    chessboard = generate_chessboard(squares_x=9, squares_y=7, square_size_px=250)
    
    # Путь для сохранения
    output_path = '/home/panik_bel/.gemini/antigravity/scratch/ros2_ws_AGrep/ros2_ws/src/fake_tag_publisher/config/chessboard_pattern.png'
    
    # Сохраняем изображение
    cv2.imwrite(output_path, chessboard)
    print(f"Printable chessboard pattern successfully saved to: {output_path}")
