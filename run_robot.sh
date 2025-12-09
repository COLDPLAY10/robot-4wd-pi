#!/bin/bash
# Скрипт быстрого запуска для различных режимов работы робота

echo "======================================"
echo "Скрипты управления роботом 4WD"
echo "======================================"
echo ""
echo "Выберите режим работы:"
echo ""
echo "1) Сбор данных для диплома (по умолчанию)"
echo "2) Сбор данных с настройкой параметров"
echo "3) Детекция белых препятствий"
echo "4) Детекция черных/темно-синих препятствий"
echo "5) Детекция пестрых препятствий"
echo "6) Детекция с сохранением фото"
echo "7) Выход"
echo ""
read -p "Введите номер режима [1-7]: " choice

case $choice in
    1)
        echo ""
        echo "Запуск сбора данных с параметрами по умолчанию..."
        python3 data_collector.py
        ;;
    2)
        echo ""
        read -p "Введите директорию для сохранения [collected_data]: " output_dir
        output_dir=${output_dir:-collected_data}

        read -p "Введите скорость [20]: " speed
        speed=${speed:-20}

        read -p "Введите интервал фото в секундах [0.5]: " interval
        interval=${interval:-0.5}

        read -p "Введите время движения вперед [3.0]: " forward_time
        forward_time=${forward_time:-3.0}

        read -p "Введите время поворота [2.5]: " rotate_time
        rotate_time=${rotate_time:-2.5}

        echo ""
        echo "Запуск сбора данных..."
        python3 data_collector.py \
            --output-dir "$output_dir" \
            --speed "$speed" \
            --photo-interval "$interval" \
            --forward-time "$forward_time" \
            --rotate-time "$rotate_time"
        ;;
    3)
        echo ""
        echo "Запуск детекции белых препятствий..."
        python3 obstacle_detector.py --color white
        ;;
    4)
        echo ""
        echo "Запуск детекции черных/темно-синих препятствий..."
        python3 obstacle_detector.py --color black
        ;;
    5)
        echo ""
        echo "Запуск детекции пестрых препятствий..."
        python3 obstacle_detector.py --color mixed
        ;;
    6)
        echo ""
        echo "Выберите цвет препятствия:"
        echo "1) Белый (white)"
        echo "2) Черный (black)"
        echo "3) Пестрый (mixed)"
        read -p "Введите номер [1-3]: " color_choice

        case $color_choice in
            1) color="white" ;;
            2) color="black" ;;
            3) color="mixed" ;;
            *) color="white" ;;
        esac

        read -p "Введите директорию для фото [obstacle_photos]: " photo_dir
        photo_dir=${photo_dir:-obstacle_photos}

        echo ""
        echo "Запуск детекции $color препятствий с сохранением фото..."
        python3 obstacle_detector.py --color "$color" --save-photos --output-dir "$photo_dir"
        ;;
    7)
        echo "Выход."
        exit 0
        ;;
    *)
        echo "Неверный выбор. Запуск режима по умолчанию..."
        python3 data_collector.py
        ;;
esac

echo ""
echo "======================================"
echo "Завершено!"
echo "======================================"

