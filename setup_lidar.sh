#!/bin/bash
# Скрипт настройки UART для лидара на Raspberry Pi

echo "=========================================="
echo "Настройка UART для лидара T-MINI Plus"
echo "=========================================="
echo ""

# Проверка прав суперпользователя
if [ "$EUID" -ne 0 ]; then
    echo "Пожалуйста, запустите с правами root: sudo $0"
    exit 1
fi

echo "1. Проверка наличия UART портов..."
if [ -e /dev/ttyAMA0 ]; then
    echo "   ✓ /dev/ttyAMA0 найден"
elif [ -e /dev/ttyS0 ]; then
    echo "   ✓ /dev/ttyS0 найден"
else
    echo "   ✗ UART порты не найдены!"
    echo "   Возможно, UART не включен в настройках"
fi

if [ -e /dev/ttyUSB0 ]; then
    echo "   ✓ /dev/ttyUSB0 найден (USB-Serial)"
fi

echo ""
echo "2. Проверка конфигурации /boot/config.txt..."

CONFIG_FILE="/boot/config.txt"
if [ ! -f "$CONFIG_FILE" ]; then
    CONFIG_FILE="/boot/firmware/config.txt"
fi

if [ -f "$CONFIG_FILE" ]; then
    if grep -q "^enable_uart=1" "$CONFIG_FILE"; then
        echo "   ✓ enable_uart=1 уже настроен"
    else
        echo "   Добавление enable_uart=1..."
        echo "" >> "$CONFIG_FILE"
        echo "# UART для лидара" >> "$CONFIG_FILE"
        echo "enable_uart=1" >> "$CONFIG_FILE"
        echo "   ✓ Добавлено"
    fi

    if grep -q "^dtoverlay=uart0" "$CONFIG_FILE"; then
        echo "   ✓ dtoverlay=uart0 уже настроен"
    else
        echo "   Добавление dtoverlay=uart0..."
        echo "dtoverlay=uart0" >> "$CONFIG_FILE"
        echo "   ✓ Добавлено"
    fi
else
    echo "   ✗ Файл $CONFIG_FILE не найден!"
fi

echo ""
echo "3. Настройка прав доступа..."

# Добавляем пользователя в группу dialout
CURRENT_USER=${SUDO_USER:-$USER}
if groups "$CURRENT_USER" | grep -q dialout; then
    echo "   ✓ Пользователь $CURRENT_USER уже в группе dialout"
else
    usermod -a -G dialout "$CURRENT_USER"
    echo "   ✓ Пользователь $CURRENT_USER добавлен в группу dialout"
    echo "   ! Требуется перезайти в систему для применения"
fi

# Устанавливаем права на порты
for PORT in /dev/ttyAMA0 /dev/ttyS0 /dev/ttyUSB0; do
    if [ -e "$PORT" ]; then
        chmod 666 "$PORT"
        echo "   ✓ Права для $PORT установлены"
    fi
done

echo ""
echo "4. Отключение консоли на serial порту..."

# Проверяем cmdline.txt
CMDLINE_FILE="/boot/cmdline.txt"
if [ ! -f "$CMDLINE_FILE" ]; then
    CMDLINE_FILE="/boot/firmware/cmdline.txt"
fi

if [ -f "$CMDLINE_FILE" ]; then
    if grep -q "console=serial0" "$CMDLINE_FILE" || grep -q "console=ttyAMA0" "$CMDLINE_FILE"; then
        echo "   Удаление console из cmdline.txt..."
        sed -i 's/console=serial0,[0-9]\+ //g' "$CMDLINE_FILE"
        sed -i 's/console=ttyAMA0,[0-9]\+ //g' "$CMDLINE_FILE"
        echo "   ✓ Консоль отключена"
    else
        echo "   ✓ Консоль уже отключена"
    fi
fi

echo ""
echo "5. Установка зависимостей Python..."

sudo -u "$CURRENT_USER" pip3 install pyserial numpy || {
    echo "   Попытка установки через apt..."
    apt-get update
    apt-get install -y python3-serial python3-numpy
}

echo ""
echo "=========================================="
echo "Настройка завершена!"
echo "=========================================="
echo ""
echo "Следующие шаги:"
echo "1. Перезагрузите Raspberry Pi: sudo reboot"
echo "2. После перезагрузки проверьте лидар: python3 lidar.py"
echo ""
echo "Если лидар не работает, проверьте:"
echo "- Подключение проводов (TX→RX, RX→TX, VCC→5V, GND→GND)"
echo "- Питание лидара (должен светиться и вращаться)"
echo "- Порт в коде (попробуйте /dev/ttyAMA0, /dev/ttyS0, /dev/ttyUSB0)"
echo ""

read -p "Перезагрузить сейчас? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "Перезагрузка..."
    reboot
fi

