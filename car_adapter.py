from Raspbot_Lib import Raspbot
import time
import math

bot = Raspbot()
debug = 0

#ехать вперЄд вычисл€ет значени€ дл€ четырЄх моторов

def move_forward(speed):
    l1, l2, r1, r2 = set_deflection(speed, 90)
    if debug == 1:
        print(f"L1:{l1:>4}| w |R1:{r1:<4}")
        print(f"L2:{l2:>4}|   |R2:{r2:<4}\n")
    bot.Ctrl_Muto(0, l1 + 0)
    bot.Ctrl_Muto(1, l2 + 0)
    bot.Ctrl_Muto(2, r1 + 0)
    bot.Ctrl_Muto(3, r2 + 0)

#eхать вперЄд с процентной регулировкой стороны

def move_param_forward(speed, param_percent):

    # защитные приведени€
    s = int(speed)
    if s < 0:
        s = 0
    if s > 255:
        s = 255

    # ограничиваем процент
    try:
        p = float(param_percent)
    except Exception:
        p = 0.0
    if p > 100.0:
        p = 100.0
    if p < -100.0:
        p = -100.0

    # базовые значени€ моторов дл€ движени€ вперЄд
    l1, l2, r1, r2 = set_deflection(s, 90)

    # если процент 0 Ч просто ехать вперЄд
    if abs(p) < 1e-6:
        bot.Ctrl_Muto(0, _clamp_pwm(l1))
        bot.Ctrl_Muto(1, _clamp_pwm(l2))
        bot.Ctrl_Muto(2, _clamp_pwm(r1))
        bot.Ctrl_Muto(3, _clamp_pwm(r2))
        return True

    # вычисл€ем поправку как процент от соответствующей стороны
    if p >= 0:
        # усиливаем правую сторону
        r1_adj = int(round(r1 * (p / 100.0)))
        r2_adj = int(round(r2 * (p / 100.0)))
        bot.Ctrl_Muto(0, _clamp_pwm(l1))
        bot.Ctrl_Muto(1, _clamp_pwm(l2))
        bot.Ctrl_Muto(2, _clamp_pwm(r1 + r1_adj))
        bot.Ctrl_Muto(3, _clamp_pwm(r2 + r2_adj))
    else:
        # p < 0: усиливаем левую сторону
        pp = abs(p)
        l1_adj = int(round(l1 * (pp / 100.0)))
        l2_adj = int(round(l2 * (pp / 100.0)))
        bot.Ctrl_Muto(0, _clamp_pwm(l1 + l1_adj))
        bot.Ctrl_Muto(1, _clamp_pwm(l2 + l2_adj))
        bot.Ctrl_Muto(2, _clamp_pwm(r1))
        bot.Ctrl_Muto(3, _clamp_pwm(r2))

    return True

#ехать назад

def move_backward(speed):
    l1, l2, r1, r2 = set_deflection(speed, 270)
    if debug == 1:
        print(f"L1:{l1:>4}| x |R1:{r1:<4}")
        print(f"L2:{l2:>4}|   |R2:{r2:<4}\n")
    bot.Ctrl_Muto(0, l1 + 0)
    bot.Ctrl_Muto(1, l2 + 0)
    bot.Ctrl_Muto(2, r1 + 0)
    bot.Ctrl_Muto(3, r2 + 0)
    
#ехать влево

def move_left(speed):
    l1, l2, r1, r2 = set_deflection(speed, 180)
    if debug == 1:
        print(f"L1:{l1:>4}| a |R1:{r1:<4}")
        print(f"L2:{l2:>4}|   |R2:{r2:<4}\n")
    bot.Ctrl_Muto(0, l1 + 0)
    bot.Ctrl_Muto(1, l2 + 0)
    bot.Ctrl_Muto(2, r1 + 0)
    bot.Ctrl_Muto(3, r2 + 0)

#ехать вправо

def move_right(speed):
    l1, l2, r1, r2 = set_deflection(speed, 0)
    if debug == 1:
        print(f"L1:{l1:>4}| d |R1:{r1:<4}")
        print(f"L2:{l2:>4}|   |R2:{r2:<4}\n")
    bot.Ctrl_Muto(0, l1 + 0)
    bot.Ctrl_Muto(1, l2 + 0)
    bot.Ctrl_Muto(2, r1 + 0)
    bot.Ctrl_Muto(3, r2 + 0)

#поворот влево на месте

def rotate_left(speed):
    l1, l2, r1, r2 = set_deflection(speed, 180)
    if debug == 1:
        print(f"L1:{l1:>4}| q |R1:{r1:<4}")
        print(f"L2:{-l2:>4}|   |R2:{abs(r2):<4}\n")
    bot.Ctrl_Muto(0, l1 + 0)
    bot.Ctrl_Muto(1, -l2 + 0)
    bot.Ctrl_Muto(2, r1 + 0)
    bot.Ctrl_Muto(3, abs(r2) + 0)

#поворот вправо на месте

def rotate_right(speed):
    l1, l2, r1, r2 = set_deflection(speed, 0)
    if debug == 1:
        print(f"L1:{l1:>4}| e |R1:{r1:<4}")
        print(f"L2:{abs(l2):>4}|   |R2:{-r2:<4}\n")
    bot.Ctrl_Muto(0, l1 + 0)
    bot.Ctrl_Muto(1, abs(l2) + 0)
    bot.Ctrl_Muto(2, r1 + 0)
    bot.Ctrl_Muto(3, -r2 + 0)

#движение по диагонали влево вперЄд

def move_diagonal_left_front(speed):
    l1, l2, r1, r2 = set_deflection(speed, 135)
    if debug == 1:
        print(f"L1:{l1:>4}| q |R1:{r1:<4}")
        print(f"L2:{l2:>4}|   |R2:{r2:<4}\n")
    bot.Ctrl_Muto(0, l1 + 0)
    bot.Ctrl_Muto(1, l2 + 0)
    bot.Ctrl_Muto(2, r1 + 0)
    bot.Ctrl_Muto(3, r2 + 0)
    
#движение по диагонали влево назад

def move_diagonal_left_back(speed):
    l1, l2, r1, r2 = set_deflection(speed, 225)
    if debug == 1:
        print(f"L1:{l1:>4}| z |R1:{r1:<4}")
        print(f"L2:{l2:>4}|   |R2:{r2:<4}\n")
    bot.Ctrl_Muto(0, l1 + 0)
    bot.Ctrl_Muto(1, l2 + 0)
    bot.Ctrl_Muto(2, r1 + 0)
    bot.Ctrl_Muto(3, r2 + 0)
    
#движение по диагонали вправо вперЄд

def move_diagonal_right_front(speed):
    l1, l2, r1, r2 = set_deflection(speed, 45)
    if debug == 1:
        print(f"L1:{l1:>4}| e |R1:{r1:<4}")
        print(f"L2:{l2:>4}|   |R2:{r2:<4}\n")
    bot.Ctrl_Muto(0, l1 + 0)
    bot.Ctrl_Muto(1, l2 + 0)
    bot.Ctrl_Muto(2, r1 + 0)
    bot.Ctrl_Muto(3, r2 + 0)

#движение по диагонали вправо назад

def move_diagonal_right_back(speed):
    l1, l2, r1, r2 = set_deflection(speed, 315)
    if debug == 1:
        print(f"L1:={l1:>4}| c |R1:={r1:<4}")
        print(f"L2:={l2:>4}|   |R2:={r2:<4}\n")
    bot.Ctrl_Muto(0, l1 + 0)
    bot.Ctrl_Muto(1, l2 + 0)
    bot.Ctrl_Muto(2, r1 + 0)
    bot.Ctrl_Muto(3, r2 + 0)
    
#остановка робота

def stop_robot():
        bot.Ctrl_Car(0, 0, 0)
        bot.Ctrl_Car(1, 0, 0)
        bot.Ctrl_Car(2, 0, 0)
        bot.Ctrl_Car(3, 0, 0)

#безопасна€ остановка робота 

def stop():
    for i in range(4):
        time.sleep(0.25)
        bot.Ctrl_Car(0, 0, 0)
        bot.Ctrl_Car(1, 0, 0)
        bot.Ctrl_Car(2, 0, 0)
        bot.Ctrl_Car(3, 0, 0)

#расчЄт значений дл€ четырЄх моторов по направлению

def set_deflection(speed, deflection):
    if(speed>255):speed=255
    if(speed<0):speed=0
    rad2deg = math.pi / 180
    vx = speed * math.cos(deflection * rad2deg)
    vy = speed * math.sin(deflection * rad2deg)
    l1 = int(vy + vx) 
    l2 = int(vy - vx)
    r1 = int(vy - vx)
    r2 = int(vy + vx)
    return l1,l2,r1,r2
    
#расчЄт дл€ компенсации скольжени€

def set_deflection_rate(speed, deflection,rate):
    if(speed>255):speed=255
    if(speed<0):speed=0
    rad2deg = math.pi / 180
    vx = speed * math.cos(deflection * rad2deg)
    vy = speed * math.sin(deflection * rad2deg)
    vp = -rate * (117+ 132)/2
    l1 = int(vy + vx - vp) 
    l2 = int(vy - vx + vp)
    r1 = int(vy - vx - vp)
    r2 = int(vy + vx + vp)
    return l1,l2,r1,r2

#”становка моторов

def drifting(speed,deflection,rate):
    l1,l2,r1,r2=set_deflection_rate(speed,deflection,rate)
    bot.Ctrl_Muto(0, l1+ 0)
    bot.Ctrl_Muto(1, l2+ 0)
    bot.Ctrl_Muto(2, r1+ 0)
    bot.Ctrl_Muto(3, r2+ 0)