import sensor, image, time, os, tf, uos, gc, json
from pyb import Pin, LED
from pyb import UART

pin0 = Pin('P0', Pin.IN, Pin.PULL_UP)
pin1 = Pin('P1', Pin.OUT_PP, Pin.PULL_NONE)
pin2 = Pin('P2', Pin.OUT_PP, Pin.PULL_NONE)
pin3 = Pin('P3', Pin.OUT_PP, Pin.PULL_NONE)
pin4 = Pin('P4', Pin.IN, Pin.PULL_UP)
pin5 = Pin('P5', Pin.IN, Pin.PULL_UP)
pin6 = Pin('P6', Pin.OUT_PP, Pin.PULL_NONE)
# Define LED pins
green_led = LED(2)  # Green LED
blue_led = LED(3)   # Blue LED
red_led = LED(1)    # Red LED

blue_led.on()

sensor.reset()                         # 重置并初始化传感器
sensor.set_pixformat(sensor.RGB565)    # 设置像素格式为RGB565（或灰度图）
sensor.set_framesize(sensor.VGA)      # 设置帧大小为VGA（640x480）
sensor.skip_frames(time=2000)          # 等待相机自动调整

net1 = "trained.tflite"
net2 = "trained_car_V1.1.tflite"
labels1 = [line.rstrip('\n') for line in open("labels.txt")]
labels2 = [line.rstrip('\n') for line in open("labels_car_V1.1.txt")]

try:
    # 加载模型1
    net1 = tf.load("trainedV2.0.tflite", load_to_fb=uos.stat('trainedV2.0.tflite')[6] > (gc.mem_free() - (64*1024)))
except Exception as e:
    print(e)
    raise Exception('加载 "trainedV2.0.tflite" 失败，请确保已将.tflite和labels.txt文件复制到存储设备上。(' + str(e) + ')')

try:
    # 加载模型2
    net2 = tf.load("trained_car_V1.1.tflite", load_to_fb=uos.stat('trained_car_V1.1.tflite')[6] > (gc.mem_free() - (64*1024)))
except Exception as e:
    print(e)
    raise Exception('加载 "trained_car_V1.1.tflite" 失败，请确保已将.tflite和car_labels.txt文件复制到存储设备上。(' + str(e) + ')')

try:
    labels1 = [line.rstrip('\n') for line in open("labelsV2.0.txt")]
except Exception as e:
    raise Exception('加载 "labelsV2.0.txt" 失败，请确保已将.tflite和labels.txt文件复制到存储设备上。(' + str(e) + ')')

try:
    labels2 = [line.rstrip('\n') for line in open("labels_car_V1.1.txt")]
except Exception as e:
    raise Exception('加载 "labels_car_V1.1.txt" 失败，请确保已将.tflite和car_labels.txt文件复制到存储设备上。(' + str(e) + ')')

clock = time.clock()

blue_led.off()
def mmain():

    red_led.off()
    green_led.off()
    blue_led.on()
    last_prediction = None
    prediction_count = 0
    current_prediction = 0
    max_probability = 0.0
    while True:
        clock.tick()

        img = sensor.snapshot()

        # 进行图像分类（模型1）
        for obj in net1.classify(img, min_scale=1.0, scale_mul=0.8, x_overlap=0.5, y_overlap=0.5):
            print("**********\n检测结果 [x=%d,y=%d,w=%d,h=%d]" % obj.rect())
            img.draw_rectangle(obj.rect())
            # 将标签和置信度值组合成元组列表
            predictions_list = list(zip(labels1, obj.output()))

            current_prediction = 0
            max_probability = 0.0

            # 选择具有最大概率的预测结果
            for i, prediction in enumerate(predictions_list):
                if prediction[1] > max_probability:
                    current_prediction = i
                    max_probability = prediction[1]

            # 判断连续五次预测结果是否相同
            if current_prediction == last_prediction:
                prediction_count += 1
                if prediction_count >= 4:
                    # 在屏幕上显示概率和标签
                    img.draw_string(5, 10, "Prediction: %s" % predictions_list[current_prediction][0], color=(255, 255, 255), scale=1)
                    img.draw_string(10, 40, "Probability: %.2f" % predictions_list[current_prediction][1], color=(255, 255, 255), scale=2)
                    print("识别结果:", predictions_list[current_prediction][0])
                    return current_prediction
            else:
                prediction_count = 1

            last_prediction = current_prediction

def car():
    last_prediction = None
    prediction_count = 0
    current_prediction = 0
    max_probability = 0.0
    car_detected = False  # 是否检测到小车
    while True:
        clock.tick()

        img = sensor.snapshot()

        # 进行图像分类（模型2）
        for obj in net2.classify(img, min_scale=1.0, scale_mul=0.8, x_overlap=0.5, y_overlap=0.5):
            print("**********\n检测结果 [x=%d,y=%d,w=%d,h=%d]" % obj.rect())
            img.draw_rectangle(obj.rect())
            # 将标签和置信度值组合成元组列表
            predictions_list = list(zip(labels2, obj.output()))

            current_prediction = 0
            max_probability = 0.0

            # 选择具有最大概率的预测结果
            for i, prediction in enumerate(predictions_list):
                if prediction[1] > max_probability:
                    current_prediction = i
                    max_probability = prediction[1]

            if current_prediction == last_prediction:
                prediction_count += 1
                if prediction_count >= 3:
                    # 在屏幕上显示概率和标签
                    img.draw_string(5, 10, "Prediction: %s" % predictions_list[current_prediction][0], color=(255, 255, 255), scale=1)
                    img.draw_string(10, 40, "Probability: %.2f" % predictions_list[current_prediction][1], color=(255, 255, 255), scale=2)
                
                    print("识别结果:", predictions_list[current_prediction][0])
                    if current_prediction == 1:                 
                        if not pin5.value() and predictions_list[current_prediction][1] > 0.75:  # 检测到小车                                        
                            return 1
                    else:
                        return 0
            else:
                prediction_count = 1

            last_prediction = current_prediction

while True:
    count = 10
    num = 10
    if not pin4.value():
        count = mmain()
    if count == 2:
        print("true red")
        pin1.value(pin0.value())
        pin2.value(not pin0.value())
        pin3.value(not pin0.value())

    elif count == 3:
        print("false red")
        pin1.value(not pin0.value())
        pin2.value(not pin0.value())
        pin3.value(pin0.value())

    elif count == 1:
        print("true blue")
        pin1.value(not pin0.value())
        pin2.value(pin0.value())
        pin3.value(not pin0.value())

    elif count == 0:
        print("false blue")
        pin1.value(not pin0.value())
        pin2.value(not pin0.value())
        pin3.value(pin0.value())

    else:
        pin1.value(not pin0.value())
        pin2.value(not pin0.value())
        pin3.value(not pin0.value())


    num=car()
    if num==1:
        pin6.value(pin0.value())
        blue_led.off()
        green_led.off()
        red_led.on()
    if num==0:
        pin6.value(not pin0.value())
        blue_led.off()
        red_led.off()
        green_led.on()