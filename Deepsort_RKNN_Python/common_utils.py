import cv2
import numpy as np
import serial
import serial.tools.list_ports

#返回摄像头的索引
def detect_cameras():
    available_cameras = []
    # 遍历索引，尝试打开摄像头
    for index in range(10):  # 可以根据需要调整索引的范围
        cap = cv2.VideoCapture(index)
        # 检查摄像头是否成功打开
        if cap.isOpened():
            available_cameras.append(index)
            cap.release()
    return available_cameras
def detect_gps_port():
    available_ports = list(serial.tools.list_ports.comports())
    gps_ports = []
    print("available_ports",available_ports)
    for port in available_ports:
        try:
            # 尝试打开串口
            # ser = serial.Serial(port.device, 115200)
            ser = serial.Serial(port.device, baudrate=115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                                stopbits=serial.STOPBITS_ONE)

            res=gps(ser)
            print(res)
            if res[0]!=0:
                gps_ports.append(port.device)
            # 关闭串口
            ser.close()
            if len(gps_ports)>0:
                break
        except :
            pass

    return gps_ports

def gps(ser):
    hdop, alt, lat,lon,sog,cog= 0,0,0,0,0,0
    line = str(ser.readline())
    GN_line = line
    # print(GN_line)
    if GN_line.startswith("b\'$GNGGA"):
        # print(GN_line)
        GNGGA_line = str(GN_line).split(',')  # 将line以“，”为分隔符
        if GNGGA_line[6] != '0':
            hdop = float(GNGGA_line[8])
            alt = float(GNGGA_line[9])

    elif GN_line.startswith("b\'$GNRMC"):
        # print(GN_line)
        GNRMC_line = str(GN_line).split(',')  # 将line以“，”为分隔符
        if GNRMC_line[2] == 'A':
            lat = float(GNRMC_line[3][:2]) + float(GNRMC_line[3][2:]) / 60
            lon = float(GNRMC_line[5][:3]) + float(GNRMC_line[5][3:]) / 60
            sog = float(GNRMC_line[7])
            cog = float(GNRMC_line[8])
            # print("纬度：  " + GNRMC_line[4] + " " + str(lat), "经度：  " + GNRMC_line[6] + " " + str(lon), "速度/节：", sog,
            #       "   航向/度：  ", cog, "hdop：  ", hdop, "    高程：  ", alt)

            return hdop, alt, lat,lon,sog,cog
#求点p是否在区域pts里面

palette = (2 ** 11 - 1, 2 ** 15 - 1, 2 ** 20 - 1)
def isIn_2(p, pts):
    # pts = pts[0]
    px, py = p
    is_in = False
    for i, corner in enumerate(pts):
        next_i = i + 1 if i + 1 < len(pts) else 0
        x1, y1 = corner
        x2, y2 = pts[next_i]
        x1, y1, x2, y2 = int(float(x1)), int(float(y1)), int(float(x2)), int(float(x2))
        if (x1 == px and y1 == py) or (x2 == px and y2 == py):  # if point is on vertex
            is_in = True
            break
        if min(y1, y2) < py <= max(y1, y2):  # find horizontal edges of polygon
            x = x1 + (py - y1) * (x2 - x1) / (y2 - y1)
            if x == px:  # if point is on edge
                is_in = True
                break
            elif x > px:  # if point is on left-side of line
                is_in = not is_in
    return is_in

def isIn(p, pts):
    # pts = pts[0]
    px, py = p
    cross = 0
    for i, corner in enumerate(pts):
        next_i = i + 1 if i + 1 < len(pts) else 0
        x1, y1 = corner
        x2, y2 = pts[next_i]
        if (x1 == px and y1 == py) or (x2 == px and y2 == py):  # if point is on vertex
            return True
        ymin = y1 if y1 < y2 else y2
        ymax = y1 if y2 <= y1 else y2
        if ymin < py <= ymax:  # find horizontal edges of polygon
            x = x1 + (x2 - x1) * (py - y1) / (y2 - y1)
            if x == px:  # if point is on edge
                return True
            elif x > px:  # if point is on left-side of line,have intersect
                cross += 1
    return (cross % 2 == 1)

# 求两个列表的交集
def list_intersection(list1: list, list2: list):
    return list(set(list1).intersection(set(list2)))

def xyxy_to_xywh(*xyxy):
    """" Calculates the relative bounding box from absolute pixel values. """
    bbox_left = min([xyxy[0].item(), xyxy[2].item()])
    bbox_top = min([xyxy[1].item(), xyxy[3].item()])
    bbox_w = abs(xyxy[0].item() - xyxy[2].item())
    bbox_h = abs(xyxy[1].item() - xyxy[3].item())
    x_c = (bbox_left + bbox_w / 2)
    y_c = (bbox_top + bbox_h / 2)
    w = bbox_w
    h = bbox_h
    return x_c, y_c, w, h

def compute_color_for_labels(label):
    """
    Simple function that adds fixed color depending on the class
    """
    color = [int((p * (label ** 2 - label + 1)) % 255) for p in palette]
    return tuple(color)
def draw_boxes(img, bbox, identities=None, offset=(0, 0)):
    for i, box in enumerate(bbox):
        x1, y1, x2, y2 = [int(i) for i in box]
        x1 += offset[0]
        x2 += offset[0]
        y1 += offset[1]
        y2 += offset[1]
        # box text and bar
        id = int(identities[i]) if identities is not None else 0
        color = compute_color_for_labels(id)
        label = '{}{:d}'.format("", id)
        t_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_PLAIN, 2, 2)[0]
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 4)
        cv2.rectangle(
            img, (x1, y1), (x1 + t_size[0] + 3, y1 + t_size[1] + 4), color, -1)
        cv2.putText(img, label, (x1, y1 +
                                 t_size[1] + 4), cv2.FONT_HERSHEY_PLAIN, 2, [255, 255, 255], 2)
    return img

def cal_area_duty_ratio(rect,boxs,img_src):
    rect_area=cv2.contourArea(rect)
    img=np.zeros((img_src.shape[0],img_src.shape[1]),np.uint8)
    for box in boxs:
        x1, y1, x2, y2 = box
        x1, y1, x2, y2=int(x1), int(y1), int(x2), int(y2)
        # for i in range(int(x2 - x1)):
        #     for j in range(int(y2 - y1)):
        #         img[y1 + j, x1 + i] = 255
        img[y1:y2, x1:x2] = 255
    # cv2.imshow('area_duty_ratio',cv2.resize(img.copy(), (img.shape[1]//3,img.shape[0]//3)))
    # cv2.waitKey(1)
    # print(img.shape)
    return round((img==255).sum()/rect_area,3)
