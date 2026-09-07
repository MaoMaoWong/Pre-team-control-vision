"""九宫格汉字识别。类别顺序与 hanzi.yaml 一致。"""
from __future__ import annotations
import argparse, json
import struct
from pathlib import Path
from typing import Dict, Optional, Tuple
import cv2
import numpy as np
from ultralytics import YOLO

DEFAULT_MODEL = Path(__file__).with_name("trained.pt")
# 协议规定的偏旁名称及 ID（必须与训练集/通信协议保持一致）
CLASS_NAMES = ("禾", "人", "氺", "而", "王", "山", "雨", "口", "木")
RADICAL_IDS = {name: i for i, name in enumerate(CLASS_NAMES)}
GRID_WORLD_POS = {(0,0):(68,-24),(0,1):(68,0),(0,2):(68,24),(1,0):(44,-24),(1,1):(44,0),(1,2):(44,24),(2,0):(20,-24),(2,1):(20,0),(2,2):(20,24)}

def order_corners(points: np.ndarray) -> np.ndarray:
    points = np.asarray(points, dtype=np.float32).reshape(4, 2)
    s, d = points.sum(1), np.diff(points, axis=1).ravel()
    return points[[np.argmin(s), np.argmin(d), np.argmax(s), np.argmax(d)]]

def find_board_corners(image: np.ndarray) -> Optional[np.ndarray]:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(cv2.GaussianBlur(gray, (5,5), 0), 40, 140)
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    area_limit = image.shape[0] * image.shape[1] * .05; candidates = []
    for c in contours:
        if cv2.contourArea(c) < area_limit: continue
        a = cv2.approxPolyDP(c, .03 * cv2.arcLength(c, True), True)
        if len(a) == 4 and cv2.isContourConvex(a): candidates.append((cv2.contourArea(c), a.reshape(4,2)))
    return order_corners(max(candidates, key=lambda x:x[0])[1]) if candidates else None

def warp_board(image, corners, size=768):
    if corners is None: return image, None
    dst = np.float32([[0,0],[size-1,0],[size-1,size-1],[0,size-1]])
    m = cv2.getPerspectiveTransform(order_corners(corners), dst)
    return cv2.warpPerspective(image, m, (size,size)), m

def recognize(image: np.ndarray, model: YOLO, confidence=.25) -> Dict[Tuple[int,int], dict]:
    r = model.predict(image, conf=confidence, verbose=False)[0]; h,w = image.shape[:2]; cells = {}
    if r.boxes is None: return cells
    for box, cls, conf in zip(r.boxes.xyxy.cpu().numpy(), r.boxes.cls.cpu().numpy(), r.boxes.conf.cpu().numpy()):
        x1,y1,x2,y2 = box; col=min(2,max(0,int((x1+x2)/2/w*3))); row=min(2,max(0,int((y1+y2)/2/h*3))); key=(row,col)
        item={"char":CLASS_NAMES[int(cls)] if int(cls)<len(CLASS_NAMES) else str(int(cls)),"confidence":round(float(conf),4),"x_cm":GRID_WORLD_POS[key][0],"y_cm":GRID_WORLD_POS[key][1]}
        if key not in cells or item["confidence"]>cells[key]["confidence"]: cells[key]=item
    return cells

def make_frame(cmd: int, data: bytes = b"") -> bytes:
    """按通信协议3.0组帧：AA55|CMD|LEN(uint16 LE)|DATA|XOR|ED。"""
    if len(data) > 256: raise ValueError("协议数据长度不能超过256字节")
    body = bytes([cmd]) + struct.pack("<H", len(data)) + data
    checksum = 0
    for b in body: checksum ^= b
    return b"\xAA\x55" + body + bytes([checksum, 0xED])

def make_grid_data(result) -> bytes:
    """生成72字节GRID_DATA。棋盘原点左下角，格子边长200 mm。"""
    data = bytearray()
    for row in range(3):
        for col in range(3):
            item = result.get((row, col), {})
            char = item.get("char")
            if char not in RADICAL_IDS: raise ValueError(f"第{row},{col}格未识别")
            u, v = col * 200 + 100, (2 - row) * 200 + 100
            data += struct.pack("<BhhB2x", RADICAL_IDS[char], u, v, 0)
    return bytes(data)

def send_grid_data(result, port: str, baudrate: int = 115200):
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError("发送功能需要安装 pyserial：python -m pip install pyserial") from exc
    frame = make_frame(0x21, make_grid_data(result))
    with serial.Serial(port, baudrate=baudrate, timeout=1) as ser:
        ser.write(frame); ser.flush()
    print("已发送 GRID_DATA:", frame.hex(" ").upper())

def draw_grid_result(image: np.ndarray, result):
    """在图像上绘制固定的行列编号、汉字和相对坐标，避免标注顺序混淆。"""
    out = image.copy(); h, w = out.shape[:2]
    for i in (1, 2):
        cv2.line(out, (w*i//3, 0), (w*i//3, h), (0, 255, 0), 2)
        cv2.line(out, (0, h*i//3), (w, h*i//3), (0, 255, 0), 2)
    for (row, col), item in result.items():
        cx, cy = int((col + .5)*w/3), int((row + .5)*h/3)
        text = f"{row},{col}: {item['char'] or '未知'}"
        cv2.putText(out, text, (cx-70, cy), cv2.FONT_HERSHEY_SIMPLEX, .8, (0,0,255), 2, cv2.LINE_AA)
    return out

def detect_chessboard(image_path=None, use_camera=False, model_path=str(DEFAULT_MODEL), confidence=.25, show=False, port=None):
    model=YOLO(model_path)
    if use_camera:
        cap=cv2.VideoCapture(0); ok,image=cap.read(); cap.release()
        if not ok: raise RuntimeError("摄像头打开失败")
    else:
        if not image_path: raise ValueError("请提供 --image 或使用 --camera")
        image=cv2.imread(str(image_path))
        if image is None: raise FileNotFoundError(image_path)
    board,_=warp_board(image,find_board_corners(image)); detected=recognize(board,model,confidence); output=[]
    for row in range(3):
        line=[]
        for col in range(3):
            pos=GRID_WORLD_POS[(row,col)]; item=detected.get((row,col),{"char":None,"confidence":0.0,"x_cm":pos[0],"y_cm":pos[1]})
            line.append(item["char"] or "未知"); output.append({"row":row,"col":col,**item})
        print(" | ".join(line))
    print(json.dumps(output,ensure_ascii=False,indent=2))
    if port: send_grid_data(detected, port)
    if show:
        cv2.imshow("Hanzi 3x3 (row,col)", draw_grid_result(board, detected)); cv2.waitKey(0); cv2.destroyAllWindows()
    return output

if __name__ == "__main__":
    p=argparse.ArgumentParser(description="识别九宫格汉字并输出相对位置"); p.add_argument("--image",default="test.jpg"); p.add_argument("--camera",action="store_true"); p.add_argument("--model",default=str(DEFAULT_MODEL)); p.add_argument("--conf",type=float,default=.25); p.add_argument("--show",action="store_true"); p.add_argument("--port",help="USB虚拟串口，如COM5；指定后发送GRID_DATA"); a=p.parse_args()
    detect_chessboard(None if a.camera else a.image,a.camera,a.model,a.conf,a.show,a.port)
