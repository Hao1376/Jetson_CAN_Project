import os
import sys
import cv2
import time
import subprocess
import shlex
import socket

FRAME_FILE = "/home/Jetson/Desktop/project/Cam/Frame_File"
CTRL_SOCKET = "/tmp/cam_ctrl.sock"
VIDEO_DEV = os.environ.get("CAM_VIDEO_DEV", "/dev/video0")

TFTP_SERVER = "192.168.0.100"
LINUX_ADDRESS = "192.168.0.200"
LINUX_MASK = "255.255.255.0"
TFTP_TIMEOUT = 10  # 秒

# C 端协议键名 -> v4l2-ctl 控件名（USB UVC）
V4L2_CTRL = {
    "brightness": "brightness",
    "contrast": "contrast",
    "saturation": "saturation",
    "hue": "hue",
    "gamma": "gamma",
    "gain": "gain",
    "sharpness": "sharpness",
}

cap = None


def Set_Temp_File():
    """创建照片存储目录"""
    os.makedirs(FRAME_FILE, exist_ok=True)
    print(f"照片目录就绪: {FRAME_FILE}")


def Send_With_TFTP(local_path, remote_name):
    """通过 TFTP 上传文件；成功返回 True"""
    cmd = f"tftp -m binary {TFTP_SERVER} -c put {local_path} {remote_name}"
    print(f"执行 TFTP: {cmd}")
    try:
        result = subprocess.run(
            shlex.split(cmd),
            text=True,
            capture_output=True,
            timeout=TFTP_TIMEOUT,
        )
        if result.returncode == 0:
            print(f"TFTP 上传成功: {remote_name}")
            return True
        print(f"TFTP 上传失败: {result.stderr.strip()}")
        return False
    except subprocess.TimeoutExpired:
        print(f"TFTP 上传超时({TFTP_TIMEOUT}s): {remote_name}")
        return False
    except Exception as e:
        print(f"TFTP 命令异常: {e}")
        return False


def Set_Param(key, value):
    """通过 v4l2-ctl 设置 USB UVC 参数"""
    ctrl = V4L2_CTRL.get(key)
    if ctrl is None:
        raise ValueError(f"未知参数: {key}")

    cmd = ["v4l2-ctl", "-d", VIDEO_DEV, f"--set-ctrl={ctrl}={value}"]
    try:
        result = subprocess.run(
            cmd,
            text=True,
            capture_output=True,
            timeout=1.5,
        )
    except FileNotFoundError:
        raise RuntimeError("未安装 v4l2-ctl，请执行 sudo apt install v4l-utils")
    except subprocess.TimeoutExpired:
        raise RuntimeError("v4l2-ctl 执行超时")

    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or f"v4l2-ctl 返回 {result.returncode}")

    print(f"参数已设置: {ctrl}={value}")


def Capture_Frame(site, index, total):
    """拍照、保存并上传；全部成功返回文件名，否则返回 None"""
    print(f"收到拍照命令: 地点={site} 第 {index + 1}/{total} 张")

    ret, frame = cap.read()
    if not ret or frame is None:
        print("拍照失败: 摄像头读取失败")
        return None

    file_name = f"img_{int(time.time() * 1000)}_{site}_{index}.jpg"
    file_path = os.path.join(FRAME_FILE, file_name)

    if not cv2.imwrite(file_path, frame, [cv2.IMWRITE_JPEG_QUALITY, 80]):
        print(f"拍照失败: 图片写入失败 {file_path}")
        return None

    print(f"已保存: {file_path}")

    # 必须等 TFTP 上传成功再回 OK，C 端收到文件名后会删除本地文件
    if not Send_With_TFTP(file_path, file_name):
        print("TFTP 上传失败，本地文件保留")
        return None

    return file_name


def Handle_Line(line):
    """解析一行命令，返回回包字符串（不含换行）"""
    parts = line.split()
    if not parts:
        return "ERROR 空命令"

    cmd = parts[0].upper()

    if cmd == "PARAM":
        if len(parts) < 3:
            return "ERROR PARAM 参数不足"
        try:
            value = int(parts[2])
        except ValueError:
            return "ERROR 参数值不是整数"
        try:
            Set_Param(parts[1], value)
            return "OK"
        except Exception as e:
            return f"ERROR {e}"

    if cmd == "CAPTURE":
        if len(parts) < 4:
            return "ERROR CAPTURE 参数不足"
        try:
            site = int(parts[1])
            index = int(parts[2])
            total = int(parts[3])
        except ValueError:
            return "ERROR CAPTURE 参数不是整数"

        file_name = Capture_Frame(site, index, total)
        if file_name is None:
            return "ERROR 拍照或上传失败"
        return f"OK {file_name}"

    if cmd == "PING":
        return "OK"

    return f"ERROR 未知命令: {cmd}"


def Server_Loop():
    """Unix domain socket 服务端：一次处理一条命令"""
    if os.path.exists(CTRL_SOCKET):
        os.unlink(CTRL_SOCKET)

    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(CTRL_SOCKET)
    srv.listen(5)
    print(f"控制 socket 就绪: {CTRL_SOCKET}")

    while True:
        conn, _ = srv.accept()
        with conn:
            try:
                reader = conn.makefile("r", encoding="utf-8", newline="\n")
                line = reader.readline().strip()
                if not line:
                    continue

                print(f"收到命令: {line}")
                reply = Handle_Line(line) + "\n"
                conn.sendall(reply.encode("utf-8"))
            except BrokenPipeError:
                print("C 端已断开，回包丢弃")
            except Exception as e:
                print(f"处理命令异常: {e}")
                try:
                    conn.sendall(f"ERROR {e}\n".encode("utf-8"))
                except Exception:
                    pass


def Camera_Index():
    """从 /dev/videoN 解析 OpenCV 摄像头编号"""
    prefix = "/dev/video"
    if VIDEO_DEV.startswith(prefix):
        try:
            return int(VIDEO_DEV[len(prefix):])
        except ValueError:
            pass
    return 0


def Init():
    """初始化摄像头和网络"""
    global cap

    cap = cv2.VideoCapture(Camera_Index(), cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"无法打开摄像头 {VIDEO_DEV}，请检查连接")
        sys.exit(1)

    print("正在配置网络...")
    subprocess.run(shlex.split("sudo ip link set end0 down"), check=True)
    subprocess.run(shlex.split(f"sudo ip addr replace {LINUX_ADDRESS}/24 dev end0"), check=True)
    subprocess.run(shlex.split("sudo ip link set end0 up"), check=True)
    print(f"网卡 end0 已配置为 IP:{LINUX_ADDRESS} 掩码:{LINUX_MASK}")


if __name__ == "__main__":
    Init()
    Set_Temp_File()
    Server_Loop()
