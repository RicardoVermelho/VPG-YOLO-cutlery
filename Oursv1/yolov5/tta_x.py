import subprocess
import time

start = time.time()
subprocess.call('python3 test.py --weights ./runs/train/train_yolov5x_300epochs/weights/best.pt --data ./data/clothing.yaml --img 832 --augment', shell=True)
end = time.time()

total_time = end-start
print(total_time)
