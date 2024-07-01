import threading
import time

# 创建一个锁
lock = threading.Lock()

# 定义一个函数，用于模拟长时间运行的任务
def long_running_task(name):
    with lock:
        print(f"{name} has acquired the lock.")
        time.sleep(2)  # 模拟长时间任务
        print(f"{name} is releasing the lock.")

# 创建并启动两个线程，模拟并发执行
thread1 = threading.Thread(target=long_running_task, args=("Thread 1",))
thread2 = threading.Thread(target=long_running_task, args=("Thread 2",))

thread1.start()
thread2.start()

thread1.join()
thread2.join()

print("Both threads have finished their execution.")