import threading
import time

def subprocess(text):
    while 1:
        print(text)
        time.sleep(1)

def main():
    th = threading.Thread(target=subprocess, args=("subprocess",))
    th.start()
    while 1:
        print("main")
        time.sleep(1)

if __name__ == "__main__":
    main()