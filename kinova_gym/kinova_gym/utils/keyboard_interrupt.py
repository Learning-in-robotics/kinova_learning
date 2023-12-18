import threading
import pynput.keyboard
import time

stop_thread = False

def on_press(key):
    global stop_thread
    if key == pynput.keyboard.Key.esc:
        stop_thread = True

if __name__ == '__main__':
    # print hello in a while loop, but when the user presses a key, stop the loop
    # and print goodbye
    listener = pynput.keyboard.Listener(on_press=on_press)
    listener.start()

    def print_hello():
        global stop_thread
        while True:
            print('hello')
            if stop_thread:
                break
            time.sleep(0.1)

    t = threading.Thread(target=print_hello)
    t.start()
    t.join()

    print('goodbye')