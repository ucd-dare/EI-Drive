"""
Keyboard listener: a keyboard listener to record typying and can be used for pause, resume and stop the program
using multi-threading.
"""

from pynput import keyboard


class KeyListener(object):
    """
    A keyboard listener class to record the states of keys.

    Attributes
    ----------
    keys : dict
        Keyboard keys and states
    """
    def __init__(self):
        """
        Initialize a listener instance
        """
        self.keys = {'p': False, 'esc': False}

    def start(self):
        """
        Start keyboard listening
        """
        listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        listener.start()

    def on_press(self, key):
        """
        Update the dict when a key is pressed

        Parameters
        ----------
        key: A KeyCode represents the description of a key code used by the operating system.

        """
        try:
            print(f'alphanumeric key {key.char} pressed')
            if key.char in self.keys:
                self.keys[key.char] = not self.keys[key.char]
            else:
                self.keys[key] = False
        except AttributeError:
            print(f'special key {key} pressed')

    def on_release(self, key):
        """
        Update the dict keys when a key is released

        Parameters
        ----------
        key: A KeyCode represents the description of a key code used by the operating system.

        """
        if key == keyboard.Key.esc:
            self.keys['esc'] = True


