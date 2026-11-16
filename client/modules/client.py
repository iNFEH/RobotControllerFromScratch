"""Provide the graphical client for sending robot commands."""

from pickle import dumps
from re import match
import socket
from threading import Thread
from time import monotonic
import tkinter as tk

import modules.commands as cmds


class ClientHandler:
    """Manages the control interface and communication with the robot server."""

    def __init__(self):
        """Build the interface and start the Tkinter event loop."""
        # the locally stored ip entered by the user
        self._ip_addr = ""
        self._threadPing = None
        self._threadPingRun = True

        # this is the UI (using tkinter), very similar to Java Swing
        self._root = tk.Tk()
        self._root.title("AGV")
        self._root.geometry("300x260")
        self._root.geometry("+200+200")  # this sets the position of the window on screen

        self._manualMode_frame = tk.Frame(self._root)
        manualMode_label = tk.Label(
            self._manualMode_frame,
            text="Steer with the Arrow keys\n\nr = Reverse\n\na = Increase speed\n\nz = Decrease speed\n\nq,w = Tight turns\n\ns = Emergency stop",
        )
        manualMode_label.pack(pady=20)
        manualMode_button = tk.Button(self._manualMode_frame, text="Back", command=self._switchToModeScreen)
        manualMode_button.pack(side=tk.BOTTOM)

        self._autonomousMode_frame = tk.Frame(self._root)
        autonomousMode_label = tk.Label(self._autonomousMode_frame, text="Enter Destination:")
        autonomousMode_label.pack(pady=5)
        self._autonomousMode_lat = tk.Entry(self._autonomousMode_frame, width=30)
        self._autonomousMode_lat.insert(0, "0.000000")
        self._autonomousMode_lat.pack(anchor=tk.CENTER, pady=2)
        self._autonomousMode_long = tk.Entry(self._autonomousMode_frame, width=30)
        self._autonomousMode_long.insert(0, "90.000000")
        self._autonomousMode_long.pack(anchor=tk.CENTER)
        self._autonomousMode_checkbox_usingGPS = tk.IntVar(value=0)
        autonomousMode_checkbox = tk.Checkbutton(
            self._autonomousMode_frame, text="use Lat/Long?", variable=self._autonomousMode_checkbox_usingGPS
        )
        autonomousMode_checkbox.pack(anchor=tk.CENTER)
        autonomousMode_button1 = tk.Button(self._autonomousMode_frame, text="Back", command=self._switchToModeScreen)
        autonomousMode_button1.pack(side=tk.LEFT, pady=5)
        autonomousMode_button2 = tk.Button(self._autonomousMode_frame, text="Start", command=self._submitDest)
        autonomousMode_button2.pack(side=tk.RIGHT, pady=5)

        self._modeScreen_frame = tk.Frame(self._root)
        modeScreen_button1 = tk.Button(self._modeScreen_frame, text="Manual Mode", command=self._switchToManual)
        modeScreen_button1.pack(anchor=tk.CENTER, padx=10, pady=10)
        modeScreen_button2 = tk.Button(self._modeScreen_frame, text="Autonomous Mode", command=self._switchToAutonomous)
        modeScreen_button2.pack(anchor=tk.CENTER, padx=10, pady=5)
        modeScreen_button3 = tk.Button(self._modeScreen_frame, text="Back", command=self._switchToStartScreen)
        modeScreen_button3.pack(side=tk.BOTTOM)

        self._startScreen_frame = tk.Frame(self._root)
        startScreen_label = tk.Label(self._startScreen_frame, text="Server IP:")
        self._startScreen_ip = tk.Entry(self._startScreen_frame, width=30)
        self._startScreen_ip.insert(0, "127.0.0.1")
        startScreen_button = tk.Button(self._startScreen_frame, text="Next", command=self._submitIp)
        startScreen_label.pack(pady=5)
        self._startScreen_ip.pack(pady=0)
        startScreen_button.pack(pady=5)

        self._modeScreen_frame.pack_forget()
        self._manualMode_frame.pack_forget()
        self._autonomousMode_frame.pack_forget()
        self._startScreen_frame.pack()

        self._root.mainloop()

    def _sendCommand(self, command):
        """Serialize a command and send it to the server over TCP."""
        serialized_command = dumps(command)

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(5)
            try:
                s.connect((self._ip_addr, 12345))
                s.sendall(serialized_command)
            except OSError as e:
                print(str(e) + " (IP: " + self._ip_addr + ")")

    def _switchToStartScreen(self):
        """Show the server address screen and disable keyboard controls."""
        self._startScreen_ip.insert(0, self._ip_addr)
        self._manualMode_frame.pack_forget()
        self._modeScreen_frame.pack_forget()
        self._autonomousMode_frame.pack_forget()
        self._startScreen_frame.pack()
        self._root.unbind("<KeyPress>")

    def _switchToModeScreen(self):
        """Show mode selection, stop active modes and stop sending keepalive pings."""
        self._startScreen_frame.pack_forget()
        self._manualMode_frame.pack_forget()
        self._autonomousMode_frame.pack_forget()
        self._modeScreen_frame.pack()
        self._sendCommand(cmds.CommandHandler().QuitManual())
        self._sendCommand(cmds.CommandHandler().QuitAutonomous())  # NOTE should be fine, but keep an eye out.
        self._stopPingThread()
        self._root.unbind("<KeyPress>")

    def _switchToManual(self):
        """Enable manual mode, keyboard controls and keepalive pings."""
        self._startScreen_frame.pack_forget()
        self._modeScreen_frame.pack_forget()
        self._autonomousMode_frame.pack_forget()
        self._manualMode_frame.pack()
        self._sendCommand(cmds.CommandHandler().StartManual())
        self._startPingThread()
        self._root.bind("<KeyPress>", self._onKeyPress)

    def _switchToAutonomous(self):
        """Show destination inputs and disable keyboard controls."""
        self._startScreen_frame.pack_forget()
        self._modeScreen_frame.pack_forget()
        self._manualMode_frame.pack_forget()
        self._autonomousMode_frame.pack()
        self._root.unbind("<KeyPress>")

    def _startPingThread(self):
        """Start a background thread that sends keepalive pings."""
        self._threadPingRun = True
        self._threadPing = Thread(target=self._sendPing)
        self._threadPing.daemon = True
        self._threadPing.start()

    def _stopPingThread(self):
        """Signal the ping thread to stop."""
        self._threadPingRun = False  # NOTE this will cause the thread to return once done

    def _sendPing(self):
        """Send a ping once per second while the ping thread is enabled."""
        last = monotonic()
        while self._threadPingRun:
            current = monotonic()
            if (current - last) >= 1:
                last = current
                self._sendCommand(cmds.CommandHandler().Ping())

    def _submitDest(self):
        """Validate the destination and request autonomous driving."""
        lat = self._autonomousMode_lat.get()
        long = self._autonomousMode_long.get()
        usingGPS = self._autonomousMode_checkbox_usingGPS.get()
        if self._isDestValid(lat) and self._isDestValid(long):
            if usingGPS:
                print("DEST:\n   lat: " + lat + "\n   long: " + long)
            else:
                print("DEST:\n   x: " + lat + "\n   y: " + long)
            self._sendCommand(cmds.CommandHandler().SetDestination(float(lat), float(long), bool(usingGPS)))
            self._sendCommand(cmds.CommandHandler().StartAutonomous())
            self._startPingThread()
        else:
            self._autonomousMode_lat.delete(0, tk.END)
            self._autonomousMode_long.delete(0, tk.END)

    # TODO improve this, long is [-90, 90] and lat is [-180, 180] etc.
    def _isDestValid(self, text):
        """Check a destination value against the selected coordinate format."""
        if self._autonomousMode_checkbox_usingGPS.get():
            dest_pattern = r"^\d{1,3}(?:\.\d{1,6})?$"
            if match(dest_pattern, text):
                return True
            return False
        try:
            float(text)
            return True
        except ValueError:
            return False

    def _submitIp(self):
        """Store a valid server address and show mode selection."""
        text = self._startScreen_ip.get()
        if self._isIpValid(text):
            print("IP:", text)
            self._ip_addr = text
            self._startScreen_ip.delete(0, tk.END)
            self._switchToModeScreen()
        else:
            self._startScreen_ip.delete(0, tk.END)

    def _isIpValid(self, text):
        """Check whether the text is a valid IPv4 address."""
        ipv4_pattern = r"^((25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$"
        if match(ipv4_pattern, text):
            return True
        return False

    # keep this if/elif, we only want to allow 1 command at a time.
    def _onKeyPress(self, event):
        """Send a motor command associated with a keyboard event."""
        if event.keysym == "Up":
            print("Up arrow key pressed")
            self._sendCommand(cmds.CommandHandler().MotorForward())
        elif event.keysym == "r":
            print("r key pressed")
            self._sendCommand(cmds.CommandHandler().MotorReverse())
        elif event.keysym == "Down":
            print("Down arrow key pressed")
            self._sendCommand(cmds.CommandHandler().MotorSmoothStop())
        elif event.keysym == "Left":
            print("Left arrow key pressed")
            self._sendCommand(cmds.CommandHandler().MotorTurnLeft())
        elif event.keysym == "q":
            print("q key pressed")
            self._sendCommand(cmds.CommandHandler().MotorTurnTightLeft())
        elif event.keysym == "Right":
            print("Right arrow key pressed")
            self._sendCommand(cmds.CommandHandler().MotorTurnRight())
        elif event.keysym == "w":
            print("w key pressed")
            self._sendCommand(cmds.CommandHandler().MotorTurnTightRight())
        elif event.keysym == "a":
            print("a key pressed")
            self._sendCommand(cmds.CommandHandler().MotorIncreaseSpeed())
        elif event.keysym == "z":
            print("z key pressed")
            self._sendCommand(cmds.CommandHandler().MotorDecreasedSpeed())
        elif event.keysym == "s":
            print("s key pressed")
            self._sendCommand(cmds.CommandHandler().MotorStop())
