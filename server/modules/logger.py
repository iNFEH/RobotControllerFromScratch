"""Console logging."""

from datetime import datetime
from pathlib import Path


class LogHandler:
    """Manage timestamped messages and optional logging to disk."""

    def __init__(self, save):
        """Configure the log directory and whether to save messages."""
        self._log_directory = "logs"
        self._current_log_filename = None
        self._save = save

    def logInit(self):
        """Announce a new log and create its file when disk logging is enabled."""
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

        if self._save:
            # create the directory incase it does not already exist.
            Path(self._log_directory).mkdir(parents=True, exist_ok=True)

            # name and write initial line to the new log.
            self._current_log_filename = f"{self._log_directory}/log_{timestamp}.txt"
            with Path(self._current_log_filename).open("w") as file:
                file.write(f"---------- Log created at {timestamp} ----------\n")

        print(f"---------- Log created at {timestamp} ----------")

    def logAdd(self, text: str, shouldPrint: bool):
        """Write a message to the log output."""
        # this includes milliseconds.
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S-%f")[:-3]
        entry = f"{timestamp} | {text}\n"

        if self._save:
            # raise error if the log has not been created.
            if self._current_log_filename is None:
                raise ValueError("LOGGER: Log not created. Call logCreate() first.")

            # append to the current logfile.
            with Path(self._current_log_filename).open("a") as file:
                file.write(f"{entry}")

        if shouldPrint:
            print(entry)
