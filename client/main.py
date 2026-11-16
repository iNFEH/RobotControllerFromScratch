"""Launch the robot control client."""

from modules import client


def main():
    """Create the client interface and run its event loop."""
    client.ClientHandler()


if __name__ == "__main__":
    main()
