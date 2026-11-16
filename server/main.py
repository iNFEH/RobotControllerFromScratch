"""Launch the robot controller server."""

from modules import controller


def main():
    """Initialize the robot controller and run the server."""
    server = controller.ControlHandler()
    server.run()


if __name__ == '__main__':
    main()
