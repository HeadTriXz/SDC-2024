from telemetry.webapp.telemetry_server import TelemetryServer


def main() -> None:
    """Start the main loop."""
    server = TelemetryServer()
    server.start()
    input("Press Enter to exit...")


if __name__ == "__main__":
    """The main function."""
    main()
