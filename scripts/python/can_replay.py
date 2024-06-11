import argparse
import logging
import threading

from pathlib import Path

from src.constants import CANControlIdentifier, CANFeedbackIdentifier
from src.driving.can import get_can_bus, replay_log


def log_all_can_messages(stopped: threading.Event, blacklist: list[int], whitelist: list[int]) -> threading.Thread:
    """Log all messages of the CAN bus."""
    listen_can = get_can_bus()

    # create and apply filters to listen to only the messages we want to log
    allowed_ids = set([int(v) for v in CANControlIdentifier] + [int(v) for v in CANFeedbackIdentifier]) - set(blacklist)

    if len(whitelist) > 0:
        allowed_ids = set(whitelist)

    logging.info("Listening to messages with IDs: %s", [hex(i) for i in allowed_ids])
    filters = [{"can_id": can_id, "can_mask": 0xFFF, "extended": False} for can_id in allowed_ids]
    listen_can.set_filters(filters)

    def __listen() -> None:
        """Listen to the CAN bus and log all messages."""
        while not stopped.is_set():
            message = listen_can.recv()
            print(message)  # noqa: T201

        listen_can.shutdown()

    thread = threading.Thread(target=__listen, daemon=True)
    thread.start()
    return thread


if __name__ == "__main__":
    """Replay CAN messages from a file and log to a file.

    This script will replay CAN messages from a file and log them to a file.
    We will also log them to the console depending on the arguments.
    You can only use either --blacklist or --whitelist.

    Arguments:
        logfile: The path to the log file.
        --log: Log all CAN messages.
        --blacklist: The list of message IDs to ignore when logging. The IDs are in hexadecimal.
        --whitelist: The list of message IDs to listen to. The IDs are in hexadecimal.
    """
    logging.basicConfig(level=logging.INFO)

    arg_parser = argparse.ArgumentParser(description="Replay CAN messages from a file.")
    arg_parser.add_argument("logfile", type=Path, help="The path to the log file.")
    id_list_group = arg_parser.add_mutually_exclusive_group()
    id_list_group.add_argument(
        "--blacklist",
        nargs="+",
        type=lambda x: int(x, 0),
        help="The list of message IDs to ignore when logging.",
        default=[],
    )
    id_list_group.add_argument(
        "--whitelist", nargs="+", type=lambda x: int(x, 0), help="The list of message IDs to listen to.", default=[]
    )
    id_list_group.add_argument("--log", action="store_true", help="Log all CAN messages.")
    args = arg_parser.parse_args()

    # Check if the log file exists.
    if not args.logfile.exists():
        logging.error("The log file %s does not exist.", args.logfile)
        exit(1)

    can_bus = get_can_bus()

    # Setup and start the log thread if needed. If we want to log then we will set stop_event, so we can use stop_event
    # not being None to know if we want to log or not.
    log_thread = None
    stop_event = threading.Event() if args.log or len(args.blacklist) > 0 or len(args.whitelist) > 0 else None
    if stop_event is not None:
        log_thread = log_all_can_messages(stop_event, args.blacklist, args.whitelist)

    # Replay the log file.
    logging.info("Replaying CAN messages from file...")
    replay_log(can_bus, args.logfile)
    logging.info("Replaying finished.")

    if stop_event is not None:
        stop_event.set()
        log_thread.join(1)

    can_bus.shutdown()
