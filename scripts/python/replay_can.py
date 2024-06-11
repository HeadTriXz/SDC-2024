import argparse
import logging
import threading

from src.driving.can import get_can_bus, replay_log
from src.constants import CANControlIdentifier, CANFeedbackIdentifier


def log_all_can_messages(stopped: threading.Event, blacklist: list[int], whitelist: list[int]) -> threading.Thread:
    """Log all messages of the CAN bus."""
    listen_can = get_can_bus()

    # create and apply filters to listen to only the messages we want to log
    all_ids = [int(v) for v in CANControlIdentifier] + [int(v) for v in CANFeedbackIdentifier]
    blacklist = set(blacklist)

    if len(whitelist) > 0:
        logging.info(f"Listening to messages with IDs: {whitelist}")
        blacklist = set(all_ids) - set(whitelist)

    logging.info(f"Listening to messages with IDs: {whitelist}")
    listen_can.set_filters(
        [{"can_id": can_id, "can_mask": 0xFFF, "extended": False} for can_id in set(all_ids) - set(blacklist)]
    )

    def __listen():
        while not stopped.is_set():
            message = listen_can.recv()
            print(message)

        listen_can.shutdown()

    thread = threading.Thread(target=__listen, daemon=True)
    return thread


if __name__ == "__main__":
    """Replay CAN messages from a file and log to a file."""
    logging.basicConfig(level=logging.INFO)

    arg_parser = argparse.ArgumentParser(description="Replay CAN messages from a file.")
    arg_parser.add_argument("logfile", type=str, help="The path to the log file.")
    arg_parser.add_argument(
        "--blacklist", nargs="+", type=int, help="The list of message IDs to ignore when logging.", default=[]
    )
    arg_parser.add_argument(
        "--whitelist", nargs="+", type=int, help="The list of message IDs to listen to.", default=[]
    )
    arg_parser.add_argument("--log", action="store_true", help="Log all CAN messages.")
    args = arg_parser.parse_args()

    can_bus = get_can_bus()

    log_thread = None
    stop_event = None if args.log or len(args.blacklist) > 0 or len(args.whitelist) > 0 else threading.Event()
    if stop_event is not None:
        log_thread = log_all_can_messages(stop_event, args.blacklist, args.whitelist)
        log_thread.start()

    logging.info("Replaying CAN messages from file...")
    replay_log(can_bus, args.logfile)
    logging.info("Replaying finished.")

    if stop_event is not None:
        stop_event.set()
        log_thread.join(1)

    can_bus.shutdown()
