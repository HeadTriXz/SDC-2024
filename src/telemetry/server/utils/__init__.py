import os


def get_file_relative_path(file: str) -> str:
    """Get the relative path of the file."""
    current_file = __file__
    current_dir = os.path.dirname(current_file)
    return os.path.join(current_dir, file)
