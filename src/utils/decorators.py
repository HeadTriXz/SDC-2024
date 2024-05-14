from typing import Any, Callable


def check_if(prop: str) -> Callable:
    """Check if a property is true before calling a function.

    :param prop: The property to check.
    :return: The wrapper function.
    """
    def decorator(func: Callable) -> Callable:
        def wrapper(self: object, *args: Any, **kwargs: Any) -> Any:
            if getattr(self, prop):
                return func(self, *args, **kwargs)

            return None

        return wrapper

    return decorator
