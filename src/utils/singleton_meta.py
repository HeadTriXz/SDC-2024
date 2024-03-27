class SingletonMeta(type):
    """A metaclass that creates a Singleton base class when called."""

    _instances = {}

    def __call__(cls, *args, **kwargs):  # noqa: ANN204, ANN002, ANN003
        """Create a new instance of the class if it doesn't already exist.

        :param cls: The class to create an instance of.
        :param args: The arguments to pass to the class constructor
        :param kwargs: The keyword arguments to pass to the class constructor
        :return: The instance of the class.
        """
        if cls not in cls._instances:
            instance = super().__call__(*args, **kwargs)
            cls._instances[cls] = instance
        return cls._instances[cls]
