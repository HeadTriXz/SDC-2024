import os
import time

from omegaconf import DictConfig, ListConfig, OmegaConf

from utils.singleton_meta import SingletonMeta


class ConfigLoader(metaclass=SingletonMeta):
    """The configuration loader.

    This class will load the configuration file.
    If no file is supplied, it will look at the `environment` variable ENVIRONMENT.
    If this isn't found, it will default to development configuration.

    the config also supports interpolation, so you can use `${}` to reference other values in the configuration. it
    supports both absolute and relative references. these references are updated when the value is accessed.
    for more information see https://omegaconf.readthedocs.io/en/2.3_branch/usage.html#variable-interpolation

    Examples
    --------
    ```python
    config = ConfigLoader()
    config.speed_modes.selected = "${.options.fast}" # set the selected speed mode to the fast option
    config.speed_modes.options.fast = 100 # set the fast option to 100

    print(config.speed_modes.selected) # prints 100.


    config.save_changes() # save the changes to the configuration file without making a backup
    config.save_changes(True) # save the changes to the configuration file and make a backup
    config.rollback() # rollback to the previous configuration
    ```

    """

    __loaded_config: DictConfig = None
    __signature: str | None = None

    def __init__(self, environment: str | None = None) -> None:
        """Initialize the configuration loader.

        This function will initialize the configuration loader.

        :param environment: the environment to load the configuration for.
        """
        if environment is None:
            environment = os.environ.get("ENVIRONMENT", "development")

        self.__environment = environment

        path = os.path.join(os.path.dirname(__file__), f"config.{environment}.yaml")
        if not os.path.exists(path):
            raise FileNotFoundError(f"Configuration file not found at {path}")

        defaults = OmegaConf.load(os.path.join(os.path.dirname(__file__), "config.defaults.yaml"))
        environment_config = OmegaConf.load(path)
        merged_config = OmegaConf.merge(defaults, environment_config)
        self.__loaded_config = merged_config


    def __getattr__(self, item: str) -> str | DictConfig | ListConfig:
        """Get the attribute.

        This function will get the attribute.

        :param item: the item to get.
        :return: the item.
        """
        return self.__loaded_config[item]

    def __get_signature(self) -> str:
        """Get the execution signature.

        This function will get the execution signature.

        :return: the execution signature.
        """
        if self.__signature is None:
            self.__signature = str(time.time()).replace(".", "")

        return self.__signature

    def save_changes(self, backup: bool = True) -> None:
        """Save the changes.

        This function will save the changes to the configuration file.
        If backup is set to True, it will create a backup of the original file.

        :param backup: whether to create a backup of the original file.
        """
        config_file_name = f"config.{self.__environment}.yaml"
        file_path = os.path.join(os.path.dirname(__file__), config_file_name)

        if backup:
            backup_file_name = f"{self.__get_signature()}-config.{self.__environment}.yaml"
            backup_path = os.path.join(os.path.dirname(__file__), "backups", backup_file_name)

            if not os.path.exists(os.path.join(os.path.dirname(__file__), "backups")):
                os.makedirs(os.path.join(os.path.dirname(__file__), "backups"))

            # move the original file to the backup location
            os.rename(os.path.join(os.path.dirname(__file__), f"config.{self.__environment}.yaml"), backup_path)

        OmegaConf.save(self.__loaded_config, file_path)

    def rollback(self) -> None:
        """Rollback to the latest backup."""
        backup_path = os.path.join(os.path.dirname(__file__), "backups")
        backups = os.listdir(backup_path)

        if not backups:
            raise FileNotFoundError("No backups found.")

        backups.sort(reverse=True)
        latest_backup = backups[0]

        # delete the original file
        os.remove(os.path.join(os.path.dirname(__file__), f"config.{self.__environment}.yaml"))

        # move the latest backup to the original file location
        os.rename(
            os.path.join(backup_path, latest_backup),
            os.path.join(os.path.dirname(__file__), f"config.{self.__environment}.yaml"),
        )

        # reload the configuration
        self.__loaded_config = OmegaConf.load(
            os.path.join(os.path.dirname(__file__), f"config.{self.__environment}.yaml")
        )
