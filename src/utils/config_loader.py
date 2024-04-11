import os
import time

from omegaconf import DictConfig, ListConfig, OmegaConf
from pathlib import Path
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

    __config_dir: Path
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
        self.__config_dir = Path(__file__).parents[2] / "configs"
        self.__load_config()

    def __load_config(self) -> None:
        default_path = self.__config_dir / "config.defaults.yaml"
        environment_path = self.__config_dir / f"config.{self.__environment}.yaml"

        if not environment_path.exists():
            raise FileNotFoundError(f"Configuration file not found at {environment_path}")

        self.__default_config = OmegaConf.load(default_path)
        self.__environment_config = OmegaConf.load(environment_path)
        self.__loaded_config = OmegaConf.merge(self.__default_config, self.__environment_config)

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
        self.__signature = str(time.time()).replace(".", "")

        return self.__signature

    def save_changes(self, backup: bool = True) -> None:
        """Save the changes.

        This function will save the changes to the configuration file.
        If backup is set to True, it will create a backup of the original file.

        :param backup: whether to create a backup of the original file.
        """
        file_path = self.__config_dir / f"config.{self.__environment}.yaml"

        if backup:
            backup_file_name = f"{self.__get_signature()}-config.{self.__environment}.yaml"
            backup_path = self.__config_dir / "backups" / backup_file_name
            backup_path.parent.mkdir(parents=True, exist_ok=True)

            # Move the original file to the backup location
            file_path.rename(backup_path)

        # get the differences between the loaded config and the default config and store them
        diff = self.get_changes()
        OmegaConf.save(diff, file_path)

    def get_changes(self) -> dict:
        """Get the changes.

        This function will get the changes between the loaded configuration and the default configuration.

        :return: the changes between the loaded configuration and the default configuration.
        """
        loaded_dict = OmegaConf.to_container(self.__loaded_config)
        default_dict = OmegaConf.to_container(self.__default_config)

        return self.__diff_dicts(loaded_dict, default_dict)

    def __diff_dicts(self, loaded_dict: dict, default_dict: dict) -> dict:
        """Get the difference between two dictionaries.

        This function will get the difference between two dictionaries.

        :param loaded_dict: the loaded dictionary.
        :param default_dict: the default dictionary.
        :return: the difference between the two dictionaries.
        """
        diff = {}
        for key, value in loaded_dict.items():
            if key not in default_dict:
                diff[key] = value
            elif isinstance(value, dict):
                nested_diff = self.__diff_dicts(value, default_dict[key])
                if nested_diff:
                    diff[key] = nested_diff
            elif value != default_dict[key]:
                diff[key] = value
        return diff

    def rollback(self) -> None:
        """Rollback to the latest backup."""
        backup_path = self.__config_dir / "backups"
        backups = list(backup_path.glob("*.yaml"))

        if not backups:
            raise FileNotFoundError("No backups found.")

        backups.sort(reverse=True)
        latest_path = backup_path / backups[0]

        # Delete the original file
        original_file = self.__config_dir / f"config.{self.__environment}.yaml"
        original_file.unlink(missing_ok=True)

        # Move the latest backup to the original file location
        latest_path.rename(original_file)

        # Reload the configuration
        self.__load_config()

    def get_config_structure(self, config: dict | None = None) -> dict:
        """Get the structure of the config.

        :param config: The configuration dictionary to analyze.
        :return: A dictionary representing the structure of the configuration.
        """
        if config is None:
            config = OmegaConf.to_container(self.__loaded_config, resolve=False)

        structure = {}
        for key, value in config.items():
            if isinstance(value, dict):
                structure[key] = self.get_config_structure(value)
            else:
                structure[key] = type(value).__name__

        return structure

    def config_dict(self) -> dict:
        """Get the loaded config as dict.

        :return: The configuration dictionary.
        """
        return OmegaConf.to_container(self.__loaded_config, resolve=False)

    def update_nested_key(self, key: str, value: str | int | float | bool) -> None:
        """Update a nested key in the configuration.

        This function updates a nested key in the configuration.

        :param key: The key to update.
        :param value: The new value.
        """
        keys = key.split(".")
        current = self.__loaded_config
        for k in keys[:-1]:
            current = current[k]
        current[keys[-1]] = value
