import os
import time
import yaml

from pathlib import Path


class ConfigLoader:
    """The configuration loader."""

    def __init__(self, environment: str | None = None) -> None:
        """Initialize the configuration loader."""
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

        # Load without using OmegaConf
        self.__default_config = self.__load_yaml(default_path)
        self.__environment_config = self.__load_yaml(environment_path)
        self.__loaded_config = self.__merge_dicts(self.__default_config, self.__environment_config)

    def __load_yaml(self, path: Path) -> dict:
        with open(path) as file:
            return yaml.safe_load(file)

    def __merge_dicts(self, default: dict, environment: dict) -> dict:
        result = default.copy()
        for key, value in environment.items():
            if isinstance(value, dict):
                result[key] = self.__merge_dicts(result.get(key, {}), value)
            else:
                result[key] = value
        return result

    def diff_dicts(self, loaded_dict: dict, default_dict: dict) -> dict:
        """Get the difference between two dictionaries."""
        diff = {}
        for key, value in loaded_dict.items():
            if key not in default_dict:
                diff[key] = value
            elif isinstance(value, dict):
                nested_diff = self.diff_dicts(value, default_dict[key])
                if nested_diff:
                    diff[key] = nested_diff
            elif value != default_dict[key]:
                diff[key] = value
        return diff

    def get_config_structure(self, config: dict | None = None) -> dict:
        """Get the structure of the config."""
        if config is None:
            config = self.__loaded_config

        structure = {}
        for key, value in config.items():
            if isinstance(value, dict):
                structure[key] = self.get_config_structure(value)
            else:
                structure[key] = type(value).__name__

        return structure

    def update_nested_key(self, key: str, value: str | int | float | bool) -> None:
        """Update a nested key in the configuration."""
        keys = key.split(".")
        current = self.__loaded_config
        for k in keys[:-1]:
            current = current[k]
        current[keys[-1]] = value

    def __get_signature(self) -> str:
        """Get the execution signature."""
        self.__signature = str(time.time()).replace(".", "")

        return self.__signature

    def config_dict(self) -> dict:
        """Get the loaded config as dict."""
        return self.__loaded_config

    def __getattr__(self, item: str) -> str | dict:
        """Get the attribute."""
        return self.__loaded_config[item]

    def __getitem__(self, item: str) -> str | dict:
        """Get the item."""
        return self.__loaded_config[item]
