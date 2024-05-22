from fastapi import APIRouter
from pydantic import BaseModel

from src.config import config


class ConfigUpdate(BaseModel):
    """Model for updating configuration."""

    key: str
    value: str | int | float | bool


def create_config_router() -> APIRouter:
    """Create router for config update."""
    router = APIRouter()

    @router.get("/get-config")
    def get_config() -> dict:
        """Retrieve the current configuration."""
        return config.config_dict()

    @router.post("/update-config")
    def update_config(config_update: ConfigUpdate) -> None:
        """Update a configuration key with a new value."""
        config.update_nested_key(config_update.key, config_update.value)

    @router.get("/get-config-structure")
    def get_config_structure() -> dict:
        """Get the structure of the configuration."""
        return config.get_config_structure()


    return router
