from __future__ import annotations

import json
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from homeassistant.components.sensor import (
    SensorEntity,
    SensorEntityDescription,
    SensorStateClass,
)
from homeassistant.const import (
    EntityCategory
)
from homeassistant.core import HomeAssistant
from homeassistant.helpers.entity_platform import AddEntitiesCallback
from homeassistant.helpers.typing import StateType

from vacuum_map_parser_base.map_data import MapData
from .entity import XiaomiCloudMapExtractorEntity, as_list_dict
from .coordinator import XiaomiCloudMapExtractorDataUpdateCoordinator
from .types import XiaomiCloudMapExtractorConfigEntry


@dataclass(frozen=True, kw_only=True)
class XiaomiCloudMapExtractorSensorEntityDescription(SensorEntityDescription):
    value_fn: Callable[[MapData], StateType]
    attributes_fn: Callable[[MapData], dict[str, Any]] = lambda _: {}

"""
      # - carpet_map
      # - charger
      # - cleaned_rooms
      # - country
      # - goto
      - goto_path
      - goto_predicted_path
      - image
      - is_empty
      - map_name
      - mop_path
      # - no_carpet_areas
      # - no_go_areas
      # - no_mopping_areas
      - obstacles
      - ignored_obstacles
      - obstacles_with_photo
      - ignored_obstacles_with_photo
      - path
      # - room_numbers
      # - rooms
      # - vacuum_position
      # - vacuum_room
      # - vacuum_room_name
      - walls
      - zones
"""


SENSOR_TYPES: tuple[XiaomiCloudMapExtractorSensorEntityDescription, ...] = (
    XiaomiCloudMapExtractorSensorEntityDescription(
        key="no_go_areas",
        translation_key="no_go_areas",
        suggested_display_precision=0,
        value_fn=lambda map_data: len(map_data.no_go_areas or []),
        attributes_fn=lambda map_data: {"areas": as_list_dict(map_data.no_go_areas)},
        state_class=SensorStateClass.MEASUREMENT,
        entity_category=EntityCategory.DIAGNOSTIC,
        entity_registry_enabled_default=False,
        entity_registry_visible_default=False,
    ),
    XiaomiCloudMapExtractorSensorEntityDescription(
        key="charger_position",
        translation_key="charger_position",
        value_fn=lambda map_data: json.dumps(map_data.charger.as_dict()) if map_data.charger else None,
        attributes_fn=lambda map_data: map_data.charger.as_dict() if map_data.charger else {},
        entity_category=EntityCategory.DIAGNOSTIC,
        entity_registry_enabled_default=False,
        entity_registry_visible_default=False,
    ),
    XiaomiCloudMapExtractorSensorEntityDescription(
        key="vacuum_position",
        translation_key="vacuum_position",
        value_fn=lambda map_data: json.dumps(map_data.vacuum_position.as_dict()) if map_data.vacuum_position else None,
        attributes_fn=lambda map_data: map_data.vacuum_position.as_dict() if map_data.vacuum_position else {},
        entity_category=EntityCategory.DIAGNOSTIC,
        entity_registry_enabled_default=False,
        entity_registry_visible_default=False,
    ),
    XiaomiCloudMapExtractorSensorEntityDescription(
        key="vacuum_room_id",
        translation_key="vacuum_room_id",
        value_fn=lambda map_data: map_data.vacuum_room,
        entity_category=EntityCategory.DIAGNOSTIC,
        entity_registry_enabled_default=False,
        entity_registry_visible_default=False,
    ),
    XiaomiCloudMapExtractorSensorEntityDescription(
        key="vacuum_room_name",
        translation_key="vacuum_room_name",
        value_fn=lambda map_data: map_data.vacuum_room_name,
        entity_category=EntityCategory.DIAGNOSTIC,
        entity_registry_enabled_default=False,
        entity_registry_visible_default=False,
    ),
    XiaomiCloudMapExtractorSensorEntityDescription(
        key="no_carpet_areas",
        translation_key="no_carpet_areas",
        suggested_display_precision=0,
        value_fn=lambda map_data: len(map_data.no_carpet_areas or []),
        attributes_fn=lambda map_data: {"areas": as_list_dict(map_data.no_carpet_areas)},
        state_class=SensorStateClass.MEASUREMENT,
        entity_category=EntityCategory.DIAGNOSTIC,
        entity_registry_enabled_default=False,
        entity_registry_visible_default=False,
    ),
    XiaomiCloudMapExtractorSensorEntityDescription(
        key="no_mopping_areas",
        translation_key="no_mopping_areas",
        suggested_display_precision=0,
        value_fn=lambda map_data: len(map_data.no_mopping_areas or []),
        attributes_fn=lambda map_data: {"areas": as_list_dict(map_data.no_mopping_areas)},
        state_class=SensorStateClass.MEASUREMENT,
        entity_category=EntityCategory.DIAGNOSTIC,
        entity_registry_enabled_default=False,
        entity_registry_visible_default=False,
    ),
    XiaomiCloudMapExtractorSensorEntityDescription(
        key="cleaned_rooms_ids",
        translation_key="cleaned_rooms_ids",
        suggested_display_precision=0,
        value_fn=lambda map_data: len(map_data.cleaned_rooms or []),
        attributes_fn=lambda map_data: {"rooms_ids": as_list_dict(map_data.cleaned_rooms)},
        state_class=SensorStateClass.MEASUREMENT,
        entity_category=EntityCategory.DIAGNOSTIC,
        entity_registry_enabled_default=False,
        entity_registry_visible_default=False,
    ),
    XiaomiCloudMapExtractorSensorEntityDescription(
        key="goto_position",
        translation_key="goto_position",
        value_fn=lambda map_data: json.dumps(map_data.goto.as_dict()) if map_data.goto else None,
        attributes_fn=lambda map_data: map_data.goto.as_dict() if map_data.goto else {},
        entity_category=EntityCategory.DIAGNOSTIC,
        entity_registry_enabled_default=False,
        entity_registry_visible_default=False,
    ),
)


async def async_setup_entry(
        hass: HomeAssistant,
        config_entry: XiaomiCloudMapExtractorConfigEntry,
        async_add_entities: AddEntitiesCallback,
) -> None:
    coordinator = config_entry.runtime_data.coordinator

    async_add_entities(
        XiaomiCloudMapExtractorSensorEntity(coordinator, config_entry, description)
        for description in SENSOR_TYPES
    )

class XiaomiCloudMapExtractorSensorEntity(XiaomiCloudMapExtractorEntity, SensorEntity):
    entity_description: XiaomiCloudMapExtractorSensorEntityDescription

    def __init__(
            self,
            coordinator: XiaomiCloudMapExtractorDataUpdateCoordinator,
            config_entry: XiaomiCloudMapExtractorConfigEntry,
            description: XiaomiCloudMapExtractorSensorEntityDescription,
    ) -> None:
        super().__init__(coordinator, config_entry)

        self._attr_unique_id = description.key
        self.entity_description = description

    @property
    def native_value(self) -> StateType:
        if (map_data := self._map_data()) is None:
            return None
        return self.entity_description.value_fn(map_data)

    @property
    def extra_state_attributes(self) -> dict[str, Any]:
        attrs = super().extra_state_attributes
        if (map_data := self._map_data()) is None:
            return attrs
        return {**attrs, **self.entity_description.attributes_fn(map_data)}
