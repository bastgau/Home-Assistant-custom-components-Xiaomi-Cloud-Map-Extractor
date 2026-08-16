import base64
import json
import logging
import math
from dataclasses import dataclass
from typing import Self, Any

from miio.exceptions import DeviceException
from miio.miot_device import MiotDevice
from vacuum_map_parser_base.map_data import MapData, Point
from vacuum_map_parser_xiaomi.aes_decryptor import gen_md5_key
from vacuum_map_parser_xiaomi.map_data_parser import XiaomiMapDataParser
from vacuum_map_parser_xiaomi.status_mapping import XiaomiVacuumStatusMapping, get_status_mapping

from .base.model import VacuumConfig, VacuumApi
from .base.vacuum_v2 import BaseXiaomiCloudVacuumV2
from ..utils.exceptions import FailedConnectionException

_LOGGER = logging.getLogger(__name__)
OFF_UPDATES = 3

# Status mappings that vacuum-map-parser-xiaomi does not carry yet. Its default
# idle_at tuple is (0, 1, 2, 4, 8, 10) and applies to every model but e101gb.
#
# xiaomi.vacuum.b108gl (Xiaomi Robot Vacuum S20+), values read from the device:
#   1 = stopped mid-run       (idle)      2 = docked, charging      (idle)
#   4 = sweeping              (ACTIVE)    5 = task suspended        (idle)
#   6 = returning to the dock (ACTIVE)    8 = docked, not charging  (idle)
# The default tuple contains 4, so should_update_map declares the vacuum idle
# while it is sweeping and the map stops being downloaded after OFF_UPDATES.
#
# 0 and 10 were never observed and are deliberately left out: a value missing
# from idle_at is treated as active, which merely costs a needless refresh,
# whereas wrongly listing an active value freezes the map.
_STATUS_MAPPING_OVERRIDES: dict[str, XiaomiVacuumStatusMapping] = {
    "xiaomi.vacuum.b108gl": XiaomiVacuumStatusMapping(idle_at=(1, 2, 5, 8)),
}

@dataclass
class XiaomiVacuumPropertyMapping:
    """Dataclass containing mapping for map property"""

    # vacuum map service id
    siid: int = 10

    # current map property id in vacuum map service
    piid: int = 1

    # Realtime position property id in the vacuum map service, for models whose
    # downloaded map carries no telemetry. Its value is a JSON object
    # {"position": [x, y, yaw]}, with x and y in the same millimetre frame as
    # the map, and yaw in milliradians.
    position_piid: int | None = None

    # EXPERIMENTAL. Property id naming the in-session map object, when the model
    # publishes one besides the persisted map named by `piid`. On b108gl `piid`
    # resolves to object .../3 -- a persisted map (map_type 1) carrying neither
    # "paths" nor "position" -- while this one resolves to .../1, an object that
    # does not appear in the saved map list and may hold the live path.
    # Leave None to keep downloading the object named by `piid`.
    session_map_piid: int | None = None

_NON_STANDARD_MAP_PROP = [
    (
        [
            "xiaomi.vacuum.b108gl",
        ],
        XiaomiVacuumPropertyMapping(siid=7, position_piid=4, session_map_piid=2),
    ),
    (
        [
            "xiaomi.vacuum.b108gp",
            "xiaomi.vacuum.ov32gl",
            "xiaomi.vacuum.ov43gl",
            "xiaomi.vacuum.ov51",
            "xiaomi.vacuum.ov81",
        ],
        XiaomiVacuumPropertyMapping(siid=9),
    ),
    (
        [
            "xiaomi.vacuum.b106bk",
            "xiaomi.vacuum.b106tr",
            "xiaomi.vacuum.b112",
            "xiaomi.vacuum.b112bk",
            "xiaomi.vacuum.b112gl",
            "xiaomi.vacuum.b112tr",
            "xiaomi.vacuum.c101",
            "xiaomi.vacuum.c101eu",
            "xiaomi.vacuum.c102",
            "xiaomi.vacuum.c104",
            "xiaomi.vacuum.e101gl",
        ],
        XiaomiVacuumPropertyMapping(piid=2),
    ),
]

class XiaomiCloudVacuum(BaseXiaomiCloudVacuumV2):
    def __init__(self, vacuum_config: VacuumConfig):
        super().__init__(vacuum_config)
        self._token = vacuum_config.token
        self._host = vacuum_config.host

        self._miot_device = MiotDevice(self._host, self._token, timeout=2)

        self._xiaomi_map_data_parser = XiaomiMapDataParser(
            vacuum_config.palette,
            vacuum_config.sizes,
            vacuum_config.drawables,
            vacuum_config.image_config,
            vacuum_config.texts
        )

        self._status_mapping = _STATUS_MAPPING_OVERRIDES.get(self.model) or get_status_mapping(self.model)
        self._off_counter = 0

        self._vacuum_map = next((mapping for models, mapping in _NON_STANDARD_MAP_PROP if self.model in models), XiaomiVacuumPropertyMapping())

    @property
    def should_update_map(self: Self) -> bool:
        try:
            status_value = self._miot_device.get_property_by(self._status_mapping.siid,
                                                             self._status_mapping.piid)[0]["value"]
            _LOGGER.debug("Vacuum status: %s (idle_at: %s)", status_value, self._status_mapping.idle_at)

            if status_value in self._status_mapping.idle_at:
                self._off_counter += 1
                _LOGGER.debug(
                    "Vacuum is not moving. Off counter: %d", self._off_counter)
                return self._off_counter <= OFF_UPDATES
            else:
                self._off_counter = 0
                return True
        except DeviceException as de:
            if "token" not in repr(de):
                _LOGGER.debug("Could not read vacuum status, skipping map update: %s", de)
                return False
            raise FailedConnectionException(de)

    @staticmethod
    def vacuum_platform() -> VacuumApi:
        return VacuumApi.XIAOMI

    @property
    def map_archive_extension(self) -> str:
        return "zlib.enc"

    @property
    def map_data_parser(self) -> XiaomiMapDataParser:
        return self._xiaomi_map_data_parser
    
    async def get_map_name(self: Self) -> str:
        piid = self._vacuum_map.session_map_piid or self._vacuum_map.piid
        response = self._miot_device.get_property_by(self._vacuum_map.siid, piid)[0].get("value")
        _LOGGER.debug("Map object from %d/%d: %r", self._vacuum_map.siid, piid, response)

        if response is None:
            return await super().get_map_name()

        if isinstance(response, int):
            return str(response)
        else:
            map_name = None
            try:
                map_name = json.loads(response).get("obj_name", None)
            except json.JSONDecodeError:
                if isinstance(response, str) and "/" in response:
                    map_name = response
            if map_name is None:
                return await super().get_map_name()
            return map_name.split("/")[-1]

    async def get_map_url(self, map_name: str) -> str | None:
        return await self.get_fallback_map_url(map_name)

    def decode_and_parse(self, raw_map: bytes) -> MapData:
        # Try parsing as JSON first (old format), otherwise use raw data directly (new format)
        try:
            raw_map = base64.decodebytes(json.loads(raw_map)["data"].encode("latin1"))
        except (json.JSONDecodeError, KeyError, UnicodeDecodeError):
            # Data may not be JSON-wrapped
            pass
        
        raw_map = raw_map.hex()
        decoded_map = self.map_data_parser.unpack_map(
            raw_map,
            model=self.model.replace("xiaomi", "mi"),
            device_id=str(self._device_id),
        )
        map_data = self.map_data_parser.parse(decoded_map)
        # EXPERIMENTAL. Reports what the downloaded object actually carried, so the
        # session map object can be compared against the persisted one without
        # having to download diagnostics.
        _LOGGER.debug(
            "Parsed telemetry: vacuum_position=%s, path=%d point(s), mop_path=%d point(s)",
            map_data.vacuum_position,
            sum(len(sub) for sub in map_data.path.path) if map_data.path else 0,
            sum(len(sub) for sub in map_data.mop_path.path) if map_data.mop_path else 0,
        )
        self._apply_realtime_position(map_data)
        return map_data

    def _apply_realtime_position(self: Self, map_data: MapData) -> None:
        """Fill in the vacuum position from a MIoT property.

        Some models only publish a persisted map (map_type 1) carrying neither a
        "position" nor a "paths" field, so the parser leaves
        MapData.vacuum_position empty and the vacuum is never drawn. Those models
        expose the live position as a separate property of the same vacuum map
        service instead.

        Does nothing when the model declares no position property or when the
        parser already found a position, so other models are unaffected.
        """
        if self._vacuum_map.position_piid is None or map_data.vacuum_position is not None:
            return

        try:
            value = self._miot_device.get_property_by(
                self._vacuum_map.siid, self._vacuum_map.position_piid
            )[0].get("value")
        except DeviceException as de:
            _LOGGER.debug("Could not read realtime position: %s", de)
            return

        if not isinstance(value, str):
            _LOGGER.debug("Unexpected realtime position value: %r", value)
            return

        try:
            position = json.loads(value).get("position")
        except (json.JSONDecodeError, AttributeError):
            _LOGGER.debug("Realtime position is not a JSON object: %r", value)
            return

        if not isinstance(position, list) or len(position) < 2:
            _LOGGER.debug("Realtime position has no usable coordinates: %r", position)
            return

        angle = None
        if len(position) > 2:
            # The third element is a heading in milliradians, kept in [0, 360).
            angle = math.degrees(float(position[2]) / 1000.0) % 360

        map_data.vacuum_position = Point(float(position[0]), float(position[1]), angle)
        _LOGGER.debug("Realtime vacuum position: %s", map_data.vacuum_position)

    def additional_data(self: Self) -> dict[str, Any]:
        super_data = super().additional_data()
        enc_key = gen_md5_key(
            self.model.replace("xiaomi", "mi"),
            str(self._device_id),
        )

        return {**super_data, "enc_key": enc_key}
