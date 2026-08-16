import base64
import binascii
import json
import logging
import math
import struct
import zlib
from dataclasses import dataclass, replace
from typing import Self, Any

from miio.exceptions import DeviceException
from miio.miot_device import MiotDevice
from vacuum_map_parser_base.config.drawable import Drawable
from vacuum_map_parser_base.image_generator import ImageGenerator
from vacuum_map_parser_base.map_data import MapData, Path, Point
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

# One record of the in-session path object: a mode flag, then the coordinates
# as little-endian int32 millimetres in the map's own frame.
_SESSION_PATH_RECORD = struct.Struct("<Bii")

# Layers this connector fills in after the parser has already drawn the map, and
# which therefore have to be painted in a second pass.
_LATE_DRAWABLES = (Drawable.VACUUM_POSITION, Drawable.PATH)

# How far outside every room the vacuum may sit and still be attributed to the
# closest one. Room bounds are rounded to the pixel, so a vacuum against a wall
# can fall just outside them all -- on the reference map the dock itself misses
# its own room by 15 mm.
_ROOM_MATCH_TOLERANCE = 300

# A full turn in milliradians, the ceiling of any heading on models using them.
_MAX_MILLIRADIANS = 2000 * math.pi

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

    # Property id naming the in-session path object, for models whose map
    # carries no "paths" field. On b108gl `piid` names the persisted map
    # (object .../3, map_type 1) while this one names object .../1, a plain
    # base64+zlib stream of 9-byte records that holds the cleaning path.
    # It is neither AES-encrypted nor JSON, so it must not go through
    # `unpack_map`; `_parse_session_path` decodes it instead.
    session_path_piid: int | None = None

    # Whether this model expresses headings in milliradians. On b108gl the dock
    # reports 1570, which is pi/2 x 1000, and every reading stays below
    # 2 pi x 1000. XiaomiMapDataParser reads such values as centi-degrees, so
    # both the live position and the charger need converting.
    yaw_in_milliradians: bool = False

_NON_STANDARD_MAP_PROP = [
    (
        [
            "xiaomi.vacuum.b108gl",
        ],
        XiaomiVacuumPropertyMapping(
            siid=7, position_piid=4, session_path_piid=2, yaw_in_milliradians=True
        ),
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

        self._status_mapping = _STATUS_MAPPING_OVERRIDES.get(self.model) or get_status_mapping(self.model)
        self._off_counter = 0

        self._vacuum_map = next((mapping for models, mapping in _NON_STANDARD_MAP_PROP if self.model in models), XiaomiVacuumPropertyMapping())

        # XiaomiMapDataParser.parse() ends by drawing the map, so anything this
        # connector completes afterwards would never reach the image. Those layers
        # are taken away from the parser and painted in a second pass instead, so
        # each one is still drawn exactly once.
        #
        # The charger only moves across when its heading has to be corrected.
        #
        # Nothing moves across at all on a rotated map: the parser turns the
        # raster as its last act, while Point.to_img() maps coordinates in the
        # unrotated frame, so a late pass would draw in the wrong place. The
        # parser then keeps every layer and behaves exactly as before -- it
        # simply has no telemetry to draw.
        self._image_rotation = vacuum_config.image_config.rotate
        late: list[Drawable] = []
        if self._image_rotation:
            _LOGGER.debug("Map is rotated by %s, drawing everything in the parser",
                          self._image_rotation)
        else:
            late = [d for d in vacuum_config.drawables if d in _LATE_DRAWABLES]
            if self._vacuum_map.yaw_in_milliradians and Drawable.CHARGER in vacuum_config.drawables:
                late.append(Drawable.CHARGER)
        self._late_drawables = late

        self._xiaomi_map_data_parser = XiaomiMapDataParser(
            vacuum_config.palette,
            vacuum_config.sizes,
            [d for d in vacuum_config.drawables if d not in late],
            vacuum_config.image_config,
            vacuum_config.texts
        )
        self._late_image_generator = ImageGenerator(
            vacuum_config.palette,
            vacuum_config.sizes,
            late,
            replace(vacuum_config.image_config, rotate=0),
            [],
        )

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
    
    @staticmethod
    def _object_name(response: Any) -> str | None:
        """Extract the trailing object name from a vacuum map service property.

        Those properties name a cloud object in one of three shapes: a bare
        integer, a plain "<user>/<device>/<name>" string, or a JSON object
        carrying that string under "obj_name". Only the trailing segment is
        needed, since get_map_url composes the full path again.
        """
        if isinstance(response, int):
            return str(response)
        if not isinstance(response, str):
            return None
        try:
            name = json.loads(response).get("obj_name")
        except (json.JSONDecodeError, AttributeError):
            name = response if "/" in response else None
        return name.split("/")[-1] if name else None

    async def get_map_name(self: Self) -> str:
        response = self._miot_device.get_property_by(self._vacuum_map.siid,
                                                     self._vacuum_map.piid)[0].get("value")

        if response is None:
            return await super().get_map_name()

        map_name = self._object_name(response)
        if map_name is None:
            return await super().get_map_name()
        return map_name

    async def get_map_url(self, map_name: str) -> str | None:
        return await self.get_fallback_map_url(map_name)

    async def get_map(self: Self) -> tuple[MapData, bytes]:
        map_data, raw_map_data = await super().get_map()
        self._fix_charger_heading(map_data)
        self._apply_realtime_position(map_data)
        self._apply_vacuum_room(map_data)
        await self._apply_session_path(map_data)
        self._draw_late_overlays(map_data)
        return map_data, raw_map_data

    @staticmethod
    def _apply_vacuum_room(map_data: MapData) -> None:
        """Work out which room the vacuum stands in, from the room bounds.

        XiaomiImageParser.get_current_vacuum_room() would answer this per pixel,
        but it is never called by the parser and needs the decoded pixel grid,
        which MapData does not keep. The room bounding boxes it does keep are
        enough here: on the reference map only two of six rooms overlap, by one
        pixel each, so the answer is unambiguous almost everywhere.

        Where boxes do overlap the smallest one wins, being the most specific.
        A vacuum outside every box falls back to the closest room within
        _ROOM_MATCH_TOLERANCE, which covers the pixel rounding of the bounds.
        """
        if map_data.vacuum_position is None or not map_data.rooms:
            return
        if map_data.vacuum_room is not None:
            return

        x, y = map_data.vacuum_position.x, map_data.vacuum_position.y
        inside, inside_area = None, None
        closest, closest_distance = None, None

        for room in map_data.rooms.values():
            gap_x = max(room.x0 - x, 0, x - room.x1)
            gap_y = max(room.y0 - y, 0, y - room.y1)
            distance = math.hypot(gap_x, gap_y)
            area = abs(room.x1 - room.x0) * abs(room.y1 - room.y0)
            if distance == 0 and (inside_area is None or area < inside_area):
                inside, inside_area = room, area
            if closest_distance is None or distance < closest_distance:
                closest, closest_distance = room, distance

        room = inside
        if room is None and closest_distance is not None and closest_distance <= _ROOM_MATCH_TOLERANCE:
            room = closest
            _LOGGER.debug("Vacuum is %.0f mm outside every room, closest wins", closest_distance)
        if room is None:
            _LOGGER.debug("Vacuum at %.0f,%.0f matches no room", x, y)
            return

        map_data.vacuum_room = room.number
        # An unnamed room reports no name rather than an empty one; the room id
        # sensor still identifies it.
        map_data.vacuum_room_name = room.name or None
        _LOGGER.debug("Vacuum room: %s (%s)", map_data.vacuum_room, map_data.vacuum_room_name)

    def _fix_charger_heading(self: Self, map_data: MapData) -> None:
        """Restore the charger heading on models reporting milliradians.

        _json_yaw_to_degrees treats anything above 180 as centi-degrees, so a
        dock at 1570 milliradians -- pi/2 x 1000, due north -- is reported as
        15.7 degrees, and its half-disc is drawn pointing the wrong way.

        The original value is recoverable exactly: headings never exceed
        2 pi x 1000, so the parser's `% 180` fold can never have fired and its
        output is simply the reading divided by 100.
        """
        charger = map_data.charger
        if not self._vacuum_map.yaw_in_milliradians or charger is None or charger.a is None:
            return
        if not 0 <= charger.a <= _MAX_MILLIRADIANS / 100:
            _LOGGER.debug("Charger heading %s is not a folded milliradian reading", charger.a)
            return
        charger.a = math.degrees(charger.a * 100 / 1000) % 360
        _LOGGER.debug("Charger heading corrected to %.1f degrees", charger.a)

    def _draw_late_overlays(self: Self, map_data: MapData) -> None:
        """Paint the layers filled in after the parser drew the map.

        The parser draws as the last step of parse(), so the position and path
        fetched afterwards are in MapData but absent from the image. Only the
        missing layers are painted, over the image the parser produced.

        Skipped when a rotation is configured: the parser has already turned the
        raster, while Point.to_img() maps coordinates in the unrotated frame, so
        anything drawn now would land in the wrong place. Fixing that belongs
        upstream, where the telemetry could be supplied before the map is drawn.
        """
        if not self._late_drawables:
            return
        if map_data.image is None or map_data.image.is_empty:
            return
        try:
            self._late_image_generator.draw_map(map_data)
        except Exception as err:  # noqa: BLE001 - a missing overlay must never break the map
            _LOGGER.debug("Could not draw vacuum and path overlays: %s", err)

    async def _apply_session_path(self: Self, map_data: MapData) -> None:
        """Fill in the cleaning path from the in-session path object.

        The map this integration downloads is a persisted one carrying no
        "paths" field, so the parser leaves MapData.path empty and no trace is
        drawn. Models declaring `session_path_piid` name a second cloud object
        holding the path, fetched and decoded here.

        Never raises: any failure leaves the path empty and the map is rendered
        without a trace.
        """
        if self._vacuum_map.session_path_piid is None or map_data.path is not None:
            return

        try:
            value = self._miot_device.get_property_by(
                self._vacuum_map.siid, self._vacuum_map.session_path_piid
            )[0].get("value")
        except DeviceException as de:
            _LOGGER.debug("Could not read session path property: %s", de)
            return

        object_name = self._object_name(value)
        if object_name is None:
            _LOGGER.debug("No session path object named by the property: %r", value)
            return

        try:
            raw_path = await self.get_raw_map_data(object_name)
        except Exception as err:  # noqa: BLE001 - a missing trace must never break the map
            _LOGGER.debug("Could not download session path %r: %s", object_name, err)
            return
        if raw_path is None:
            _LOGGER.debug("Session path object %r returned no data", object_name)
            return

        map_data.path = self._parse_session_path(raw_path)

    @staticmethod
    def _parse_session_path(raw: bytes) -> Path | None:
        """Decode the in-session path object into a Path.

        Layout, confirmed against a real run of 1234 points: ASCII base64 of a
        zlib stream, inflating to a whole number of 9-byte records, each one a
        uint8 flag followed by two little-endian int32 coordinates in the same
        millimetre frame as the map.

        The flag marks a mode rather than a break, so every point goes into a
        single sub-path: measured over that run, consecutive points sit 69 mm
        apart in median and 79 mm across a flag change, with no gap wider than
        256 mm. Splitting on it would fragment a continuous trace for nothing.
        """
        try:
            blob = zlib.decompress(base64.b64decode(raw, validate=True))
        except (binascii.Error, ValueError, zlib.error) as err:
            _LOGGER.debug("Session path is not base64-encoded zlib: %s", err)
            return None

        if not blob or len(blob) % _SESSION_PATH_RECORD.size:
            _LOGGER.debug(
                "Session path is %d bytes, not a whole number of %d-byte records",
                len(blob), _SESSION_PATH_RECORD.size,
            )
            return None

        points = [
            Point(float(x), float(y))
            for _flag, x, y in _SESSION_PATH_RECORD.iter_unpack(blob)
        ]
        _LOGGER.debug("Decoded session path: %d point(s)", len(points))
        return Path(None, None, None, [points])

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
        # Reports the telemetry the downloaded map itself carried, before this
        # connector fills in whatever the model publishes elsewhere.
        _LOGGER.debug(
            "Parsed telemetry: vacuum_position=%s, path=%d point(s), mop_path=%d point(s)",
            map_data.vacuum_position,
            sum(len(sub) for sub in map_data.path.path) if map_data.path else 0,
            sum(len(sub) for sub in map_data.mop_path.path) if map_data.mop_path else 0,
        )
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
        if len(position) > 2 and self._vacuum_map.yaw_in_milliradians:
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
