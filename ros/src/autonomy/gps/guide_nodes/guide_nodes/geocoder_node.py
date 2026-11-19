"""ROS 2 node: looks destination names up online (Nominatim), with an on-disk cache.

Serves guide/geocode (guide_interfaces/srv/Geocode). The mission manager
calls it only when the curated destination list has no match. All decisions
(cache first, off switch, rate limit, falling back to old answers) live in
guide_core.geocoder; this node reads parameters, makes the HTTP request and
copies results into ROS messages.

This is the only node that talks to the internet. It follows the public
server's usage policy: https://operations.osmfoundation.org/policies/nominatim/
(max 1 request/s, identifying User-Agent, results cached, can be switched
off at runtime, no personal data sent). Search results (c) OpenStreetMap
contributors, ODbL.
"""

import json
import os
import sys
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode, urlsplit
from urllib.request import Request, urlopen

from guide_core.destinations import match_score, normalise
from guide_core.geocode_cache import CacheConfig, DEFAULT_CACHE_FILE, GeocodeCache
from guide_core.geocoder import FetchError, Geocoder, Status
from guide_core.geocoding import (
    ATTRIBUTION, bbox_from_meta, GeocodeConfig, GeocodeError, is_large_area, labels,
    NOMINATIM_SOURCE)
from guide_core.rate_limit import RateLimitConfig, RateLimiter
from guide_interfaces.msg import DestinationCandidate
from guide_interfaces.srv import Geocode
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import yaml

DEFAULT_BASE_URL = 'https://nominatim.openstreetmap.org'
DEFAULT_USER_AGENT = 'NaviGoat-guide/0.7 (+https://github.com/FlavioGazzetta/naviGoat)'
MAX_ANSWER_BYTES = 1_000_000   # a /search answer with 5 results is a few kB


class StartupError(Exception):
    """The node cannot start (bad parameter, missing graph metadata)."""


def user_agent_problem(user_agent):
    """Return why a User-Agent is not acceptable to Nominatim, or '' if it is fine.

    The policy requires one that identifies the application; the default
    of HTTP libraries ("Python-urllib/3.12") is rejected by the server.
    """
    text = user_agent.strip()
    if not text:
        return 'user_agent is empty'
    if text.lower().startswith(('python-urllib', 'python-requests', 'curl/')):
        return f'user_agent {text!r} is a library default; it must identify NaviGoat'
    return ''


class GeocoderNode(Node):
    """Serves guide/geocode; HTTP requests go to base_url."""

    def __init__(self):
        super().__init__('geocoder')

        # --- Parameters ---
        # Defaults come from guide_core's dataclasses, so they live in one place.
        # Float parameters must be given as floats on the command line (5.0, not 5).
        geo, cache_cfg, limits = GeocodeConfig(), CacheConfig(), RateLimitConfig()
        self.declare_parameter('online_enabled', True)   # the off switch; settable at runtime
        self.declare_parameter('base_url', DEFAULT_BASE_URL)
        self.declare_parameter('user_agent', DEFAULT_USER_AGENT)
        self.declare_parameter('timeout', 5.0)            # seconds per HTTP request
        self.declare_parameter('cache_file', DEFAULT_CACHE_FILE)
        self.declare_parameter('graph_file', '')          # the area comes from its .meta.yaml
        self.declare_parameter('max_results', geo.max_results)
        self.declare_parameter('edge_margin', geo.edge_margin)
        self.declare_parameter('large_area_size', geo.large_area_size)
        self.declare_parameter('duplicate_distance', geo.duplicate_distance)
        self.declare_parameter('language', geo.language)
        self.declare_parameter('max_age_days', cache_cfg.max_age_days)
        self.declare_parameter('negative_max_age_days', cache_cfg.negative_max_age_days)
        self.declare_parameter('min_interval', limits.min_interval)
        self.declare_parameter('max_per_hour', limits.max_per_hour)
        self.declare_parameter('block_cooldown', limits.block_cooldown)

        def param(name):
            return self.get_parameter(name).value

        self._base_url = param('base_url').rstrip('/')
        self._user_agent = param('user_agent').strip()
        self._timeout = param('timeout')
        if urlsplit(self._base_url).scheme not in ('http', 'https'):
            raise StartupError(f'base_url must start with http:// or https://, '
                               f'got {self._base_url!r}')
        if not self._timeout > 0.0:
            raise StartupError(f'timeout must be > 0, got {self._timeout}')
        try:
            self._config = GeocodeConfig(
                max_results=param('max_results'), edge_margin=param('edge_margin'),
                large_area_size=param('large_area_size'),
                duplicate_distance=param('duplicate_distance'), language=param('language'))
            cache_config = CacheConfig(max_age_days=param('max_age_days'),
                                       negative_max_age_days=param('negative_max_age_days'))
            limiter = RateLimiter(RateLimitConfig(
                min_interval=param('min_interval'), max_per_hour=param('max_per_hour'),
                block_cooldown=param('block_cooldown')))
        except (ValueError, TypeError) as e:   # GeocodeError is a ValueError
            raise StartupError(f'Invalid geocoder parameter: {e}') from e

        # --- Graph area ---
        # Only the small .meta.yaml next to the graph is read (not the graph itself).
        graph_file = os.path.expanduser(param('graph_file'))
        if not graph_file:
            raise StartupError('Parameter graph_file is required (the same .graphml the '
                               'route planner uses; its .meta.yaml gives the area).')
        meta_file = os.path.splitext(graph_file)[0] + '.meta.yaml'
        try:
            with open(meta_file, encoding='utf-8') as f:
                bbox = bbox_from_meta(yaml.safe_load(f))
        except (OSError, yaml.YAMLError, GeocodeError) as e:
            raise StartupError(f'Could not read the graph area from {meta_file!r}: {e}') from e

        # --- Cache ---
        # Problems with the file never stop the node: they are logged and the
        # cache starts empty (the bad file is kept, see geocode_cache).
        cache = GeocodeCache(param('cache_file'), cache_config)
        for warning in cache.load():
            self.get_logger().warning(warning)

        # --- Off switch ---
        # online_enabled can be changed while running, without a restart or a
        # software update: ros2 param set /geocoder online_enabled false
        # A bad User-Agent forces it off (the server would block us).
        online = bool(param('online_enabled'))
        problem = user_agent_problem(self._user_agent)
        if online and problem:
            self.get_logger().error(f'Online search switched off: {problem}')
            online = False
        self._geocoder = Geocoder(bbox, cache, self._fetch, self._config, limiter, online)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # --- Service ---
        # Blocking inside the callback is fine here: this node does nothing
        # else, and one request at a time is exactly what the policy asks for.
        # The mission manager calls it asynchronously, so it never waits on us.
        self._service = self.create_service(Geocode, 'guide/geocode', self._on_geocode)
        self.get_logger().info(
            f'Geocoder ready: online search {"ON" if online else "OFF"} via {self._base_url}, '
            f'User-Agent "{self._user_agent}", {len(cache)} cached answers in {cache.path}'
            f'{" (read-only)" if cache.read_only else ""}, area lat {bbox.min_lat:.4f}..'
            f'{bbox.max_lat:.4f} lon {bbox.min_lon:.4f}..{bbox.max_lon:.4f}. {ATTRIBUTION}.')

    def _on_set_parameters(self, params):
        """Accept a runtime change of online_enabled only (others need a restart)."""
        for p in params:
            if p.name != 'online_enabled':
                return SetParametersResult(
                    successful=False, reason=f'{p.name} is read at startup: restart the node')
            if not isinstance(p.value, bool):
                return SetParametersResult(successful=False,
                                           reason='online_enabled must be true or false')
        for p in params:
            problem = user_agent_problem(self._user_agent)
            if p.value and problem:
                return SetParametersResult(successful=False, reason=problem)
            self._geocoder.online = p.value
            self.get_logger().info(f'Online search switched {"ON" if p.value else "OFF"}')
        return SetParametersResult(successful=True)

    def _fetch(self, params):
        """Send one GET /search request and return the parsed JSON (FetchError on failure).

        urlopen raises HTTPError for 4xx/5xx answers and URLError (or
        TimeoutError) when the server cannot be reached in time.
        """
        host = urlsplit(self._base_url).netloc
        request = Request(f'{self._base_url}/search?{urlencode(params)}',
                          headers={'User-Agent': self._user_agent,
                                   'Accept': 'application/json'})
        try:
            with urlopen(request, timeout=self._timeout) as answer:
                body = answer.read(MAX_ANSWER_BYTES + 1)
        except HTTPError as e:
            kind = 'refused' if e.code in (403, 429) else 'offline'
            raise FetchError(f'HTTP {e.code} from {host}', kind) from e
        except (URLError, OSError) as e:   # includes TimeoutError and DNS failures
            reason = getattr(e, 'reason', e)
            raise FetchError(f'cannot reach {host}: {reason}') from e
        if len(body) > MAX_ANSWER_BYTES:
            raise FetchError(f'answer from {host} is too large', 'bad_answer')
        try:
            return json.loads(body.decode('utf-8'))
        except (UnicodeDecodeError, ValueError) as e:
            raise FetchError(f'answer from {host} is not JSON', 'bad_answer') from e

    def _on_geocode(self, request, response):
        """Look the query up; every outcome is a status, never an exception."""
        near = (request.latitude, request.longitude) if request.radius > 0.0 else None
        result = self._geocoder.lookup(request.query, near, request.radius)
        response.status = int(result.status)
        response.from_cache = result.from_cache
        response.stale = result.stale
        response.message = result.message
        response.attribution = ATTRIBUTION
        query_key = normalise(result.query)
        for place, label in zip(result.places, labels(result.places)):
            candidate = DestinationCandidate()
            candidate.name = label
            candidate.matched_text = place.name
            candidate.match_type = DestinationCandidate.FUZZY   # Nominatim's search is fuzzy
            candidate.score = float(match_score(query_key, normalise(place.name)))
            candidate.note = 'large area' if is_large_area(place, self._config) else ''
            candidate.latitude = place.lat
            candidate.longitude = place.lon
            candidate.straight_line_distance = -1.0   # robot position unknown here
            candidate.source = NOMINATIM_SOURCE
            response.candidates.append(candidate)
        # OK and NO_RESULTS are normal answers; anything else is worth a warning.
        line = f'"{result.query}" -> {result.status.name}: {result.message}'
        if result.status in (Status.OK, Status.NO_RESULTS):
            self.get_logger().info(line)
        else:
            self.get_logger().warning(line)
        return response


def main(args=None):
    """Start the node; exit with code 1 if it cannot start."""
    rclpy.init(args=args)
    try:
        node = GeocoderNode()
    except StartupError as e:
        rclpy.logging.get_logger('geocoder').fatal(str(e))
        rclpy.try_shutdown()
        sys.exit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl+C: normal shutdown
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
