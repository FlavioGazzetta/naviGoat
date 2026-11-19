#!/usr/bin/env python3
"""A fake Nominatim server on localhost, for testing the geocoder without the internet.

Serves GET /search?q=... from the canned responses saved in 7b.1
(guide_core/test/data/nominatim_<query>.json): "science museum" ->
nominatim_science_museum.json, any query without a file -> [] (no results).
Prints every request with its User-Agent, so you can check what the
geocoder really sends.

Dev tool only (not a ROS node, not part of the production build). Run:
    ros2 run guide_sim fake_nominatim [--port 8088] [--delay 10] [--status 429]
then start the geocoder with -p base_url:=http://127.0.0.1:8088
--delay makes every answer late (test timeouts); --status answers every
request with that HTTP error (test 429/403 refusals and 5xx errors).
"""

import argparse
from http.server import BaseHTTPRequestHandler, HTTPServer
import os
import re
import time
from urllib.parse import parse_qs, urlsplit

DEFAULT_DATA_DIR = '~/guide_ws/src/guide_core/test/data'


def canned_file(data_dir, query):
    """Return the canned answer's path: 'science museum' -> nominatim_science_museum.json."""
    key = re.sub(r'[^a-z0-9]+', '_', query.lower()).strip('_')
    return os.path.join(data_dir, f'nominatim_{key}.json')


def make_handler(data_dir, delay, status):
    """Build the request handler class (http.server creates one per request)."""

    class Handler(BaseHTTPRequestHandler):

        def do_GET(self):  # noqa: N802 (name required by http.server)
            url = urlsplit(self.path)
            query = parse_qs(url.query).get('q', [''])[0]
            print(f'GET {url.path} q={query!r} User-Agent={self.headers.get("User-Agent")!r}',
                  flush=True)
            if delay > 0.0:
                time.sleep(delay)
            if status:
                self.send_error(status, 'fake_nominatim --status')
                return
            if url.path != '/search':
                self.send_error(404, 'only /search is faked')
                return
            path = canned_file(data_dir, query)
            body = b'[]'
            if os.path.exists(path):
                with open(path, 'rb') as f:
                    body = f.read()
            print(f'  -> {os.path.basename(path) if os.path.exists(path) else "[] (no file)"}',
                  flush=True)
            self.send_response(200)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):
            pass  # do_GET prints its own one-line summary

    return Handler


def main(argv=None):
    """Serve until Ctrl+C."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--port', type=int, default=8088)
    parser.add_argument('--data-dir', default=DEFAULT_DATA_DIR)
    parser.add_argument('--delay', type=float, default=0.0, help='seconds before answering')
    parser.add_argument('--status', type=int, default=0, help='answer with this HTTP error')
    args = parser.parse_args(argv)
    data_dir = os.path.expanduser(args.data_dir)
    files = sorted(f for f in os.listdir(data_dir) if f.endswith('.json'))
    print(f'Fake Nominatim on http://127.0.0.1:{args.port} serving {len(files)} canned '
          f'answers from {data_dir}' + (f', delay {args.delay} s' if args.delay else '')
          + (f', always HTTP {args.status}' if args.status else ''), flush=True)
    handler = make_handler(data_dir, args.delay, args.status)
    server = HTTPServer(('127.0.0.1', args.port), handler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
