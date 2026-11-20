"""The language model Pete's nodes share (scene description and the middleman).

With an API key and a connection it is Claude, over the internet. Otherwise, or when Claude is slow or fails, it is the
small local vision model through Ollama. Both get the same instructions, so the caller does not care which answered.

    ANTHROPIC_API_KEY   the key; without it only the local model is used
    PETE_CLOUD_MODEL    default claude-haiku-5-5
    PETE_LOCAL_MODEL    default the LFM2.5-VL 1.6B below
    PETE_LOCAL_LLM      false: the local model is never loaded or asked (pete.launch.py's local_llm:=false), which frees
                        about 0.9 GB. Claude then answers alone, and offline nothing does
"""

import concurrent.futures
import json
import os
import socket
import threading
import time
import urllib.error
import urllib.request

OLLAMA_URL = 'http://localhost:11434/api/chat'
# Chosen on Pete's own two jobs against Qwen3-VL 2B (2.0 GB loaded): 0.88 GB loaded, the same 8 of 11 middleman requests
# right, descriptions in under a second on the laptop. The 450M version was too weak (a pizza photo became a bedroom).
LOCAL_MODEL = os.environ.get('PETE_LOCAL_MODEL', 'lfm2.5-vl:1.6b')
LOCAL_ENABLED = os.environ.get('PETE_LOCAL_LLM', 'true').strip().lower() not in ('false', '0', 'off', 'no')
OLLAMA_OPTIONS = {'num_ctx': 4096}  # identical on every request, or Ollama reloads the model

CLOUD_MODEL = os.environ.get('PETE_CLOUD_MODEL', 'claude-haiku-5-5')
CLOUD_TIMEOUT = 4.0  # seconds before giving up on Claude and asking the local model, the address lookup included
CLOUD_RETRY_AFTER = 30.0  # after a rate limit or a server error, seconds of local-only answers
CLOUD_HOST = 'api.anthropic.com'
PROBE_EVERY = 10.0  # while offline, seconds between background checks that Claude can be reached again

_client = None
_cloud_off_until = 0.0
_cloud_disabled = False
# Offline: no request waits for Claude; a background thread checks for the connection and clears this. Without it, on
# the Jetson with Wi-Fi off, each attempt hung for 23 s looking up Claude's address, which the client's timeout misses.
_offline = threading.Event()
_pool = concurrent.futures.ThreadPoolExecutor(max_workers=4)


def _probe():
    """Runs while offline: clear _offline once Claude's server can be reached again."""
    while _offline.is_set():
        time.sleep(PROBE_EVERY)
        try:
            socket.create_connection((CLOUD_HOST, 443), timeout=3).close()
            _offline.clear()
            print('llm: online again, using Claude.', flush=True)
        except OSError:
            pass


def _went_offline():
    if not _offline.is_set():
        _offline.set()
        print('llm: Claude cannot be reached; using the local model until the connection is back.', flush=True)
        threading.Thread(target=_probe, daemon=True).start()


def _cloud_client():
    """The Anthropic client, or None without a key or without the anthropic package."""
    global _client, _cloud_disabled
    if _client is None and not _cloud_disabled:
        if not os.environ.get('ANTHROPIC_API_KEY'):
            _cloud_disabled = True
            return None
        try:
            import anthropic
            # gzip only: the navigoat:gps image's Brotli (1.1.0) is too old for its web library, and every Brotli-compressed
            # reply failed to unpack, which looked like "Claude cannot be reached"
            _client = anthropic.Anthropic(timeout=CLOUD_TIMEOUT, max_retries=0, default_headers={'Accept-Encoding': 'gzip'})
        except ImportError:
            _cloud_disabled = True
    return _client


def cloud_available():
    return (not _cloud_disabled and _cloud_client() is not None and not _offline.is_set()
            and time.monotonic() >= _cloud_off_until)


def ask_cloud(system, text, image=None, schema=None, max_tokens=1024):
    """Claude's answer as text (JSON text when a schema is given), or None: no key, offline, slow, refused or an error.
    image is a base64 JPEG."""
    global _cloud_off_until, _cloud_disabled
    if not cloud_available():
        return None
    import anthropic
    content = [{'type': 'text', 'text': text}]
    if image:
        content.insert(0, {'type': 'image', 'source': {'type': 'base64', 'media_type': 'image/jpeg', 'data': image}})
    output_config = {'effort': 'low'}  # short answers: little thinking, quicker and cheaper
    if schema:
        output_config['format'] = {'type': 'json_schema', 'schema': schema}
    call = _pool.submit(_client.messages.create, model=CLOUD_MODEL, max_tokens=max_tokens, system=system,
                        messages=[{'role': 'user', 'content': content}], output_config=output_config)
    try:
        response = call.result(timeout=CLOUD_TIMEOUT)  # a hard limit: the client's own timeout misses the address lookup
    except concurrent.futures.TimeoutError:
        _went_offline()
        return None
    except anthropic.AuthenticationError:
        print('llm: the Anthropic API key was rejected; using the local model only.', flush=True)
        _cloud_disabled = True
        return None
    except anthropic.RateLimitError:
        _cloud_off_until = time.monotonic() + CLOUD_RETRY_AFTER
        return None
    except anthropic.APIStatusError as error:
        print(f'llm: Claude answered with an error ({error.status_code}); using the local model.', flush=True)
        _cloud_off_until = time.monotonic() + CLOUD_RETRY_AFTER
        return None
    except anthropic.APIConnectionError:  # offline
        _went_offline()
        return None
    if response.stop_reason == 'refusal':
        return None
    return next((block.text for block in response.content if block.type == 'text'), None)


def ollama_chat(body, timeout=60):
    request = urllib.request.Request(OLLAMA_URL, json.dumps(body).encode(), {'Content-Type': 'application/json'})
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.load(response)


def ask_local(system, text, image=None, schema=None, max_tokens=90, timeout=60, model=None, options=None):
    """The local model's answer as text, or None if it is switched off, or Ollama cannot be reached or is too slow."""
    if not LOCAL_ENABLED:
        return None
    user = {'role': 'user', 'content': text}
    if image:
        user['images'] = [image]
    body = {'model': model or LOCAL_MODEL, 'stream': False, 'keep_alive': -1, 'messages': [{'role': 'system', 'content': system}, user],
            'options': {**OLLAMA_OPTIONS, 'num_predict': max_tokens, **(options or {})}}
    if schema:
        body['format'] = schema
    try:
        return ollama_chat(body, timeout).get('message', {}).get('content')
    except (urllib.error.URLError, TimeoutError, ValueError):
        return None


def load_local(model=None):
    """Load the local model onto the GPU and keep it there. True if it loaded; False also when it is switched off."""
    if not LOCAL_ENABLED:
        return False
    for _ in range(30):  # Ollama may still be starting
        try:
            ollama_chat({'model': model or LOCAL_MODEL, 'messages': [], 'keep_alive': -1, 'options': OLLAMA_OPTIONS}, timeout=300)
            return True
        except urllib.error.URLError:
            time.sleep(1)
        except TimeoutError:
            return False
    return False
