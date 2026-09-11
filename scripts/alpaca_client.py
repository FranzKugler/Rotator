"""Minimal ASCOM Alpaca REST client for the Rotator - standard library only,
deliberately independent of everything under main/WebServer.cpp's
/api/debug/* routes (those exist for firmware development, not for driving
the rotator as an ASCOM client would). scripts/alpaca_conformance.py and
scripts/alpaca_random_sweep.py both build on this and this alone, so neither
can accidentally reach for a debug shortcut.

Usage as a library:
    from alpaca_client import AlpacaClient
    rotator = AlpacaClient("172.22.102.30")
    rotator.put_connected(True)
    rotator.move_absolute(123.4)
    rotator.wait_until_stopped()
    print(rotator.get_position())

Usage as a quick CLI smoke test:
    python3 scripts/alpaca_client.py --host 172.22.102.30 status
"""

import argparse
import json
import socket
import sys
import time
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode
from urllib.request import Request, urlopen

DISCOVERY_PORT = 32227
DISCOVERY_MESSAGE = b"alpacadiscovery1"

# Alpaca error numbers this driver actually uses (main/alpaca_server/api.h) -
# see scripts/alpaca_conformance.py for what each means and where it applies.
ERR_NOT_IMPLEMENTED = 0x400
ERR_INVALID_VALUE = 0x401
ERR_VALUE_NOT_SET = 0x402
ERR_NOT_CONNECTED = 0x407
ERR_INVALID_OPERATION = 0x40B
ERR_ACTION_NOT_IMPLEMENTED = 0x40C


class AlpacaHttpError(Exception):
    """A raw HTTP-level failure (status != 200) - the firmware uses this for
    malformed requests (unknown device, missing required parameter), as
    opposed to a well-formed request that fails with an Alpaca ErrorNumber
    inside a normal 200 response."""

    def __init__(self, status, body):
        super().__init__(f"HTTP {status}: {body!r}")
        self.status = status
        self.body = body


class AlpacaError(Exception):
    """A well-formed Alpaca response reporting ErrorNumber != 0."""

    def __init__(self, error_number, error_message, response):
        super().__init__(f"Alpaca error 0x{error_number:03X}: {error_message}")
        self.error_number = error_number
        self.error_message = error_message
        self.response = response


def discover_host(host, timeout=3.0):
    """Sends the Alpaca discovery datagram directly (unicast) to `host` and
    returns the AlpacaPort it reports, or None if it didn't answer within
    `timeout`. More reliable for testing one known device than a broadcast,
    which depends on the test runner's own subnet/broadcast domain."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.settimeout(timeout)
        sock.sendto(DISCOVERY_MESSAGE, (host, DISCOVERY_PORT))
        try:
            data, _ = sock.recvfrom(1024)
        except socket.timeout:
            return None
        try:
            return json.loads(data)["AlpacaPort"]
        except Exception:  # noqa: BLE001
            return None


def discover(timeout=2.0, broadcast="255.255.255.255"):
    """Sends the Alpaca discovery datagram and returns a list of
    (address, alpaca_port) tuples that answered before `timeout`."""
    found = []
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.settimeout(timeout)
        sock.sendto(DISCOVERY_MESSAGE, (broadcast, DISCOVERY_PORT))
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                data, addr = sock.recvfrom(1024)
            except socket.timeout:
                break
            try:
                port = json.loads(data)["AlpacaPort"]
            except Exception:  # noqa: BLE001 - a malformed reply just doesn't count
                continue
            found.append((addr[0], port))
    return found


class AlpacaClient:
    def __init__(self, host, device_type="rotator", device_number=0, client_id=1234, timeout=15):
        self.host = host
        self.device_type = device_type
        self.device_number = device_number
        self.client_id = client_id
        self.timeout = timeout
        self._transaction_id = 0

    # ---- low-level plumbing -------------------------------------------------

    def _next_transaction_id(self):
        self._transaction_id += 1
        return self._transaction_id

    def _device_url(self, member):
        return f"http://{self.host}/api/v1/{self.device_type}/{self.device_number}/{member}"

    def raw_get(self, member, params=None, base_url=None):
        """Returns (status_code, parsed_json_or_None, raw_body_bytes). Never
        raises for a non-200 status or an ErrorNumber!=0 - see get()/put()
        for the checked convenience wrapper most callers want instead."""
        query = dict(params or {})
        query.setdefault("ClientID", self.client_id)
        query.setdefault("ClientTransactionID", self._next_transaction_id())
        url = (base_url or self._device_url(member)) + "?" + urlencode(query)
        try:
            with urlopen(url, timeout=self.timeout) as resp:
                body = resp.read()
                return resp.status, self._try_json(body), body
        except HTTPError as e:
            body = e.read()
            return e.code, self._try_json(body), body

    def raw_put(self, member, params=None):
        """Returns (status_code, parsed_json_or_None, raw_body_bytes)."""
        body_fields = dict(params or {})
        body_fields.setdefault("ClientID", self.client_id)
        body_fields.setdefault("ClientTransactionID", self._next_transaction_id())
        data = urlencode(body_fields).encode()
        req = Request(self._device_url(member), data=data, method="PUT",
                      headers={"Content-Type": "application/x-www-form-urlencoded"})
        try:
            with urlopen(req, timeout=self.timeout) as resp:
                body = resp.read()
                return resp.status, self._try_json(body), body
        except HTTPError as e:
            body = e.read()
            return e.code, self._try_json(body), body

    @staticmethod
    def _try_json(body):
        if not body:
            return None
        try:
            return json.loads(body)
        except Exception:  # noqa: BLE001
            return None

    def _checked(self, status, response, raw_body):
        if status != 200 or response is None:
            raise AlpacaHttpError(status, raw_body)
        error_number = response.get("ErrorNumber", 0)
        if error_number:
            raise AlpacaError(error_number, response.get("ErrorMessage", ""), response)
        return response

    def get(self, member, **params):
        """GET member, raise on any HTTP or Alpaca-level failure, return the
        full parsed response dict (including Value, ClientTransactionID,
        ServerTransactionID)."""
        return self._checked(*self.raw_get(member, params))

    def put(self, member, **params):
        return self._checked(*self.raw_put(member, params))

    def get_value(self, member, **params):
        return self.get(member, **params)["Value"]

    # ---- ASCOM common members ------------------------------------------------

    def get_connected(self):
        return bool(self.get_value("connected"))

    def put_connected(self, connected):
        self.put("connected", Connected="true" if connected else "false")

    def get_description(self):
        return self.get_value("description")

    def get_driverinfo(self):
        return self.get_value("driverinfo")

    def get_driverversion(self):
        return self.get_value("driverversion")

    def get_interfaceversion(self):
        return int(self.get_value("interfaceversion"))

    def get_name(self):
        return self.get_value("name")

    def get_supportedactions(self):
        return self.get_value("supportedactions")

    def action(self, action_name, parameters=""):
        return self.put("action", Action=action_name, Parameters=parameters)

    def commandblind(self, command, raw=False):
        self.put("commandblind", Command=command, Raw="true" if raw else "false")

    def commandbool(self, command, raw=False):
        return bool(self.put("commandbool", Command=command, Raw="true" if raw else "false")["Value"])

    def commandstring(self, command, raw=False):
        return self.put("commandstring", Command=command, Raw="true" if raw else "false")

    # ---- Rotator-specific members -------------------------------------------

    def get_canreverse(self):
        return bool(self.get_value("canreverse"))

    def get_ismoving(self):
        return bool(self.get_value("ismoving"))

    def get_mechanicalposition(self):
        return float(self.get_value("mechanicalposition"))

    def get_position(self):
        return float(self.get_value("position"))

    def get_reverse(self):
        return bool(self.get_value("reverse"))

    def put_reverse(self, reverse):
        self.put("reverse", Reverse="true" if reverse else "false")

    def get_stepsize(self):
        return float(self.get_value("stepsize"))

    def get_targetposition(self):
        return float(self.get_value("targetposition"))

    def halt(self):
        self.put("halt")

    def move(self, position_deg):
        """Relative move, in Position-space degrees - accepts any value."""
        self.put("move", Position=position_deg)

    def move_absolute(self, position_deg):
        """Absolute move, in Position-space degrees - must be in [0, 360)."""
        self.put("moveabsolute", Position=position_deg)

    def move_mechanical(self, position_deg):
        self.put("movemechanical", Position=position_deg)

    def sync(self, position_deg):
        self.put("sync", Position=position_deg)

    def wait_until_stopped(self, timeout=90, poll_interval=0.3):
        """Polls IsMoving (the only Alpaca-sanctioned way to know a move has
        finished) until it reports False. Raises TimeoutError past `timeout`
        seconds - a real possibility if the rotator hangs, which this
        project has an open investigation into (memory/rotator_angle_cal_hang.md),
        so callers should not assume this always returns promptly."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if not self.get_ismoving():
                return
            time.sleep(poll_interval)
        raise TimeoutError(f"IsMoving still true after {timeout}s")

    # ---- management API (not device-specific) --------------------------------

    def management_apiversions(self):
        status, response, body = self.raw_get(None, base_url=f"http://{self.host}/management/apiversions")
        return self._checked(status, response, body)

    def management_description(self):
        status, response, body = self.raw_get(None, base_url=f"http://{self.host}/management/v1/description")
        return self._checked(status, response, body)

    def management_configureddevices(self):
        status, response, body = self.raw_get(
            None, base_url=f"http://{self.host}/management/v1/configureddevices")
        return self._checked(status, response, body)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", required=True)
    ap.add_argument("command", nargs="?", default="status", choices=["status", "discover"])
    args = ap.parse_args()

    if args.command == "discover":
        found = discover()
        if not found:
            sys.exit("no Alpaca device answered discovery")
        for addr, port in found:
            print(f"{addr}:{port}")
        return

    client = AlpacaClient(args.host)
    try:
        print(json.dumps(client.management_description()["Value"], indent=2))
        print(f"connected={client.get_connected()}")
        connected_here = not client.get_connected()
        if connected_here:
            client.put_connected(True)
        print(f"position={client.get_position():.3f}  mechanical={client.get_mechanicalposition():.3f}  "
              f"reverse={client.get_reverse()}  ismoving={client.get_ismoving()}")
        if connected_here:
            client.put_connected(False)
    except (AlpacaError, AlpacaHttpError, HTTPError, URLError) as e:
        sys.exit(str(e))


if __name__ == "__main__":
    main()
