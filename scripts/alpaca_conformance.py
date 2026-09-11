"""ASCOM Alpaca conformance suite for the Rotator - uses ONLY the Alpaca REST
API (scripts/alpaca_client.py), never main/WebServer.cpp's /api/debug/*
routes. Meant as a lightweight, repeatable, no-.NET-runtime alternative to
ASCOM's own ConformU tool, tailored to exactly what this driver implements
(see main/alpaca_server/api.cpp and main/RotatorApi.cpp) - it is not a
substitute for ConformU against the full Alpaca spec, but it does check
every behaviour this project has specifically audited/fixed against that
spec this far (see CLAUDE.md's history and the ASCOM Rotator reference at
https://ascom-standards.org/api/#/Rotator%20Specific%20Methods).

Most checks are read-only or use tiny (~1 deg) moves. A few genuinely move
the rotator further (see check_bounded_single_move/check_movemechanical_
never_refused) to confirm real motion behaviour, not just error codes -
pass --skip-motion to run only the protocol-level checks.

Usage:
    python3 scripts/alpaca_conformance.py --host 172.22.102.30
    python3 scripts/alpaca_conformance.py --host 172.22.102.30 --skip-motion
"""

import argparse
import sys
import time

sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.abspath(__file__)))
from alpaca_client import (  # noqa: E402
    ERR_ACTION_NOT_IMPLEMENTED,
    ERR_INVALID_VALUE,
    ERR_NOT_CONNECTED,
    ERR_NOT_IMPLEMENTED,
    AlpacaClient,
    AlpacaError,
    AlpacaHttpError,
    discover_host,
)

CHECKS = []


class Check:
    def __init__(self, name, fn, needs_motion):
        self.name = name
        self.fn = fn
        self.needs_motion = needs_motion


def check(name, needs_motion=False):
    def deco(fn):
        CHECKS.append(Check(name, fn, needs_motion))
        return fn
    return deco


def expect_alpaca_error(fn, expected_error_number, what):
    try:
        fn()
    except AlpacaError as e:
        if e.error_number != expected_error_number:
            raise AssertionError(
                f"{what}: expected ErrorNumber 0x{expected_error_number:03X}, "
                f"got 0x{e.error_number:03X} ({e.error_message!r})") from None
        return
    raise AssertionError(f"{what}: expected AlpacaError 0x{expected_error_number:03X}, call succeeded instead")


def expect_http_error(fn, expected_status, what):
    try:
        fn()
    except AlpacaHttpError as e:
        if e.status != expected_status:
            raise AssertionError(f"{what}: expected HTTP {expected_status}, got {e.status}") from None
        return
    except AlpacaError as e:
        raise AssertionError(
            f"{what}: expected a raw HTTP {expected_status}, got a well-formed "
            f"Alpaca error 0x{e.error_number:03X} instead") from None
    raise AssertionError(f"{what}: expected HTTP {expected_status}, call succeeded instead")


# ---------------------------------------------------------------- discovery

@check("UDP discovery (unicast) answers with the HTTP port")
def check_discovery(c):
    port = discover_host(c.host, timeout=3.0)
    assert port is not None, f"{c.host} did not answer Alpaca discovery on UDP 32227"
    assert 1 <= port <= 65535, f"nonsense AlpacaPort {port}"


# ------------------------------------------------------------ management API

@check("management/apiversions lists version 1")
def check_apiversions(c):
    value = c.management_apiversions()["Value"]
    assert 1 in value, f"expected API version 1 in {value}"


@check("management/v1/description has the required fields")
def check_management_description(c):
    value = c.management_description()["Value"]
    for field in ("ServerName", "Manufacturer", "ManufacturerVersion", "Location"):
        assert value.get(field), f"management description missing/empty '{field}': {value}"


@check("management/v1/configureddevices lists exactly one Rotator")
def check_configured_devices(c):
    devices = c.management_configureddevices()["Value"]
    rotators = [d for d in devices if str(d.get("DeviceType", "")).lower() == "rotator"]
    assert len(rotators) == 1, f"expected exactly one Rotator, found {len(rotators)}: {devices}"
    d = rotators[0]
    assert d.get("DeviceNumber") == 0, f"expected DeviceNumber 0, got {d.get('DeviceNumber')}"
    assert d.get("UniqueID"), "Rotator entry has no UniqueID"
    assert d.get("DeviceName"), "Rotator entry has no DeviceName"


# ------------------------------------------------------------- connected gating

@check("CanReverse is readable without Connected (spec quirk this driver leaves unwrapped)")
def check_canreverse_no_connect_needed(c):
    c.put_connected(False)
    assert c.get_canreverse() is True, "expected CanReverse=true"


@check("every other Rotator member refuses NotConnectedException while disconnected")
def check_not_connected_gating(c):
    c.put_connected(False)
    for member_name, fn in (
        ("ismoving", c.get_ismoving),
        ("mechanicalposition", c.get_mechanicalposition),
        ("position", c.get_position),
        ("reverse", c.get_reverse),
        ("stepsize", c.get_stepsize),
        ("targetposition", c.get_targetposition),
    ):
        expect_alpaca_error(fn, ERR_NOT_CONNECTED, f"GET {member_name} while disconnected")

    expect_alpaca_error(lambda: c.put_reverse(True), ERR_NOT_CONNECTED, "PUT reverse while disconnected")
    expect_alpaca_error(c.halt, ERR_NOT_CONNECTED, "PUT halt while disconnected")
    expect_alpaca_error(lambda: c.move(1.0), ERR_NOT_CONNECTED, "PUT move while disconnected")
    expect_alpaca_error(lambda: c.move_absolute(1.0), ERR_NOT_CONNECTED, "PUT moveabsolute while disconnected")
    expect_alpaca_error(lambda: c.move_mechanical(1.0), ERR_NOT_CONNECTED, "PUT movemechanical while disconnected")
    expect_alpaca_error(lambda: c.sync(1.0), ERR_NOT_CONNECTED, "PUT sync while disconnected")


@check("Connected can be toggled and reads back correctly")
def check_connected_roundtrip(c):
    c.put_connected(True)
    assert c.get_connected() is True, "Connected did not read back true"
    c.put_connected(False)
    assert c.get_connected() is False, "Connected did not read back false"
    c.put_connected(True)  # leave connected for the checks below


# --------------------------------------------------------- common device API

@check("Description/DriverInfo/DriverVersion/InterfaceVersion/Name are non-empty and sane")
def check_common_identity(c):
    assert c.get_description(), "empty Description"
    assert c.get_driverinfo(), "empty DriverInfo"
    assert c.get_driverversion(), "empty DriverVersion"
    assert c.get_interfaceversion() >= 1, "InterfaceVersion should be >= 1"
    assert c.get_name(), "empty Name"


@check("SupportedActions is a list (possibly empty)")
def check_supported_actions(c):
    actions = c.get_supportedactions()
    assert isinstance(actions, list), f"expected a JSON array, got {type(actions).__name__}"


@check("An unrecognized Action() is ActionNotImplementedException")
def check_action_not_implemented(c):
    expect_alpaca_error(lambda: c.action("DefinitelyNotARealAction"),
                        ERR_ACTION_NOT_IMPLEMENTED, "Action() with an unknown name")


@check("CommandBlind/CommandBool/CommandString are all NotImplementedException")
def check_command_blind_bool_string(c):
    expect_alpaca_error(lambda: c.commandblind("PING"), ERR_NOT_IMPLEMENTED, "CommandBlind")
    expect_alpaca_error(lambda: c.commandbool("PING"), ERR_NOT_IMPLEMENTED, "CommandBool")
    expect_alpaca_error(lambda: c.commandstring("PING"), ERR_NOT_IMPLEMENTED, "CommandString")


@check("ClientTransactionID is echoed back exactly")
def check_transaction_id_echo(c):
    distinctive = 918273
    response = c.get("position", ClientTransactionID=distinctive)
    got = response["ClientTransactionID"]
    assert got == distinctive, f"expected ClientTransactionID {distinctive} echoed back, got {got}"


@check("Parameter names are matched case-insensitively (Reverse/reverse)")
def check_case_insensitive_params(c):
    original = c.get_reverse()
    try:
        c.put("reverse", reverse="true")
        assert c.get_reverse() is True, "lowercase 'reverse' body key was not honoured"
        c.put("reverse", REVERSE="false")
        assert c.get_reverse() is False, "uppercase 'REVERSE' body key was not honoured"
    finally:
        c.put_reverse(original)


# ------------------------------------------------------------- Rotator-specific

@check("StepSize matches the AS5600 sensor resolution the firmware regulates against")
def check_stepsize(c):
    expected = 360.0 / 4096.0 / 10.0
    actual = c.get_stepsize()
    assert abs(actual - expected) < 1e-9, f"expected StepSize={expected!r}, got {actual!r}"


@check("MoveAbsolute/MoveMechanical/Sync reject Position outside [0, 360) with InvalidValueException")
def check_absolute_position_range(c):
    for member, fn in (("moveabsolute", c.move_absolute),
                       ("movemechanical", c.move_mechanical),
                       ("sync", c.sync)):
        expect_alpaca_error(lambda fn=fn: fn(-0.001), ERR_INVALID_VALUE, f"{member}(-0.001)")
        expect_alpaca_error(lambda fn=fn: fn(360.0), ERR_INVALID_VALUE, f"{member}(360.0)")


@check("MoveAbsolute/MoveMechanical/Sync/Move with no Position parameter is a raw HTTP 400")
def check_missing_position_parameter(c):
    expect_http_error(lambda: c.put("move"), 400, "move with no Position")
    expect_http_error(lambda: c.put("moveabsolute"), 400, "moveabsolute with no Position")
    expect_http_error(lambda: c.put("movemechanical"), 400, "movemechanical with no Position")
    expect_http_error(lambda: c.put("sync"), 400, "sync with no Position")


@check("Move() accepts a value outside [0, 360) unlike the absolute setters", needs_motion=True)
def check_move_accepts_out_of_range(c):
    # Move() takes a signed relative delta, not an absolute position, so
    # nothing here needs to be in [0, 360) - unlike MoveAbsolute/
    # MoveMechanical/Sync (see check_absolute_position_range).
    before = c.get_position()
    c.move(1.0)
    c.wait_until_stopped()
    after = c.get_position()
    delta = ((after - before + 180) % 360) - 180
    assert abs(delta - 1.0) < 0.5, f"Move(1.0) should land ~1 deg further, moved {delta:.3f} deg"
    c.move(-1.0)
    c.wait_until_stopped()


@check("Halt succeeds even when nothing is moving")
def check_halt_idle(c):
    assert c.get_ismoving() is False, "expected the rotator to be idle before this check"
    c.halt()  # must not raise


@check("A single commanded move never exceeds the cable-wrap window's 380 deg span", needs_motion=True)
def check_bounded_single_move(c):
    # Both the current and target mechanical positions are always within
    # +/-190 deg of the homed zero (RotatorHW.cpp's MOTION_LIMIT_DEG), so no
    # single move can ever need more than 380 deg of travel - confirmed
    # against this exact math with Franz on 2026-09-11. This is a
    # *guarantee* on how far a move can go, not a rejection: MoveMechanical
    # never refuses a request in [0, 360) - see the next check.
    before_mech = c.get_mechanicalposition()
    target = (before_mech + 170.0) % 360.0
    t0 = time.time()
    c.move_mechanical(target)
    c.wait_until_stopped()
    elapsed = time.time() - t0
    # One revolution takes ~10s at NORMAL_MOTOR_SPEED; 380 deg plus PI-refine
    # overhead tops out well under a minute - comfortably clear of a hang.
    assert elapsed < 60, f"MoveMechanical took {elapsed:.1f}s - longer than any legal <=380 deg move should"


@check("MoveMechanical never refuses a request in [0, 360), regardless of current position", needs_motion=True)
def check_movemechanical_never_refused(c):
    for target in (0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0):
        c.move_mechanical(target)
        c.wait_until_stopped()
    c.move_mechanical(0.0)
    c.wait_until_stopped()


def run(host, skip_motion, connected_at_end=False):
    client = AlpacaClient(host)
    passed, failed = [], []
    for c in CHECKS:
        if skip_motion and c.needs_motion:
            print(f"  SKIP  {c.name}  (--skip-motion)")
            continue
        try:
            c.fn(client)
        except AssertionError as e:
            print(f"  FAIL  {c.name}\n        {e}")
            failed.append(c.name)
        except (AlpacaError, AlpacaHttpError) as e:
            print(f"  FAIL  {c.name}\n        unexpected {type(e).__name__}: {e}")
            failed.append(c.name)
        else:
            print(f"  PASS  {c.name}")
            passed.append(c.name)

    try:
        client.put_connected(connected_at_end)
    except Exception:  # noqa: BLE001 - best-effort cleanup, do not mask the real result
        pass

    print(f"\n{len(passed)} passed, {len(failed)} failed, "
          f"{sum(1 for c in CHECKS if skip_motion and c.needs_motion)} skipped")
    return not failed


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", required=True)
    ap.add_argument("--skip-motion", action="store_true",
                    help="only run protocol-level checks; do not physically move the rotator")
    args = ap.parse_args()

    ok = run(args.host, args.skip_motion)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
