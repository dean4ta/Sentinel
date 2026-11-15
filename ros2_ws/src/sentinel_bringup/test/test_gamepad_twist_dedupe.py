#!/usr/bin/env python3
"""T009 selftest: gamepad_twist should_publish dedupe contract.

Runs on the Mac with NO ROS/rclpy installed. gamepad_twist.py imports rclpy
at module top, so this test never imports (executes) the module. Instead it:

1. ast.parse()s the source with feature_version=(3, 10) — py310 grammar check.
2. Extracts ONLY the pure `should_publish` FunctionDef from the module AST
   and compiles it standalone (the helper has no ROS dependency).
3. Exercises the dedupe contract with plain asserts.
4. py_compile-compiles the full module (bytecode only; imports never run).

Run:  python3 test/test_gamepad_twist_dedupe.py
"""

import ast
import pathlib
import py_compile
import types

HERE = pathlib.Path(__file__).resolve().parent
SRC = HERE.parent / 'sentinel_bringup' / 'gamepad_twist.py'


def _twist(x=0.0, y=0.0, z=0.0):
    """Minimal stand-in for geometry_msgs.msg.Twist (no ROS imports)."""
    return types.SimpleNamespace(
        linear=types.SimpleNamespace(x=x, y=y),
        angular=types.SimpleNamespace(z=z),
    )


def main():
    source = SRC.read_text()

    # 1. py310 grammar check.
    ast.parse(source, filename=str(SRC), feature_version=(3, 10))

    # 2. Extract and compile the pure helper without executing the module
    #    (top-level rclpy imports must never run here).
    tree = ast.parse(source, filename=str(SRC))
    fn = next(
        (n for n in tree.body
         if isinstance(n, ast.FunctionDef) and n.name == 'should_publish'),
        None)
    assert fn is not None, 'should_publish helper not found in gamepad_twist.py'
    helper_mod = types.ModuleType('_gamepad_twist_pure')
    code = compile(
        ast.fix_missing_locations(ast.Module(body=[fn], type_ignores=[])),
        str(SRC), 'exec')
    exec(code, helper_mod.__dict__)
    should_publish = helper_mod.should_publish
    assert callable(should_publish), 'should_publish is not callable'

    zero = _twist()
    moving = _twist(x=1.0, z=0.5)

    # 3. Dedupe contract.
    # None -> value publishes (first message).
    assert should_publish(None, moving) is True
    # Same value suppressed (autorepeat at 20 Hz).
    assert should_publish(moving, _twist(x=1.0, z=0.5)) is False
    # Tiny epsilon difference suppressed.
    assert should_publish(moving, _twist(x=1.0, z=0.5 + 5e-10)) is False
    # Real change passes (each motion axis).
    assert should_publish(moving, _twist(x=1.1, z=0.5)) is True
    assert should_publish(moving, _twist(x=1.0, y=0.01, z=0.5)) is True
    assert should_publish(moving, _twist(x=1.0, z=0.6)) is True
    # Value -> zero publishes exactly once (stop-on-release).
    assert should_publish(moving, zero) is True
    # Zero -> zero suppressed (idle sticks after the single stop).
    assert should_publish(zero, zero) is False

    # 4. Whole-module bytecode compile (imports never execute).
    py_compile.compile(str(SRC), doraise=True)

    print('test_gamepad_twist_dedupe.py: ALL PASS')


if __name__ == '__main__':
    main()
