from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]


def read_source(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8", errors="ignore")


def function_body(source: str, name: str) -> str:
    match = re.search(
        rf"\b(?:static\s+)?\w+\s+{name}\s*\([^)]*\)[^\n{{]*(?:\n\s*)?\{{",
        source,
    )
    assert match, f"{name}() is missing"
    index = match.end()
    depth = 1
    while index < len(source) and depth:
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
        index += 1
    assert depth == 0, f"{name}() body is not balanced"
    return source[match.end(): index - 1]


def test_emergency_stop_outputs_zero_current_while_stop_flag_is_set():
    main_c = read_source("Core/Src/main.c")
    motor_c = read_source("Core/Src/Motor_Speed_pid.c")
    motor_h = read_source("Core/Inc/Motor_Speed_pid.h")

    assert "Emergency_Stop_Output();" in main_c
    assert "void Emergency_Stop_Output(void);" in motor_h

    body = function_body(motor_c, "Emergency_Stop_Output")
    assert re.search(r"\bVcx\s*=\s*0", body)
    assert re.search(r"\bWc\s*=\s*0", body)
    assert re.search(r"\bmotor_pid\s*\[\s*i\s*\]\.target\s*=\s*0", body)
    assert "CAN_cmd_chassis(0,0,0,0)" in body.replace(" ", "")


def test_stop_paths_call_set_free_instead_of_referencing_function_names():
    motor_c = read_source("Core/Src/Motor_Speed_pid.c")

    active_code = re.sub(r"//.*", "", motor_c)
    assert "Set_free;" not in active_code
    assert "led_white_start;" not in active_code
    assert active_code.count("Set_free();") >= 2


def test_ps2_b_uses_disable_semantics_and_sends_zero_current_immediately():
    joystick_c = read_source("Core/Src/Joystick.c")

    b_branch = re.search(
        r"if\s*\(\s*PS2_KEY\s*==\s*14\s*\)[^{]*\{(?P<body>.*?)\n\s*\}",
        joystick_c,
        re.S,
    )
    assert b_branch, "PS2 B branch is missing"
    body = b_branch.group("body")

    assert re.search(r"\bmotor_shutdown\s*=\s*1\s*;", body)
    assert re.search(r"\bmotor_ready\s*=\s*0\s*;", body)
    assert re.search(r"\bfree_flag\s*=\s*1\s*;", body)
    assert "Set_free();" in body


def test_pid_limits_integral_and_updates_last_error():
    pid_c = read_source("Core/Src/pid.c")
    body = function_body(pid_c, "pid_calculate")

    assert re.search(r"if\s*\(\s*pid->iout\s*>\s*pid->IntegralLimit\s*\)", body)
    assert re.search(r"pid->iout\s*=\s*pid->IntegralLimit\s*;", body)
    assert re.search(r"if\s*\(\s*pid->iout\s*<\s*-\s*pid->IntegralLimit\s*\)", body)
    assert re.search(r"pid->iout\s*=\s*-\s*pid->IntegralLimit\s*;", body)
    assert re.search(r"pid->last_err\s*=\s*pid->err\s*;", body)
    assert re.search(r"\belapsed\s*==\s*0", body)
    assert re.search(r"\belapsed\s*>\s*50", body)
    assert re.search(r"pid->dtime\s*=\s*\(uint8_t\)\s*elapsed\s*;", body)


def test_firmware_build_flags_reject_unused_value_regressions():
    makefile = read_source("Makefile")

    assert "-Werror=unused-value" in makefile
