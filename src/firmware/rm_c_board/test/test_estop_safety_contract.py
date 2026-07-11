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


def block_body(source: str, pattern: str, description: str) -> str:
    match = re.search(pattern, source, re.S)
    assert match, f"{description} is missing"
    index = match.end()
    depth = 1
    while index < len(source) and depth:
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
        index += 1
    assert depth == 0, f"{description} body is not balanced"
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
    assert "Clear_Motor_PID_State();" in body
    assert "CAN_cmd_chassis(0,0,0,0)" in body.replace(" ", "")

    clear_body = function_body(motor_c, "Clear_Motor_PID_State")
    assert re.search(r"\bmotor_pid\s*\[\s*i\s*\]\.target\s*=\s*0", clear_body)
    assert re.search(r"\bmotor_pid\s*\[\s*i\s*\]\.err\s*=\s*0", clear_body)
    assert re.search(r"\bmotor_pid\s*\[\s*i\s*\]\.last_err\s*=\s*0", clear_body)
    assert re.search(r"\bmotor_pid\s*\[\s*i\s*\]\.last_output\s*=\s*0", clear_body)


def test_stop_paths_call_set_free_instead_of_referencing_function_names():
    motor_c = read_source("Core/Src/Motor_Speed_pid.c")

    active_code = re.sub(r"//.*", "", motor_c)
    assert "Set_free;" not in active_code
    assert "led_white_start;" not in active_code
    assert active_code.count("Set_free();") >= 2


def test_ps2_b_uses_active_brake_semantics_instead_of_disabling_motors():
    joystick_c = read_source("Core/Src/Joystick.c")

    body = block_body(
        joystick_c,
        r"if\s*\(\s*PS2_KEY\s*==\s*14\s*\)[^{]*\{",
        "PS2 B branch",
    )

    assert re.search(r"\bmotor_shutdown\s*=\s*0\s*;", body)
    assert re.search(r"\bmotor_ready\s*=\s*0\s*;", body)
    assert re.search(r"\bfree_flag\s*=\s*0\s*;", body)
    assert re.search(r"\bbrake_flag\s*=\s*1\s*;", body)
    assert re.search(r"\bVcx\s*=\s*0\s*;", body)
    assert re.search(r"\bWc\s*=\s*0\s*;", body)
    assert "Clear_Brake_State();" in body
    assert "Active_Brake_Output();" in body
    assert "Set_free();" not in body
    assert "led_pink_blink();" not in body


def test_ps2_b_latches_active_brake_without_resetting_ramp_while_held():
    joystick_c = read_source("Core/Src/Joystick.c")

    body = block_body(
        joystick_c,
        r"if\s*\(\s*PS2_KEY\s*==\s*14\s*\)[^{]*\{",
        "PS2 B branch",
    )

    assert re.search(r"if\s*\(\s*brake_flag\s*==\s*0\s*\)\s*\{", body), (
        "B branch must only initialize active braking on the first latched frame"
    )

    guard = block_body(
        body,
        r"if\s*\(\s*brake_flag\s*==\s*0\s*\)\s*\{",
        "B brake latch guard",
    )
    assert "Clear_Brake_State();" in guard
    assert re.search(r"\bbrake_flag\s*=\s*1\s*;", guard)
    assert body.count("Clear_Brake_State();") == 1


def test_active_brake_uses_damping_current_ramp_instead_of_drive_pid():
    motor_c = read_source("Core/Src/Motor_Speed_pid.c")
    motor_h = read_source("Core/Inc/Motor_Speed_pid.h")

    assert "extern int brake_flag;" in motor_h
    assert "void Active_Brake_Output(void);" in motor_h
    assert "void Clear_Brake_State(void);" in motor_h

    speed_set = function_body(motor_c, "Speed_set")
    assert re.search(r"if\s*\(\s*brake_flag\s*==\s*1\s*\)", speed_set)
    assert "Active_Brake_Output();" in speed_set

    body = function_body(motor_c, "Active_Brake_Output")
    assert "BRAKE_STOP_RPM" in motor_c
    assert "BRAKE_CURRENT_LIMIT" in motor_c
    assert "BRAKE_LOW_SPEED_RPM" in motor_c
    assert "BRAKE_LOW_SPEED_CURRENT_LIMIT" in motor_c
    assert "BRAKE_CURRENT_STEP" in motor_c
    assert re.search(r"#define\s+BRAKE_LOW_SPEED_RPM\s+300\b", motor_c)
    assert re.search(r"#define\s+BRAKE_LOW_SPEED_CURRENT_LIMIT\s+4500\b", motor_c)
    assert re.search(r"#define\s+BRAKE_DAMPING_KP\s+12\.0f\b", motor_c)
    assert re.search(r"\bVcx\s*=\s*0\s*;", body)
    assert re.search(r"\bWc\s*=\s*0\s*;", body)
    assert "Damping_Brake_Current" in body
    assert "Ramp_Brake_Current" in body
    assert "f_cal_pid" not in body
    assert re.search(r"\bbrake_flag\s*=\s*0\s*;", body)
    assert re.search(r"\bfree_flag\s*=\s*1\s*;", body)
    assert "CAN_cmd_chassis(0,0,0,0)" in body.replace(" ", "")

    damping_body = function_body(motor_c, "Damping_Brake_Current")
    assert re.search(r"-\s*BRAKE_DAMPING_KP\s*\*\s*speed_rpm", damping_body)
    assert "BRAKE_LOW_SPEED_CURRENT_LIMIT" in damping_body
    assert "BRAKE_CURRENT_LIMIT" in damping_body

    ramp_body = function_body(motor_c, "Ramp_Brake_Current")
    assert "brake_last_current" in ramp_body
    assert "BRAKE_CURRENT_STEP" in ramp_body


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


def test_serial_control_has_independent_command_watchdog():
    main_c = read_source("Core/Src/main.c")

    assert re.search(r"#define\s+SERIAL_COMMAND_TIMEOUT_MS\s+500U", main_c)
    watchdog = function_body(main_c, "Serial_Command_Watchdog")
    assert "HAL_GetTick() - last_serial_command_tick" in watchdog
    assert re.search(r"\bVcx\s*=\s*0\.0f\s*;", watchdog)
    assert re.search(r"\bWc\s*=\s*0\.0f\s*;", watchdog)
    assert "Serial_Command_Watchdog();" in main_c

    serial_input = function_body(main_c, "Serial_Input")
    assert "last_serial_command_tick = HAL_GetTick();" in serial_input
    assert "serial_command_seen = 1U;" in serial_input
    assert "UART1_RX_Buffer[terminator] = '\\0';" in main_c
