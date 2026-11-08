"""
hwconfig.py
===========
Читає числові #define з common/cart_pole_config.h, щоб Python ніколи не
дублював апаратні константи вручну (єдине джерело правди — C-заголовок).
"""

import math
import re
from functools import lru_cache
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
COMMON_DIR = ROOT / "common"
CONFIG_H = COMMON_DIR / "cart_pole_config.h"

_DEFINE_RE = re.compile(r"^\s*#define\s+([A-Z_][A-Z0-9_]*)\s+(.+?)\s*(//.*)?$")


def _join_continuations(text: str) -> list:
    lines, buf = [], ""
    for raw in text.splitlines():
        if raw.rstrip().endswith("\\"):
            buf += raw.rstrip()[:-1] + " "
            continue
        lines.append(buf + raw)
        buf = ""
    return lines


def _clean_expr(expr: str) -> str:
    expr = re.sub(r"/\*.*?\*/", "", expr)
    expr = re.sub(r"\((?:float|double|int|uint32_t|int32_t)\)", "", expr)
    expr = re.sub(r"(\d+\.\d*|\.\d+|\d+)[fF]\b", r"\1", expr)
    return expr.replace("M_PI", "math.pi")


@lru_cache(maxsize=None)
def load_hw_config(path: str = str(CONFIG_H)) -> dict:
    """Повертає словник {ІМ'Я: число} для всіх обчислюваних #define (поза ESP_PLATFORM)."""
    text = Path(path).read_text(encoding="utf-8")
    # Блоки #ifdef ESP_PLATFORM ... #endif пропускаємо
    text = re.sub(r"#ifdef ESP_PLATFORM.*?#endif", "", text, flags=re.S)
    raw = {}
    for line in _join_continuations(text):
        m = _DEFINE_RE.match(line)
        if m:
            raw[m.group(1)] = _clean_expr(m.group(2))

    values = {}

    def resolve(name, depth=0):
        if name in values:
            return values[name]
        if depth > 20 or name not in raw:
            raise KeyError(name)
        expr = raw[name]
        for dep in re.findall(r"\b[A-Z_][A-Z0-9_]*\b", expr):
            if dep in raw and dep not in values:
                resolve(dep, depth + 1)
        env = {"math": math, **values}
        values[name] = float(eval(expr, {"__builtins__": {}}, env))
        return values[name]

    for name in raw:
        try:
            resolve(name)
        except Exception:
            pass
    return values


HW = load_hw_config()


@lru_cache(maxsize=None)
def load_enum(type_name: str, header: str = str(COMMON_DIR / "controller.h")) -> dict:
    """Читає C enum (typedef enum {...} type_name;) -> {ІМ'Я: значення}."""
    text = Path(header).read_text(encoding="utf-8")
    m = re.search(r"typedef\s+enum\s*\{([^{}]*)\}\s*" + re.escape(type_name) + r"\s*;", text)
    if not m:
        raise KeyError(type_name)
    body = re.sub(r"//[^\n]*", "", m.group(1))
    values, nxt = {}, 0
    for item in body.split(","):
        item = item.strip()
        if not item:
            continue
        if "=" in item:
            name, val = (s.strip() for s in item.split("=", 1))
            nxt = int(eval(val, {"__builtins__": {}}, values))
        else:
            name = item
        values[name] = nxt
        nxt += 1
    return values


EVENTS = load_enum("ctrl_event_code_t")
STATES = load_enum("ctrl_state_t")
