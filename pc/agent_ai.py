# agent_ai.py — 최적 주차 위치 판단 (Agent AI: LLM 우선, 규칙기반 fallback)
#
# 설계: world가 스캔을 마치면 ctx(맵 인지 결과)를 넘겨 decide_parking()을 호출한다.
#   - config.USE_LLM=True 이고 API 키가 있으면 실제 LLM에게 JSON 판단을 요청
#   - 키가 없거나 실패하면 규칙기반 휴리스틱으로 안전하게 대체(데모 항상 작동)
#
# 반환: {"slot": [x, y], "reason": "한국어 한두 문장", "provider": "anthropic|openai|heuristic"}
#
# LLM 사용 방법:
#   1) pip install anthropic   (또는 openai)
#   2) 환경변수 ANTHROPIC_API_KEY (또는 OPENAI_API_KEY) 설정
#   3) config.USE_LLM = True

import json
import os

import config as C


def decide_parking(ctx):
    """ctx: {n, entry, exit, pos, empty[[x,y]...], occupied[...], dist{...}}"""
    if not ctx.get("empty"):
        return {"slot": None, "reason": "인지된 빈 주차칸이 없습니다", "provider": "heuristic"}
    if C.USE_LLM:
        res = _llm_decide(ctx)
        if res:
            return res
    return _heuristic(ctx)


# ── 규칙기반 fallback ────────────────────────────────────────
def _heuristic(ctx):
    """비용 = 현재까지 경로길이 + 0.6*출구거리 (작을수록 최적)."""
    dist = ctx.get("dist", {})
    best, best_score = None, 1e9
    for s in ctx["empty"]:
        key = f"{s[0]},{s[1]}"
        d = dist.get(key)
        if d:
            score = d["path_len"] + 0.6 * d["exit_dist"]
        else:
            score = abs(ctx["exit"][0] - s[0]) + abs(ctx["exit"][1] - s[1])
        if score < best_score:
            best_score, best = score, s
    reason = (f"구획 {tuple(best)} 선택 — 진입 경로가 짧고 출구와 가까워 "
              f"주차·출차가 모두 효율적 (규칙기반 판단)")
    return {"slot": best, "reason": reason, "provider": "heuristic"}


# ── LLM 판단 ────────────────────────────────────────────────
_SYSTEM = (
    "너는 자율 발렛파킹 차량의 의사결정 모듈이다. 주차장 인지 결과를 받아 "
    "가장 합리적인 빈 주차 구획 하나를 고른다. 기준: 현재 위치에서의 진입 경로 길이, "
    "출구까지의 거리, 주변 혼잡도(점유칸 인접). 반드시 주어진 빈 칸 목록 중에서만 고른다. "
    "출력은 JSON 한 개만: {\"slot\":[x,y],\"reason\":\"한국어 한두 문장\"}. 다른 텍스트 금지."
)


def _build_prompt(ctx):
    return (
        f"맵 크기: {ctx['n']}x{ctx['n']} (좌표 1-based, x=열 y=행)\n"
        f"입구 IN: {ctx['entry']}, 출구 OUT: {ctx['exit']}\n"
        f"차량 현재 위치: {ctx['pos']}\n"
        f"빈 주차칸 후보: {ctx['empty']}\n"
        f"점유된 칸: {ctx['occupied']}\n"
        f"후보별 비용(현재→칸 경로길이 path_len, 칸→출구 거리 exit_dist):\n"
        f"{json.dumps(ctx['dist'], ensure_ascii=False)}\n"
        f"위 빈 칸 후보 중 최적 한 곳을 골라 JSON으로만 답하라."
    )


def _llm_decide(ctx):
    try:
        if C.LLM_PROVIDER == "anthropic":
            return _anthropic(ctx)
        if C.LLM_PROVIDER == "openai":
            return _openai(ctx)
    except Exception as e:
        print(f"[agent_ai] LLM 실패 → 규칙기반 fallback: {e}")
    return None


def _parse(text, ctx):
    """LLM 응답에서 JSON 추출·검증."""
    s = text.find("{"); e = text.rfind("}")
    if s < 0 or e < 0:
        return None
    obj = json.loads(text[s:e + 1])
    slot = obj.get("slot")
    if isinstance(slot, list) and len(slot) == 2 and slot in ctx["empty"]:
        return {"slot": slot, "reason": obj.get("reason", "LLM 판단"),
                "provider": C.LLM_PROVIDER}
    return None


def _anthropic(ctx):
    if not os.environ.get("ANTHROPIC_API_KEY"):
        return None
    import anthropic
    client = anthropic.Anthropic()
    msg = client.messages.create(
        model=C.LLM_MODEL, max_tokens=300, system=_SYSTEM,
        messages=[{"role": "user", "content": _build_prompt(ctx)}],
    )
    return _parse(msg.content[0].text, ctx)


def _openai(ctx):
    if not os.environ.get("OPENAI_API_KEY"):
        return None
    from openai import OpenAI
    client = OpenAI()
    r = client.chat.completions.create(
        model=C.LLM_MODEL, max_tokens=300,
        messages=[{"role": "system", "content": _SYSTEM},
                  {"role": "user", "content": _build_prompt(ctx)}],
    )
    return _parse(r.choices[0].message.content, ctx)
