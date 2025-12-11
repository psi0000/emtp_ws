# token_survey.py
import json, os, time, tempfile

# 환경변수 TOKEN_LOG_FILE 로 경로 통일 (없으면 /tmp 로)
_LOG_JSONL = os.environ.get("TOKEN_LOG_FILE", os.path.join(tempfile.gettempdir(), f"token_usage_{os.getpid()}.jsonl"))

def start():
    """실행 시작 시 기존 로그 파일 초기화"""
    try:
        os.makedirs(os.path.dirname(_LOG_JSONL), exist_ok=True)
        if os.path.exists(_LOG_JSONL):
            os.remove(_LOG_JSONL)
    except Exception:
        pass

def add_openai(prompt_tokens: int = 0, completion_tokens: int = 0):
    """OpenAI 응답마다 토큰 사용량을 파일에 한 줄씩 append"""
    rec = {
        "ts": time.strftime("%Y-%m-%d %H:%M:%S"),
        "prompt_tokens": int(prompt_tokens),
        "completion_tokens": int(completion_tokens),
        "total_tokens": int(prompt_tokens) + int(completion_tokens),
    }
    with open(_LOG_JSONL, "a", encoding="utf-8") as f:
        f.write(json.dumps(rec, ensure_ascii=False) + "\n")

def save(out_path: str):
    """jsonl 읽어서 records/ totals로 집계해 JSON 저장"""
    records = []
    totals = {"prompt_tokens": 0, "completion_tokens": 0, "total_tokens": 0}

    if os.path.exists(_LOG_JSONL):
        with open(_LOG_JSONL, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                rec = json.loads(line)
                records.append(rec)
                totals["prompt_tokens"] += rec.get("prompt_tokens", 0)
                totals["completion_tokens"] += rec.get("completion_tokens", 0)
                totals["total_tokens"] += rec.get("total_tokens", 0)

    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump({"records": records, "totals": totals}, f, indent=2, ensure_ascii=False)
