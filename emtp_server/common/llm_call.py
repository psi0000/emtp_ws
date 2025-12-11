# -*- coding: utf-8 -*-
"""
통합 LLM 호출 유틸
- OpenAI: GPT-4o, GPT-4o-mini
- Google: Gemini 2.0 Flash
- OpenRouter: Llama-3.1-70B (OpenAI 호환)
- Groq: Llama-3.1-70B (OpenAI 호환)
- Ollama: 로컬 llama3
사용법:
  1) 아래 LLM_Model 값을 원하는 프로바이더로 설정
  2) call_llm(prompt) 호출 (str 또는 [SystemMessage, HumanMessage] 리스트)
"""

import os
import time
from typing import List, Union

from langchain_google_genai import ChatGoogleGenerativeAI
from openai import OpenAI, OpenAIError
from langchain_core.messages import SystemMessage, HumanMessage

# 프로젝트 내부 로깅 유틸 (질문에서 주신 그대로 사용)
from common.token_survey import add_openai

# ====== 설정 ======
# "openai" | "gemini" | "ollama" | "openrouter_llama" | "groq_llama"
LLM_Model = "openai"

# 기본 모델명(필요 시 변경 가능)
OPENAI_MODEL = "gpt-4o"            # 또는 "gpt-4o" or "gpt-4o-mini"
GEMINI_MODEL = "gemini-2.0-flash"  # "gemini-2.0-flash" ,"gemini-2.5-flash"
OPENROUTER_LLAMA_MODEL = "meta-llama/llama-3.1-70b-instruct"
GROQ_LLAMA_MODEL = "llama-3.3-70b-versatile"  # Groq 모델명


def call_llm(prompt: Union[str, List[HumanMessage]]) -> str:
    print(f"🔍 Using LLM model: {LLM_Model}")
    if LLM_Model == "openai":
        return call_openai(prompt)
    elif LLM_Model == "gemini":
        return call_gemini(prompt)
    elif LLM_Model == "ollama":
        return call_ollama(prompt)
    elif LLM_Model == "openrouter_llama":
        return call_openrouter_llama(prompt)
    elif LLM_Model == "groq_llama":
        return call_groq_llama(prompt)
    else:
        raise ValueError(f"❌ Unknown LLM_MODEL: {LLM_Model}")


# ====== 내부 헬퍼 ======
def _build_messages(prompt: Union[str, List[HumanMessage]]):
    """OpenAI 호환 messages 구성"""
    if isinstance(prompt, list):
        msgs = []
        for msg in prompt:
            if isinstance(msg, SystemMessage):
                msgs.append({"role": "system", "content": msg.content})
            elif isinstance(msg, HumanMessage):
                msgs.append({"role": "user", "content": msg.content})
            else:
                raise ValueError(f"❌ Unknown message type: {type(msg)}")
        return msgs
    elif isinstance(prompt, str):
        return [{"role": "user", "content": prompt}]
    else:
        raise ValueError("❌ prompt must be str or list of messages")


# ====== OpenAI (GPT-4o / 4o-mini) ======
def call_openai(prompt: Union[str, List[HumanMessage]]) -> str:
    """
    OPENAI_API_KEY 필요
    """
    try:
        client = OpenAI()  # OPENAI_API_KEY 읽음
        start_time = time.time()
        messages = _build_messages(prompt)

        if OPENAI_MODEL == "gpt-5" or OPENAI_MODEL == "gpt-5-mini":
            response = client.chat.completions.create(
                model=OPENAI_MODEL,
                messages=messages,
                max_completion_tokens=16000,
            )
        else:
            response = client.chat.completions.create(
                model=OPENAI_MODEL,          # "gpt-4o" 또는 "gpt-4o-mini"
                messages=messages,
                temperature=0.0,   
                    
                # max_tokens=16000,
            )

        elapsed = time.time() - start_time
        # content = (response.choices[0].message.content or "").strip().lower()
        content = (response.choices[0].message.content or "").strip()

        usage = getattr(response, "usage", None) or {}
        prompt_tokens = getattr(usage, "prompt_tokens", 0) or 0
        completion_tokens = getattr(usage, "completion_tokens", 0) or 0
        total_tokens = getattr(usage, "total_tokens", 0) or (prompt_tokens + completion_tokens)
        add_openai(prompt_tokens=prompt_tokens, completion_tokens=completion_tokens)

        print(f"✅ OpenAI 응답 완료: {elapsed:.2f}s | Prompt: {prompt_tokens} | Completion: {completion_tokens} | Total: {total_tokens}")
        return content or "analyze"

    except OpenAIError as e:
        print(f"❌ OpenAI 호출 오류: {e}")
        return "analyze"
    except Exception as e:
        print(f"❌ OpenAI 예외: {e}")
        return "analyze"


# ====== OpenRouter (Llama 70B) ======
def call_openrouter_llama(prompt: Union[str, List[HumanMessage]]) -> str:
    """
    OPENROUTER_API_KEY 필요
    - OpenAI SDK 호환: base_url만 OpenRouter로 교체
    """
    try:
        api_key = os.getenv("OPENROUTER_API_KEY")
        if not api_key:
            raise RuntimeError("OPENROUTER_API_KEY is not set")

        client = OpenAI(
            api_key=api_key,
            base_url="https://openrouter.ai/api/v1"
        )
        start_time = time.time()
        messages = _build_messages(prompt)

        response = client.chat.completions.create(
            model=OPENROUTER_LLAMA_MODEL,  # "meta-llama/llama-3.1-70b-instruct"
            messages=messages,
            temperature=0.0,
            max_tokens=16384,
            # 필요 시 JSON 강제(지원 호스팅에서만):
            # response_format={"type": "json_object"},
        )

        elapsed = time.time() - start_time
        content = (response.choices[0].message.content or "").strip().lower()

        usage = getattr(response, "usage", None) or {}
        prompt_tokens = getattr(usage, "prompt_tokens", 0) or 0
        completion_tokens = getattr(usage, "completion_tokens", 0) or 0
        total_tokens = getattr(usage, "total_tokens", 0) or (prompt_tokens + completion_tokens)
        add_openai(prompt_tokens=prompt_tokens, completion_tokens=completion_tokens)

        print(f"✅ OpenRouter(Llama-70B) 완료: {elapsed:.2f}s | Prompt: {prompt_tokens} | Completion: {completion_tokens} | Total: {total_tokens}")
        return content or "analyze"

    except Exception as e:
        print(f"❌ OpenRouter Llama 호출 오류: {e}")
        return "analyze"


# ====== Groq (Llama 70B) ======
def call_groq_llama(prompt: Union[str, List[HumanMessage]]) -> str:
    """
    GROQ_API_KEY 필요
    - OpenAI SDK 호환: base_url만 Groq로 교체
    """
    try:
        api_key = os.getenv("GROQ_API_KEY")
        if not api_key:
            raise RuntimeError("GROQ_API_KEY is not set")

        client = OpenAI(
            api_key=api_key,
            base_url="https://api.groq.com/openai/v1"
        )
        start_time = time.time()
        messages = _build_messages(prompt)

        response = client.chat.completions.create(
            model=GROQ_LLAMA_MODEL,  # "llama3-70b-8192" (8B도 가능: "llama3-8b-8192")
            messages=messages,
            temperature=0.0,
            max_tokens=16384,
        )

        elapsed = time.time() - start_time
        content = (response.choices[0].message.content or "").strip().lower()

        usage = getattr(response, "usage", None) or {}
        prompt_tokens = getattr(usage, "prompt_tokens", 0) or 0
        completion_tokens = getattr(usage, "completion_tokens", 0) or 0
        total_tokens = getattr(usage, "total_tokens", 0) or (prompt_tokens + completion_tokens)
        add_openai(prompt_tokens=prompt_tokens, completion_tokens=completion_tokens)

        print(f"✅ Groq(Llama-70B) 응답 완료: {elapsed:.2f}s | Prompt: {prompt_tokens} | Completion: {completion_tokens} | Total: {total_tokens}")
        return content or "analyze"

    except Exception as e:
        print(f"❌ Groq 호출 오류: {e}")
        return "analyze"


# ====== Gemini 2.0 Flash ======
def call_gemini(prompt: Union[str, List[HumanMessage]]) -> str:
    """
    GOOGLE_API_KEY 필요
    - langchain_google_genai 가 내부에서 GOOGLE_API_KEY를 사용
    """
    try:
        llm = ChatGoogleGenerativeAI(model=GEMINI_MODEL, temperature=0.0)
        start_time = time.time()

        # langchain 인터페이스는 messages 리스트도 그대로 넣을 수 있지만
        # 여기선 단순화를 위해 문자열/리스트 모두 invoke로 전달
        result = llm.invoke(prompt)
        elapsed = time.time() - start_time

        text = (getattr(result, "content", None) or "").strip().lower()
        print(f"✅ Gemini 응답 완료: {elapsed:.2f}s")
        return text or "analyze"

    except Exception as e:
        print(f"❌ Gemini 호출 오류: {e}")
        return "analyze"


# ====== Ollama (local llama3) ======
def call_ollama(prompt: Union[str, List[HumanMessage]]) -> str:
    """
    로컬 Ollama 서버 필요 (기본: http://localhost:11434)
    llama3 모델 사전 설치: `ollama pull llama3`
    """
    import requests

    model_name = "llama3:latest"
    start = time.time()

    try:
        # 문자열만 지원 (필요시 messages -> string 변환 로직 추가)
        if isinstance(prompt, list):
            # 간단 변환: system을 안내, user를 이어붙여 하나의 prompt로
            parts = []
            for msg in prompt:
                if isinstance(msg, SystemMessage):
                    parts.append(f"[system]\n{msg.content}")
                elif isinstance(msg, HumanMessage):
                    parts.append(f"[user]\n{msg.content}")
            prompt_text = "\n\n".join(parts)
        else:
            prompt_text = prompt

        resp = requests.post(
            "http://localhost:11434/api/generate",
            json={"model": model_name, "prompt": prompt_text, "stream": False},
            timeout=120,
        )
        resp.raise_for_status()
        data = resp.json()
        full_text = (data.get("response", "") or "").strip().lower()
        elapsed = time.time() - start
        print(f"✅ Ollama 응답 완료: {elapsed:.2f}s")
        return full_text or "analyze"

    except Exception as e:
        print(f"❌ Ollama 호출 실패: {e}")
        return "analyze"


# ====== 테스트 실행 (직접 실행 시) ======
if __name__ == "__main__":
    demo = "소방 안전 점검 항목 3가지를 JSON 배열로 반환해줘. 키는 name, why, how만 사용."
    print(call_llm(demo))
