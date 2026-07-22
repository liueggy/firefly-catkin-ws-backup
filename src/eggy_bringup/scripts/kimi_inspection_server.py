#!/usr/bin/env python3
from flask import Flask, request, jsonify
from openai import OpenAI
from PIL import Image
import base64
import json
import re
import os
from datetime import datetime

app = Flask(__name__)

try:
    app.json.ensure_ascii = False
except Exception:
    pass


def load_env_file(path):
    if not path or not os.path.exists(path):
        return
    with open(path, "r", encoding="utf-8-sig") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, value = line.split("=", 1)
            key = key.strip()
            value = value.strip().strip('"').strip("'")
            if key and key not in os.environ:
                os.environ[key] = value


load_env_file(os.environ.get("KIMI_ENV_FILE", "/root/.config/kimi_inspection.env"))

# 浠庣幆澧冨彉閲忚鍙?Kimi API Key锛岄伩鍏嶇‖缂栫爜瀵嗛挜
KIMI_API_KEY = os.environ.get("KIMI_API_KEY")
if not KIMI_API_KEY:
    raise RuntimeError("KIMI_API_KEY environment variable is required")

client = OpenAI(
    api_key=KIMI_API_KEY,
    base_url="https://api.moonshot.cn/v1",
    timeout=120.0
)


@app.route("/", methods=["GET"])
def index():
    return "Inspection server is running."


def compress_image(input_path, output_path):
    """
    鍘嬬缉鍥剧墖锛屽姞蹇笂浼犲拰璇嗗埆閫熷害銆?    """
    img = Image.open(input_path).convert("RGB")

    max_side = int(os.environ.get("KIMI_IMAGE_MAX_SIDE", "1280"))
    w, h = img.size

    if max(w, h) > max_side:
        scale = max_side / max(w, h)
        new_w = int(w * scale)
        new_h = int(h * scale)
        img = img.resize((new_w, new_h))

    quality = int(os.environ.get("KIMI_IMAGE_JPEG_QUALITY", "92"))
    img.save(output_path, "JPEG", quality=quality, optimize=True)


def image_to_base64(image_path):
    with open(image_path, "rb") as f:
        image_bytes = f.read()

    image_b64 = base64.b64encode(image_bytes).decode("utf-8")
    return f"data:image/jpeg;base64,{image_b64}"


def extract_json(text):
    """
    灏介噺浠庢ā鍨嬭繑鍥炰腑鎻愬彇 JSON銆?    """
    try:
        return json.loads(text)
    except Exception:
        pass

    match = re.search(r"\{[\s\S]*\}", text)
    if match:
        try:
            return json.loads(match.group(0))
        except Exception:
            pass

    return {
        "status": "unclear",
        "confidence": 0.0,
        "reason": "妯″瀷杩斿洖鍐呭涓嶆槸鍚堟硶 JSON",
        "raw": text
    }


def focus_meter_result(result, detected_class):
    """Keep the AI result aligned with the class locked by the detector."""
    if detected_class not in ("water_meter", "pressure_gauge") or not isinstance(result, dict):
        return result
    readings = result.get("readings")
    if not isinstance(readings, dict):
        return result

    other_class = ("pressure_gauge" if detected_class == "water_meter"
                   else "water_meter")
    other = readings.get(other_class)
    if isinstance(other, dict):
        other["present"] = False
        other["reading"] = "unknown"
        other["best_effort_reading"] = "unknown"

    if detected_class == "water_meter":
        meter = readings.get("water_meter")
        analysis = result.setdefault("analysis", {})
        if isinstance(meter, dict) and meter.get("present", False):
            reading = meter.get("reading") or meter.get("best_effort_reading") or "unknown"
            readable = str(reading).strip().lower() not in ("", "unknown", "none")
            analysis.update({
                "pressure_state": "unknown",
                "severity": "normal" if readable else "unknown",
                "message": ("水表读数为%s %s" %
                            (reading, meter.get("unit") or "m3"))
                           if readable else "未能可靠读取水表，建议重新拍摄或人工复核",
                "recommended_action": "继续巡检" if readable else "重新拍摄或人工复核",
            })
        else:
            analysis.update({
                "pressure_state": "unknown",
                "severity": "unknown",
                "message": "未能可靠识别水表，建议重新拍摄或人工复核",
                "recommended_action": "重新拍摄或人工复核",
            })
    return result


def analyze_image_with_kimi(image_file, prompt, prefix):
    """
    閫氱敤鍥惧儚鍒嗘瀽鍑芥暟銆?    image_file: Flask 涓婁紶鐨勫浘鐗?    prompt: 缁?Kimi 鐨勪换鍔℃彁绀鸿瘝
    prefix: 淇濆瓨鍥剧墖鏃剁殑鍓嶇紑锛屼緥濡?meter / pipe
    """
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    raw_path = f"{prefix}_raw_{timestamp}.jpg"
    compressed_path = f"{prefix}_compressed_{timestamp}.jpg"

    image_file.save(raw_path)
    compress_image(raw_path, compressed_path)

    image_url = image_to_base64(compressed_path)

    completion = client.chat.completions.create(
        model="kimi-k2.6",
        messages=[
            {
                "role": "system",
                "content": "You are a strict industrial inspection vision assistant. Output only the JSON requested by the user."
            },
            {
                "role": "user",
                "content": [
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": image_url
                        }
                    },
                    {
                        "type": "text",
                        "text": prompt
                    }
                ]
            }
        ],
        extra_body={
            "thinking": {
                "type": "disabled"
            }
        },
        max_tokens=512
    )

    text = completion.choices[0].message.content
    result = extract_json(text)

    return result, compressed_path


@app.route("/analyze_meter", methods=["POST"])
def analyze_meter():
    """
    姘磋〃璇绘暟璇嗗埆鎺ュ彛銆?    """
    if "image" not in request.files:
        return jsonify({
            "ok": False,
            "error": "no image"
        }), 400

    detected_class = str(request.form.get("detected_class") or "").strip()
    if detected_class not in ("water_meter", "pressure_gauge"):
        detected_class = ""
    detector_focus = ""
    if detected_class:
        other_class = ("pressure_gauge" if detected_class == "water_meter"
                       else "water_meter")
        detector_focus = (
            "\n本图是目标检测器围绕 %s 裁剪的单目标 ROI。"
            "本次只读取 %s；不要把它改判成其他类别，"
            "并将 %s 的 present 设为 false。\n" %
            (detected_class, detected_class, other_class)
        )

    prompt = """
你是一个工业巡检读表助手。图片可能是整张机器人相机画面，里面可能同时出现：
- 机械水表 water_meter：蓝色/金色外壳，上方有矩形滚轮数字窗口。
- 水压表 pressure_gauge：圆形指针表盘，刻度通常为 0 到 10。
- 干扰物：矿泉水瓶、纸箱、蓝色表盖贴纸、背景文字等。

请分别读取画面中可见的机械水表和水压表：
1. 如果只看到水表，就只给 water_meter 读数，pressure_gauge.present=false。
2. 如果只看到水压表，就只给 pressure_gauge 读数，water_meter.present=false。
3. 如果两种都看到，就两个都读。
4. 不要读取瓶身标签、纸箱文字、蓝色盖子贴纸。
5. 水表读取机械滚轮数字窗口；即使有反光/倾斜/轻微模糊，也要给 best_effort_reading。某位不确定可用 ?。
6. 水压表读取指针指向的刻度值，单位 MPa；可以给一位小数，例如 3.5。若介于两格之间，给最接近估计值。
7. confidence 用 0.0 到 1.0；不确定时 status 写 unclear，但仍尽量给 best_effort_reading。
8. 读到水压表后必须分析当前巡检情况：
   - 0 <= pressure < 4：pressure_state 写 low，severity 写 warning，message 写“水压偏低，可能存在管路破损、漏水、阀门未开或供水不足，建议巡检管路”。
   - 4 <= pressure <= 6：pressure_state 写 normal，severity 写 normal，message 写“水压处于正常范围”。
   - 6 < pressure <= 10：pressure_state 写 high，severity 写 warning，message 写“水压偏高，可能存在阀门异常、堵塞或压力过高风险，建议人工复核”。
   - 看不到或无法读取水压：pressure_state 写 unknown，severity 写 unknown，message 写“未能可靠读取水压，建议重新拍摄或人工复核”。
9. 只返回 JSON，不要输出解释性文字。

返回格式必须严格如下：
{
  "target": "meter_reading",
  "readings": {
    "water_meter": {
      "present": true/false,
      "reading": "例如 00001；完全看不到写 unknown",
      "best_effort_reading": "例如 00001；没有则 unknown",
      "unit": "m3",
      "confidence": 0.0,
      "status": "normal/unclear/abnormal",
      "reason": "简短说明读的是哪个数字窗口，哪些位不确定"
    },
    "pressure_gauge": {
      "present": true/false,
      "reading": "例如 3.5；完全看不到写 unknown",
      "best_effort_reading": "例如 3.5；没有则 unknown",
      "unit": "MPa",
      "confidence": 0.0,
      "status": "normal/unclear/abnormal",
      "reason": "简短说明指针位置和估计依据"
    }
  },
  "analysis": {
    "pressure_state": "low/normal/high/unknown",
    "severity": "normal/warning/unknown",
    "message": "给前端展示的中文巡检结论",
    "possible_causes": ["可能原因1", "可能原因2"],
    "recommended_action": "建议动作，例如继续巡检/巡检管路/人工复核"
  },
  "summary": "简短总结看到并读取了哪些表"
}
""" + detector_focus

    try:
        image = request.files["image"]
        result, image_saved = analyze_image_with_kimi(image, prompt, "meter")
        result = focus_meter_result(result, detected_class)

        return jsonify({
            "ok": True,
            "task": detected_class or "meter",
            "detected_class": detected_class or None,
            "image_saved": image_saved,
            "result": result
        })

    except Exception as e:
        return jsonify({
            "ok": False,
            "task": detected_class or "meter",
            "detected_class": detected_class or None,
            "error": str(e)
        }), 500


@app.route("/analyze_pipe", methods=["POST"])
def analyze_pipe():
    """
    姘寸鐮存崯/婕忔按/閿堣殌妫€鏌ユ帴鍙ｃ€?    """
    if "image" not in request.files:
        return jsonify({
            "ok": False,
            "error": "no image"
        }), 400

    prompt = """
浣犳槸涓€涓按绠″贰妫€鍥惧儚鍒嗘瀽鍔╂墜銆傝鍒ゆ柇鍥剧墖涓殑姘寸鏄惁瀛樺湪寮傚父銆?
娴嬭瘯绾﹀畾锛?濡傛灉姘寸涓婃垨姘寸闄勮繎鍑虹幇绾㈣壊鏍囪銆佺孩鑹茬嚎鏉°€佺孩鑹插尯鍩燂紝琛ㄧず璇ュ涓虹牬瑁?瑁傜紳鏍囪銆?閬囧埌绾㈣壊鏍囪鏃讹紝搴斿垽鏂负寮傚父锛宧as_abnormal 涓?true锛宻tatus 涓?abnormal锛宒efect_type 浼樺厛鍐?crack 鎴?breakage銆?
閲嶇偣妫€鏌ワ細
1. 鏄惁鏈夌牬鎹熸垨鏂
2. 鏄惁鏈夎缂?3. 鏄惁鏈夋紡姘寸棔杩?4. 鏄惁鏈夋槑鏄鹃攬铓€
5. 鏄惁鏈夊彉褰?6. 鎺ュご澶勬槸鍚︾枒浼兼澗鍔ㄦ垨寮傚父

瑕佹眰锛?1. 鍙繑鍥?JSON锛屼笉瑕佽緭鍑鸿В閲婃€ф枃瀛椼€?2. 濡傛灉鍥剧墖涓病鏈夋按绠★紝target 鍐?none銆?3. 濡傛灉娌℃湁鏄庢樉寮傚父锛宧as_abnormal 涓?false锛宻tatus 涓?normal銆?4. 濡傛灉瀛樺湪鐤戜技寮傚父锛宧as_abnormal 涓?true锛宻tatus 涓?abnormal銆?5. 濡傛灉鍥剧墖妯＄硦鎴栨棤娉曞垽鏂紝status 涓?unclear銆?6. severity 鍙兘鏄?none銆乴ow銆乵edium銆乭igh銆?7. defect_type 鍙兘鏄?none銆乧rack銆乥reakage銆乴eak銆乺ust銆乨eformation銆乴oose_joint銆乽nknown銆?
杩斿洖鏍煎紡蹇呴』涓ユ牸濡備笅锛?{
  "target": "pipe 鎴?none",
  "has_abnormal": true/false,
  "defect_type": "none/crack/breakage/leak/rust/deformation/loose_joint/unknown",
  "severity": "none/low/medium/high",
  "confidence": 0.0,
  "status": "normal/unclear/abnormal",
  "reason": "绠€鐭鏄庝綘鐪嬪埌浜嗕粈涔?,
  "suggestion": "澶勭悊寤鸿锛屼緥濡傜户缁贰妫€/浜哄伐澶嶆牳/绔嬪嵆澶勭悊"
}
"""

    try:
        image = request.files["image"]
        result, image_saved = analyze_image_with_kimi(image, prompt, "pipe")

        return jsonify({
            "ok": True,
            "task": "pipe_inspection",
            "image_saved": image_saved,
            "result": result
        })

    except Exception as e:
        return jsonify({
            "ok": False,
            "task": "pipe_inspection",
            "error": str(e)
        }), 500


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8000, debug=False, use_reloader=False, threaded=True)

