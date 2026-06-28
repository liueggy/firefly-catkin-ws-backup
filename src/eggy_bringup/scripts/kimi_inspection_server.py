


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


# 从环境变量读取 Kimi API Key，避免硬编码密钥
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
    压缩图片，加快上传和识别速度。
    """
    img = Image.open(input_path).convert("RGB")

    max_side = 768
    w, h = img.size

    if max(w, h) > max_side:
        scale = max_side / max(w, h)
        new_w = int(w * scale)
        new_h = int(h * scale)
        img = img.resize((new_w, new_h))

    img.save(output_path, "JPEG", quality=70, optimize=True)


def image_to_base64(image_path):
    with open(image_path, "rb") as f:
        image_bytes = f.read()

    image_b64 = base64.b64encode(image_bytes).decode("utf-8")
    return f"data:image/jpeg;base64,{image_b64}"


def extract_json(text):
    """
    尽量从模型返回中提取 JSON。
    """
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
        "reason": "模型返回内容不是合法 JSON",
        "raw": text
    }


def analyze_image_with_kimi(image_file, prompt, prefix):
    """
    通用图像分析函数。
    image_file: Flask 上传的图片
    prompt: 给 Kimi 的任务提示词
    prefix: 保存图片时的前缀，例如 meter / pipe
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
                "content": "你是一个严谨的工业巡检图像分析助手，只输出用户要求的 JSON。"
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
    水表读数识别接口。
    """
    if "image" not in request.files:
        return jsonify({
            "ok": False,
            "error": "no image"
        }), 400

    prompt = """
你是一个水表读数识别助手。请分析图片中是否存在水表，并尽量识别水表读数。

要求：
1. 只返回 JSON，不要输出解释性文字。
2. 如果能看清读数，请填写 reading。
3. 如果看不清，请 reading 写 unknown。
4. confidence 用 0.0 到 1.0 表示你的把握程度。
5. status 只能是 normal、unclear、abnormal 三种之一。
6. 如果图片不是水表，target 写 none。

返回格式必须严格如下：
{
  "target": "water_meter 或 none",
  "reading": "读数，例如 123.45；看不清写 unknown",
  "unit": "m3",
  "confidence": 0.0,
  "status": "normal/unclear/abnormal",
  "reason": "简短说明"
}
"""

    try:
        image = request.files["image"]
        result, image_saved = analyze_image_with_kimi(image, prompt, "meter")

        return jsonify({
            "ok": True,
            "task": "water_meter",
            "image_saved": image_saved,
            "result": result
        })

    except Exception as e:
        return jsonify({
            "ok": False,
            "task": "water_meter",
            "error": str(e)
        }), 500


@app.route("/analyze_pipe", methods=["POST"])
def analyze_pipe():
    """
    水管破损/漏水/锈蚀检查接口。
    """
    if "image" not in request.files:
        return jsonify({
            "ok": False,
            "error": "no image"
        }), 400

    prompt = """
你是一个水管巡检图像分析助手。请判断图片中的水管是否存在异常。

测试约定：
如果水管上或水管附近出现红色标记、红色线条、红色区域，表示该处为破裂/裂缝标记。
遇到红色标记时，应判断为异常，has_abnormal 为 true，status 为 abnormal，defect_type 优先写 crack 或 breakage。

重点检查：
1. 是否有破损或断裂
2. 是否有裂缝
3. 是否有漏水痕迹
4. 是否有明显锈蚀
5. 是否有变形
6. 接头处是否疑似松动或异常

要求：
1. 只返回 JSON，不要输出解释性文字。
2. 如果图片中没有水管，target 写 none。
3. 如果没有明显异常，has_abnormal 为 false，status 为 normal。
4. 如果存在疑似异常，has_abnormal 为 true，status 为 abnormal。
5. 如果图片模糊或无法判断，status 为 unclear。
6. severity 只能是 none、low、medium、high。
7. defect_type 只能是 none、crack、breakage、leak、rust、deformation、loose_joint、unknown。

返回格式必须严格如下：
{
  "target": "pipe 或 none",
  "has_abnormal": true/false,
  "defect_type": "none/crack/breakage/leak/rust/deformation/loose_joint/unknown",
  "severity": "none/low/medium/high",
  "confidence": 0.0,
  "status": "normal/unclear/abnormal",
  "reason": "简短说明你看到了什么",
  "suggestion": "处理建议，例如继续巡检/人工复核/立即处理"
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
