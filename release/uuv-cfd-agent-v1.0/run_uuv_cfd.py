# -*- coding: utf-8 -*-
# ============================================================
# v116 -> run_uuv_cfd Tool 化版本
# ============================================================
# 网格阶段：
#   - 保留 v112 自适应 SpaceClaim + Fluent Meshing 逻辑；
#   - 保留当前自适应 Target Mesh Size、Curvature/Proximity、
#     5 层边界层、tetra 体网格和网格质量改进。
#
# Solver 阶段：
#   - 回归用户提供的 v52/参考代码求解思路；
#   - 不采用 v115 的 Hybrid-first / 分块迭代 / 清理 Force Monitor /
#     过滤受力面等实验修改；
#   - 保留参考代码的 12 核、100 iterations、[2.0, 4.0] m/s、
#     标准入口初始化（失败才 Hybrid）、三方向 Force Report、
#     压差/摩擦阻力、case/data、云图与 Excel。
#
# v116 唯一结构性改动：
#   v112 Meshing 写出 .msh.h5 后，关闭 Meshing Fluent，
#   再启动一个全新的 Fluent Solver 读取该 mesh。
#   这样隔离 Meshing Workflow/Field Mesher 的进程状态与内存，
#   Solver 后续逻辑仍按用户参考代码执行。
# ============================================================
r"""
v114：v112 自适应高质量网格 + Fluent Solver 多速度求解一体化（v241 路径修正版）

目标：
- 不依赖 aaa111 / bbb111 / ccc333 / hull / propeller 等任何部件名称；
- 换模型、换部件名称、换部件数量后，不需要重新手写局部网格规则；
- SpaceClaim 负责提取每个部件的 bbox、面积、体积、边/面数量、P10 小特征、
  等效厚度、近邻间隙、复杂度等；Python 根据这些几何量自动决定网格尺寸；
- 流向默认由整体 AUV 最长包围盒方向自动识别；
- 所有尺寸规则按当前模型 L/D/局部特征相对缩放，不再使用某一艘 AUV 的固定 mm 阈值；
- 边界层保持 5 层；Fluent Meshing 完成后自动切换 Solver，执行材料、边界、残差、多速度求解和后处理。

输入方式（无需改代码）：
1) 当前默认模型：C:\\Users\\Administrator\\Desktop\\shixi\\new2\\new2.x_t
   当前默认工作目录：D:\\zidonghua\\new2
2) 以后换模型时可直接传：
   D:\\Python310\\python.exe -u run_all_v114_v112_plus_solver_pathfix.py "D:\\CAD\\my_auv.step" "D:\\zidonghua\\my_auv"
3) 也可用环境变量 AUV_MODEL_PATH / AUV_WORK_DIR 覆盖；
4) 若没有显式模型路径，优先使用 DEFAULT_MODEL_PATH；若默认模型不存在，再自动搜索 CAD。

支持的原始实体 CAD 后缀：
.x_t .x_b .step .stp .iges .igs .sat .sab .scdoc

求解设置：
- 材料：water-liquid；速度列表：[2.0, 4.0] m/s；每个速度 100 次迭代；残差标准 1e-7。
- 求解部分来自用户提供的稳定 Solver 流程；网格部分保持 v112，不回退到旧的按名称固定尺寸规则。
"""

import os
import re
import csv
import math
import time
import subprocess
import sys
import json
from pathlib import Path
from datetime import datetime
import warnings
import argparse

# 尽量减少新工作站终端中的弃用提示，避免影响运行观察。
warnings.filterwarnings("ignore", category=DeprecationWarning)
try:
    from ansys.fluent.core.services.field_data import PyFluentDeprecationWarning
    warnings.filterwarnings("ignore", category=PyFluentDeprecationWarning)
except Exception:
    pass


# ============================================================

# 0. 通用输入 / 输出参数
# ============================================================

SPACECLAIM_EXE = r"D:\Program Files\ANSYS Inc\v241\SCDM\SpaceClaim.exe"

# 所有模型共用的结果根目录；每个模型自动建立独立子目录，避免旧文件串模。
FLUENT_EXE_PATH = r"D:\Program Files\ANSYS Inc\v241\fluent\ntbin\win64\fluent.exe"

# 当前这艘 new2 AUV 的默认路径。以后更换模型无需改这里，
# 可直接用命令行参数或 AUV_MODEL_PATH / AUV_WORK_DIR 覆盖。
DEFAULT_MODEL_PATH = r"D:\pyfluent\uuv.x_t"
DEFAULT_WORK_DIR = r"D:\pyfluent"

# 对未来其它模型的自动输出根目录。若传入了其它模型但没指定工作目录，
# 自动写入 D:\zidonghua\<模型名>。
WORK_ROOT = r"D:\pyfluent"
DEFAULT_MODEL_SEARCH_DIR = r"D:\pyfluent"

SUPPORTED_CAD_EXTENSIONS = {
    ".x_t", ".x_b", ".step", ".stp", ".iges", ".igs", ".sat", ".sab", ".scdoc"
}


def _build_runtime_arg_parser():
    """CFD Tool 命令行接口。保留旧版 positional model/workdir 兼容。"""
    parser = argparse.ArgumentParser(
        description=(
            "UUV CFD 自动化工具：CAD -> SpaceClaim -> Fluent Meshing -> "
            "多航速 Fluent Solver -> 水动力/云图/Excel/result_manifest.json"
        )
    )
    parser.add_argument(
        "legacy_model", nargs="?", default=None,
        help="兼容旧调用：第一个位置参数可直接传 CAD 路径。推荐使用 --model。"
    )
    parser.add_argument(
        "legacy_workdir", nargs="?", default=None,
        help="兼容旧调用：第二个位置参数可直接传工作目录。推荐使用 --workdir。"
    )
    parser.add_argument(
        "--model", dest="model", default=None,
        help=r"CAD 模型路径，例如 D:\CAD\uuv.x_t"
    )
    parser.add_argument(
        "--workdir", dest="workdir", default=None,
        help=r"结果工作目录，例如 D:\CFD\uuv_case"
    )
    parser.add_argument(
        "--velocities", dest="velocities", nargs="+", type=float, default=None,
        help="航速列表，单位 m/s，例如 --velocities 2 4 6 8"
    )
    parser.add_argument(
        "--cores", dest="cores", type=int, default=None,
        help="Fluent 计算核数，默认 6"
    )
    parser.add_argument(
        "--iterations", dest="iterations", type=int, default=None,
        help="每个航速的迭代步数，默认 100"
    )
    parser.add_argument(
        "--keep-fluent-open", dest="keep_fluent_open", action="store_true",
        help="计算完成后保留 Fluent GUI 并等待人工检查；Agent/批处理模式建议不要使用。"
    )
    parser.add_argument(
        "--skip-contours", dest="skip_contours", action="store_true",
        help="跳过 Python 云图 PNG 生成。默认生成 xoy/xoz 速度和静压云图。"
    )
    return parser


def _parse_runtime_args():
    parser = _build_runtime_arg_parser()
    # parse_known_args 避免个别 IDE/宿主注入额外参数时直接退出。
    args, _unknown = parser.parse_known_args()
    return args


RUNTIME_ARGS = _parse_runtime_args()


def _parse_velocity_env(text):
    if not text:
        return []
    vals = []
    for token in re.split(r"[,;\s]+", str(text).strip()):
        if not token:
            continue
        vals.append(float(token))
    return vals


def _normalize_velocity_list(values):
    """验证航速 > 0，并按用户输入顺序去重。"""
    out = []
    seen = set()
    for value in values:
        v = float(value)
        if not math.isfinite(v) or v <= 0:
            raise ValueError(f"航速必须为有限正数，收到: {value}")
        key = round(v, 12)
        if key not in seen:
            seen.add(key)
            out.append(v)
    if not out:
        raise ValueError("航速列表不能为空。")
    return out


def resolve_input_model():
    """输入优先级：--model > 旧位置参数 > AUV_MODEL_PATH > 默认模型 > 自动发现 CAD。"""
    candidates = []

    if getattr(RUNTIME_ARGS, "model", None):
        candidates.append(Path(str(RUNTIME_ARGS.model).strip().strip('"')))

    if getattr(RUNTIME_ARGS, "legacy_model", None):
        candidates.append(Path(str(RUNTIME_ARGS.legacy_model).strip().strip('"')))

    env_path = os.environ.get("AUV_MODEL_PATH", "").strip().strip('"')
    if env_path:
        candidates.append(Path(env_path))

    if str(DEFAULT_MODEL_PATH).strip():
        candidates.append(Path(DEFAULT_MODEL_PATH))

    for p in candidates:
        if p.is_file() and p.suffix.lower() in SUPPORTED_CAD_EXTENSIONS:
            return p.resolve()
        if p.exists() and p.is_file():
            raise RuntimeError(
                "输入文件存在，但后缀不在当前实体 CAD 支持列表中: {}".format(p)
            )

    search_dirs = [Path.cwd(), Path(DEFAULT_MODEL_SEARCH_DIR)]
    found = []
    seen = set()
    for d in search_dirs:
        try:
            if not d.exists() or not d.is_dir():
                continue
            for p in d.iterdir():
                try:
                    if p.is_file() and p.suffix.lower() in SUPPORTED_CAD_EXTENSIONS:
                        key = str(p.resolve()).lower()
                        if key not in seen:
                            seen.add(key)
                            found.append(p)
                except Exception:
                    pass
        except Exception:
            pass

    if not found:
        raise RuntimeError(
            "没有找到 AUV CAD。请使用 --model 传入模型路径，"
            "或设置环境变量 AUV_MODEL_PATH。"
        )

    found.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    return found[0].resolve()


INPUT_MODEL = resolve_input_model()
# 兼容 v102 内部旧变量名；它现在可以是 x_t/STEP/IGES/SAT/SCDOC。
INPUT_XT = str(INPUT_MODEL)
MODEL_NAME = INPUT_MODEL.stem


def resolve_work_dir():
    """工作目录优先级：--workdir > 旧位置参数 > AUV_WORK_DIR > 默认/模型目录规则。"""
    if getattr(RUNTIME_ARGS, "workdir", None):
        return str(Path(str(RUNTIME_ARGS.workdir).strip().strip('"')))

    if getattr(RUNTIME_ARGS, "legacy_workdir", None):
        return str(Path(str(RUNTIME_ARGS.legacy_workdir).strip().strip('"')))

    env_work = os.environ.get("AUV_WORK_DIR", "").strip().strip('"')
    if env_work:
        return str(Path(env_work))

    try:
        if Path(INPUT_MODEL).resolve() == Path(DEFAULT_MODEL_PATH).resolve():
            return str(Path(DEFAULT_WORK_DIR))
    except Exception:
        pass

    return str(Path(WORK_ROOT) / MODEL_NAME)


WORK_DIR = resolve_work_dir()
Path(WORK_DIR).mkdir(parents=True, exist_ok=True)

OUTPUT_SCDOC = str(Path(WORK_DIR) / f"{MODEL_NAME}.scdoc")
SURFACE_LABELS_PATH = str(Path(WORK_DIR) / f"{MODEL_NAME}_surface_labels.txt")
PART_BBOX_PATH = str(Path(WORK_DIR) / f"{MODEL_NAME}_part_bounding_boxes.txt")
PART_GEOMETRY_METRICS_PATH = str(Path(WORK_DIR) / f"{MODEL_NAME}_part_geometry_metrics.txt")
TARGET_MESH_SIZE_PATH = str(Path(WORK_DIR) / f"{MODEL_NAME}_target_mesh_size.txt")
TARGET_MESH_SIZE_DIAG_PATH = str(Path(WORK_DIR) / f"{MODEL_NAME}_target_mesh_size_diagnostics.txt")
ADAPTIVE_SUMMARY_PATH = str(Path(WORK_DIR) / f"{MODEL_NAME}_adaptive_mesh_summary.json")

TEMP_SCRIPT_DIR = WORK_DIR
SPACECLAIM_TIMEOUT_SEC = 900
AFTER_SAVE_GRACE_SEC = 8
TERMINATE_SPACECLAIM_AFTER_SAVE = True


# ============================================================
# 1. SpaceClaim 自动计算域参数
# ============================================================

USE_AUTO_DOMAIN = True
# 通用 AUV 默认：最长包围盒方向作为航行/来流方向。
# 极特殊模型可通过环境变量 AUV_FLOW_AXIS=x/y/z 覆盖，而无需改代码。
_env_axis = os.environ.get("AUV_FLOW_AXIS", "").strip().lower()
AUTO_FLOW_AXIS_BY_LONGEST = _env_axis not in ["x", "y", "z"]
FORCED_FLOW_AXIS = _env_axis if _env_axis in ["x", "y", "z"] else "x"

# 计算域按 AUV 自身尺度缩放，不使用固定 mm 坐标。
UPSTREAM_LENGTH_RATIO = 3.0
DOWNSTREAM_LENGTH_RATIO = 6.0
CROSS_HALF_WIDTH_RATIO = 5.0

# 仅在 USE_AUTO_DOMAIN=False 时使用；通用模式默认不会走这里。
MANUAL_X_MIN = -1.0
MANUAL_X_MAX = 1.0
MANUAL_Y_MIN = -1.0
MANUAL_Y_MAX = 1.0
MANUAL_Z_MIN = -1.0
MANUAL_Z_MAX = 1.0

PART_BOX_MARGIN_RATIO = 0.02


# ============================================================
# 2. 几何特征驱动的自适应 Target Mesh Size
# ============================================================

# 粗/中/细网格统一倍率：粗 1.30，中 1.00，细 0.70。
MESH_LEVEL_SCALE = float(os.environ.get("AUV_MESH_LEVEL_SCALE", "1.0"))

ROLE_CN = {
    "dominant_body": "主体/最大承载体",
    "body_segment": "主体分段/壳体段",
    "thin_complex_appendage": "薄壁/复杂附体",
    "slender_appendage": "细长附体/轴杆",
    "regular_appendage": "普通附体",
}

# 以下全部是无量纲比例，不绑定某一艘 AUV 的毫米尺度。
MAIN_L_FRACTION = 0.010       # 主体目标尺寸 <= 1% 总长
MAIN_D_FRACTION = 0.100       # 主体目标尺寸 <= 10% 横向参考尺度
GLOBAL_FLOOR_D_FRACTION = 0.00050
GLOBAL_FLOOR_L_FRACTION = 0.00005

REGULAR_MAIN_FACTOR = 0.50
REGULAR_DMIN_DIV = 8.0
REGULAR_FEATURE_FACTOR = 0.80
REGULAR_GAP_FACTOR = 0.40

SLENDER_MAIN_FACTOR = 0.30
SLENDER_DMIN_DIV = 10.0
SLENDER_FEATURE_FACTOR = 0.60
SLENDER_GAP_FACTOR = 0.35

COMPLEX_MAIN_FACTOR = 0.20
COMPLEX_DMIN_DIV = 15.0
COMPLEX_EDGE_FACTOR = 0.40
COMPLEX_THICKNESS_FACTOR = 0.40
COMPLEX_FEATURE_FACTOR = 0.40
COMPLEX_GAP_FACTOR = 0.30

# 几何角色判据同样用相对/无量纲指标。
SLENDER_ASPECT_RATIO = 4.0
COMPLEXITY_SCORE_THRESHOLD = 6.0
THIN_FEATURE_RATIO_THRESHOLD = 0.20

# ============================================================
# 3. 嵌入式 SpaceClaim 脚本：v100 单位修正版
# ============================================================

_EMBEDDED_SC_SCRIPT = '\n# Python Script, API Version = V24\nimport os\nimport math\n\n# ============================================================\n# 0. 参数区\n# ============================================================\n\nINPUT_PATH = r"C:\\Users\\Administrator\\Desktop\\shixi\\new\\auv320.x_t"\nSAVE_PATH  = r"C:\\Users\\Administrator\\Desktop\\shixi\\zidonghua\\auv\\auv_320.scdoc"\nSURFACE_LABELS_PATH = r"C:\\Users\\Administrator\\Desktop\\shixi\\zidonghua\\auv\\auv_320_surface_labels.txt"\nPART_BBOX_PATH = r"C:\\Users\\Administrator\\Desktop\\shixi\\zidonghua\\auv\\auv_320_part_bounding_boxes.txt"\nPART_GEOMETRY_METRICS_PATH = r"C:\\Users\\Administrator\\Desktop\\shixi\\zidonghua\\auv\\auv_320_part_geometry_metrics.txt"\n\nUSE_AUTO_DOMAIN = True\n\n# 自动修正流向轴：\n# True  = 根据模型包围盒最长方向自动作为 inlet/outlet 方向；\n# False = 强制使用 SpaceClaim 全局 X 轴作为流向。本模型必须使用 False。\nAUTO_FLOW_AXIS_BY_LONGEST = False\n\n# 当 AUTO_FLOW_AXIS_BY_LONGEST=False 时使用。\nFORCED_FLOW_AXIS = "x"\n\n\nUPSTREAM_LENGTH_RATIO = 3.0\nDOWNSTREAM_LENGTH_RATIO = 6.0\nCROSS_HALF_WIDTH_RATIO = 5.0\n\nMANUAL_X_MIN = -3200.0\nMANUAL_X_MAX = 12800.0\nMANUAL_Y_MIN = -1600.0\nMANUAL_Y_MAX =  1600.0\nMANUAL_Z_MIN = -1600.0\nMANUAL_Z_MAX =  1600.0\n\nTOL_MM = 1.0e-2\n\n# 小部件匹配余量。若 shaft/propeller 识别不到，可调大到 0.05；\n# 若误选主体尾部，可调小到 0.01。\nPART_BOX_MARGIN_RATIO = 0.02\n\n\n# ============================================================\n# 1. 基础函数\n# ============================================================\n\ndef msg(text):\n   try:\n      print(text)\n   except:\n      pass\n\n\ndef u(value_mm):\n   try:\n      return MM(value_mm)\n   except:\n      return value_mm\n\n\nTOL_U = u(TOL_MM)\n\n\ndef get_master(obj):\n   try:\n      return obj.GetMaster()\n   except:\n      return obj\n\n\ndef safe_get_name(obj):\n   names = []\n\n   for item in [obj, get_master(obj)]:\n      try:\n         names.append(str(item.GetName()))\n      except:\n         pass\n\n      try:\n         names.append(str(item.Name))\n      except:\n         pass\n\n      try:\n         names.append(str(item.Parent.GetName()))\n      except:\n         pass\n\n      try:\n         names.append(str(item.Parent.Name))\n      except:\n         pass\n\n      try:\n         names.append(str(item.Component.GetName()))\n      except:\n         pass\n\n      try:\n         names.append(str(item.Component.Name))\n      except:\n         pass\n\n   clean = []\n\n   for n in names:\n      if n is None:\n         continue\n      if n == "":\n         continue\n      if n == "None":\n         continue\n      if n not in clean:\n         clean.append(n)\n\n   if len(clean) == 0:\n      return "unknown"\n\n   return " | ".join(clean)\n\n\nBODY_NAME_TABLE = []\n\n\ndef add_body_name_record(body, name):\n   global BODY_NAME_TABLE\n\n   if body is None:\n      return\n\n   for i in range(len(BODY_NAME_TABLE)):\n      b0, name0 = BODY_NAME_TABLE[i]\n      if b0 == body:\n         if name not in name0:\n            BODY_NAME_TABLE[i] = (b0, name0 + " | " + name)\n         return\n\n   BODY_NAME_TABLE.append((body, name))\n\n\ndef get_recorded_body_name(body):\n   names = [safe_get_name(body)]\n\n   for b0, name0 in BODY_NAME_TABLE:\n      try:\n         if b0 == body:\n            names.append(name0)\n      except:\n         pass\n\n   clean = []\n\n   for n in names:\n      if n is None:\n         continue\n      if n == "":\n         continue\n      if n == "None":\n         continue\n      if n not in clean:\n         clean.append(n)\n\n   return " | ".join(clean)\n\n\ndef set_body_name(body, name):\n   try:\n      body.Name = name\n      return True\n   except:\n      pass\n\n   try:\n      body.SetName(name)\n      return True\n   except:\n      pass\n\n   try:\n      get_master(body).Name = name\n      return True\n   except:\n      return False\n\n\ndef clear_current_document():\n   try:\n      ClearAll()\n      msg("已清空当前文档")\n   except Exception as e:\n      msg("ClearAll 未执行，继续运行。原因: " + str(e))\n\n\ndef open_geometry(file_path):\n   if not os.path.exists(file_path):\n      raise Exception("找不到原始模型文件: " + file_path)\n\n   try:\n      DocumentOpen.Execute(file_path, FileSettings1)\n   except:\n      DocumentOpen.Execute(file_path)\n\n   msg("已导入模型: " + file_path)\n\n\ndef save_scdoc(save_path):\n   folder = os.path.dirname(save_path)\n\n   if not os.path.exists(folder):\n      os.makedirs(folder)\n\n   try:\n      if os.path.exists(save_path):\n         os.remove(save_path)\n         msg("已删除旧文件: " + save_path)\n   except Exception as e:\n      msg("旧文件删除失败，可能被占用: " + str(e))\n\n   try:\n      DocumentSave.Execute(save_path)\n   except:\n      try:\n         DocumentSave.Execute(save_path, FileSettings1)\n      except:\n         raise\n\n   msg("已保存文件: " + save_path)\n\n\ndef unique_append(body_list, body):\n   if body is None:\n      return\n\n   for b in body_list:\n      if b == body:\n         return\n\n   body_list.append(body)\n\n\ndef get_all_bodies():\n   """\n   同时从 RootPart、Components、MainPart 中抓取 body，并记录组件名。\n   """\n   global BODY_NAME_TABLE\n   bodies = []\n\n   try:\n      for b in GetRootPart().GetAllBodies():\n         unique_append(bodies, b)\n         add_body_name_record(b, safe_get_name(b))\n   except Exception as e:\n      msg("GetRootPart().GetAllBodies() 未成功: " + str(e))\n\n   try:\n      for b in GetRootPart().Bodies:\n         unique_append(bodies, b)\n         add_body_name_record(b, safe_get_name(b))\n   except:\n      pass\n\n   try:\n      comps = GetRootPart().GetAllComponents()\n\n      for comp in comps:\n         comp_name = safe_get_name(comp)\n\n         try:\n            for b in comp.Content.Bodies:\n               unique_append(bodies, b)\n               add_body_name_record(b, comp_name + " | " + safe_get_name(b))\n         except:\n            pass\n\n         try:\n            for b in comp.Content.GetAllBodies():\n               unique_append(bodies, b)\n               add_body_name_record(b, comp_name + " | " + safe_get_name(b))\n         except:\n            pass\n   except:\n      pass\n\n   try:\n      part = Window.ActiveWindow.Document.MainPart\n      for b in part.Bodies:\n         unique_append(bodies, b)\n         add_body_name_record(b, safe_get_name(b))\n   except:\n      pass\n\n   try:\n      part = Window.ActiveWindow.Document.MainPart\n      for b in part.GetAllBodies():\n         unique_append(bodies, b)\n         add_body_name_record(b, safe_get_name(b))\n   except:\n      pass\n\n   return bodies\n\n\ndef get_body_faces(body):\n   try:\n      return list(body.Faces)\n   except:\n      pass\n\n   try:\n      return list(get_master(body).Faces)\n   except:\n      pass\n\n   try:\n      return list(get_master(body).Shape.Faces)\n   except:\n      return []\n\n\ndef is_solid_body(body):\n   try:\n      return get_master(body).Shape.IsClosed\n   except:\n      return True\n\n\ndef get_body_box(body):\n   try:\n      return get_master(body).Shape.GetBoundingBox(Matrix.Identity)\n   except:\n      pass\n\n   try:\n      return body.Shape.GetBoundingBox(Matrix.Identity)\n   except:\n      pass\n\n   raise Exception("无法获取 body bounding box: " + get_recorded_body_name(body))\n\n\ndef body_box_tuple(body):\n   box = get_body_box(body)\n\n   return (\n      box.MinCorner.X,\n      box.MaxCorner.X,\n      box.MinCorner.Y,\n      box.MaxCorner.Y,\n      box.MinCorner.Z,\n      box.MaxCorner.Z\n   )\n\n\ndef get_box_dims(box_tuple):\n   xmin, xmax, ymin, ymax, zmin, zmax = box_tuple\n   return abs(xmax - xmin), abs(ymax - ymin), abs(zmax - zmin)\n\n\ndef body_dims(body):\n   return get_box_dims(body_box_tuple(body))\n\n\ndef body_measure(body):\n   try:\n      return abs(get_master(body).Shape.Volume)\n   except:\n      pass\n\n   try:\n      lx, ly, lz = body_dims(body)\n      return lx * ly * lz\n   except:\n      return 0.0\n\n\ndef body_center(body):\n   xmin, xmax, ymin, ymax, zmin, zmax = body_box_tuple(body)\n\n   return (\n      0.5 * (xmin + xmax),\n      0.5 * (ymin + ymax),\n      0.5 * (zmin + zmax)\n   )\n\n\ndef delete_body(body):\n   try:\n      Delete.Execute(BodySelection.Create(body))\n      return True\n   except:\n      pass\n\n   try:\n      Delete.Execute(Selection.Create(body))\n      return True\n   except:\n      return False\n\n\ndef make_body_selection_from_list(bodies):\n   if bodies is None or len(bodies) == 0:\n      raise Exception("空 body 列表，无法创建选择集")\n\n   if len(bodies) == 1:\n      try:\n         return BodySelection.Create(bodies[0])\n      except:\n         return Selection.Create(bodies[0])\n\n   try:\n      return BodySelection.Create(*bodies)\n   except:\n      pass\n\n   try:\n      return BodySelection.Create(bodies)\n   except:\n      pass\n\n   try:\n      return Selection.Create(bodies)\n   except:\n      pass\n\n   raise Exception("无法创建 BodySelection")\n\n\ndef print_body_debug_info(title):\n   bodies = get_all_bodies()\n\n   msg("-" * 70)\n   msg(title)\n   msg("当前 Body 数量 = " + str(len(bodies)))\n\n   for i, b in enumerate(bodies):\n      name = get_recorded_body_name(b)\n\n      try:\n         closed = is_solid_body(b)\n      except:\n         closed = "unknown"\n\n      try:\n         face_count = len(get_body_faces(b))\n      except:\n         face_count = "unknown"\n\n      try:\n         lx, ly, lz = body_dims(b)\n      except:\n         lx, ly, lz = 0, 0, 0\n\n      try:\n         cx, cy, cz = body_center(b)\n      except:\n         cx, cy, cz = 0, 0, 0\n\n      msg(\n         "Body[" + str(i) + "] name = " + str(name) +\n         " | closed = " + str(closed) +\n         " | faces = " + str(face_count) +\n         " | lx = " + str(lx) +\n         " | ly = " + str(ly) +\n         " | lz = " + str(lz) +\n         " | cx = " + str(cx) +\n         " | measure = " + str(body_measure(b))\n      )\n\n   msg("-" * 70)\n\n\n# ============================================================\n# 2. 部件分类\n# ============================================================\n\n\ndef sanitize_name(raw_name):\n   if raw_name is None:\n      return "other"\n\n   name = str(raw_name)\n   parts = name.split("|")\n   best = ""\n\n   for p in parts:\n      p = p.strip()\n      if p and p.lower() not in ["none", "unknown"]:\n         best = p\n         break\n\n   if best == "":\n      best = name.strip()\n\n   out = ""\n   for ch in best:\n      if ch.isalnum():\n         out += ch\n      else:\n         out += "_"\n\n   while "__" in out:\n      out = out.replace("__", "_")\n\n   out = out.strip("_").lower()\n\n   if out == "":\n      out = "other"\n\n   return out\n\n\ndef remove_body_from_categories(categories, body):\n   for cat in categories:\n      if body in categories[cat]:\n         categories[cat].remove(body)\n\n\ndef get_digits_after_keyword(raw_name, keyword):\n   low = str(raw_name).lower()\n   key = str(keyword).lower()\n   pos = low.find(key)\n\n   if pos < 0:\n      return ""\n\n   tail = low[pos + len(key):]\n   digits = ""\n   reading = False\n\n   for ch in tail:\n      if ch.isdigit():\n         digits += ch\n         reading = True\n      else:\n         if not reading and ch in ["_", "-", " ", "<", "(", "[", "{"]:\n            continue\n         break\n\n   if digits == "":\n      return ""\n\n   try:\n      return str(int(digits))\n   except:\n      return digits\n\n\ndef numbered_category(base, raw_name, keys):\n   if base == "auv_body":\n      return "auv_body"\n\n   for k in keys:\n      num = get_digits_after_keyword(raw_name, k)\n      if num != "":\n         return base + num\n\n   return base\n\n\ndef canonical_category_from_name(name):\n   low = str(name).lower()\n   raw = str(name)\n\n   # ASCII-only keyword list for SpaceClaim IronPython stability.\n   # Part-number rules:\n   # fin1/fin2 -> fin1/fin2\n   # shaft1/shaft2 -> shaft1/shaft2\n   # propeller1 -> propeller1\n   keyword_map = [\n      ("propeller", ["propeller", "prop", "blade", "screw"]),\n      ("shaft", ["shaft", "strut", "support", "connector", "bracket", "link", "mount", "tail_shaft", "axis"]),\n      ("duct", ["duct", "thruster", "nozzle", "shroud"]),\n      ("fin", ["fin", "rudder", "wing", "stabilizer", "control_surface"]),\n      ("sonar", ["sonar_dome", "sonar", "sensor", "fairing", "dome", "transducer"]),\n      ("antenna", ["antenna", "mast"]),\n      ("payload", ["payload_bay", "payload"]),\n      ("auv_body", ["auv_body", "hull", "main_body", "main_hull", "main", "body", "nose", "tail"]),\n   ]\n\n   for item in keyword_map:\n      base = item[0]\n      keys = item[1]\n\n      for k in keys:\n         if str(k).lower() in low:\n            return numbered_category(base, raw, keys)\n\n   return None\n\n\ndef split_category_number(cat):\n   low = str(cat).lower()\n   bases = ["shaft", "propeller", "duct", "fin", "sonar", "antenna", "payload"]\n\n   for base in bases:\n      if low == base:\n         return base, 0\n\n      if low.startswith(base):\n         tail = low[len(base):]\n         if tail != "":\n            ok = True\n            for ch in tail:\n               if not ch.isdigit():\n                  ok = False\n                  break\n            if ok:\n               try:\n                  return base, int(tail)\n               except:\n                  return base, 999999\n\n   if low.startswith("other"):\n      tail = low.replace("other", "", 1)\n      if tail != "":\n         ok = True\n         for ch in tail:\n            if not ch.isdigit():\n               ok = False\n               break\n         if ok:\n            try:\n               return "other", int(tail)\n            except:\n               return "other", 999999\n\n   return low, 0\n\n\ndef make_unique_category_name(cat, categories):\n   if cat == "auv_body":\n      return cat\n\n   if cat not in categories:\n      return cat\n\n   base, number = split_category_number(cat)\n\n   i = 1\n   while True:\n      candidate = base + str(i)\n      if candidate not in categories:\n         return candidate\n      i += 1\n\n\ndef is_generic_part_name_candidate(name):\n   """过滤 SpaceClaim/导入器常见的泛化名称，优先保留用户模型自身部件名。"""\n   if name is None:\n      return True\n\n   low = str(name).strip().lower()\n\n   if low == "" or low == "none" or low == "unknown":\n      return True\n\n   generic_words = [\n      "body", "solid", "surface", "sheet", "part", "component",\n      "designbody", "root", "master", "shape", "auv320", "auv_320"\n   ]\n\n   if low in generic_words:\n      return True\n\n   for prefix in ["body", "solid", "surface", "sheet", "part", "component"]:\n      if low.startswith(prefix):\n         tail = low[len(prefix):]\n         if tail == "":\n            return True\n         ok = True\n         for ch in tail:\n            if not ch.isdigit() and ch not in ["_", "-", " "]:\n               ok = False\n               break\n         if ok:\n            return True\n\n   return False\n\n\ndef sanitize_original_part_name(raw_name, fallback_index):\n   """\n   将模型原始部件名转为 Fluent/Named Selection 更稳定的名称。\n   例如：a -> a，aaa -> aaa，part-01 -> part_01。\n   """\n   candidates = []\n\n   try:\n      for p in str(raw_name).split("|"):\n         item = p.strip()\n         if item and item not in candidates:\n            candidates.append(item)\n   except:\n      pass\n\n   best = ""\n\n   for item in candidates:\n      if not is_generic_part_name_candidate(item):\n         best = item\n         break\n\n   if best == "" and len(candidates) > 0:\n      best = candidates[0]\n\n   if best == "":\n      best = "part_" + str(fallback_index)\n\n   out = ""\n   for ch in best:\n      o = ord(ch)\n      if (o >= 48 and o <= 57) or (o >= 65 and o <= 90) or (o >= 97 and o <= 122):\n         out += ch\n      elif ch == "_":\n         out += "_"\n      else:\n         out += "_"\n\n   while "__" in out:\n      out = out.replace("__", "_")\n\n   out = out.strip("_")\n\n   if out == "":\n      out = "part_" + str(fallback_index)\n\n   try:\n      first = out[0]\n      if first.isdigit():\n         out = "part_" + out\n   except:\n      out = "part_" + str(fallback_index)\n\n   return out\n\n\ndef make_unique_original_label(base_name, used_names):\n   if base_name not in used_names:\n      return base_name\n\n   i = 2\n   while True:\n      candidate = base_name + "_" + str(i)\n      if candidate not in used_names:\n         return candidate\n      i += 1\n\n\ndef classify_parts(bodies):\n   """\n   v76 通用命名版：不再把部件强制识别为 auv_body/shaft/propeller。\n\n   规则：\n      1. 读取模型导入后的原始 body/component 名称；\n      2. 每个原始部件名作为一个面命名 label；\n      3. 如果模型有 a、b、c 三个部件，则最终在流体域内壁创建 a、b、c 三组面命名；\n      4. 重名时自动追加 _2、_3，避免 Named Selection 冲突。\n   """\n   categories = {}\n   used_names = []\n\n   if len(bodies) == 0:\n      raise Exception("no body, cannot classify")\n\n   index = 1\n   for b in bodies:\n      raw_name = get_recorded_body_name(b)\n      label = sanitize_original_part_name(raw_name, index)\n      label = make_unique_original_label(label, used_names)\n      used_names.append(label)\n\n      categories[label] = [b]\n\n      msg("原始部件命名映射: " + str(raw_name) + " -> " + str(label))\n\n      index += 1\n\n   msg("-" * 70)\n   msg("v76 原始部件名称分组结果:")\n\n   for cat in sorted(categories.keys()):\n      msg(cat + " body count = " + str(len(categories[cat])))\n\n      for b in categories[cat]:\n         try:\n            lx, ly, lz = body_dims(b)\n            xmin, xmax, ymin, ymax, zmin, zmax = body_box_tuple(b)\n            name = get_recorded_body_name(b)\n\n            msg(\n               "   " + name +\n               " | label=" + cat +\n               " | lx=" + str(lx) +\n               " | ly=" + str(ly) +\n               " | lz=" + str(lz) +\n               " | x=[" + str(xmin) + "," + str(xmax) + "]"\n            )\n         except Exception as e:\n            msg("   " + get_recorded_body_name(b) + " | label=" + cat + " | debug failed: " + str(e))\n\n   msg("-" * 70)\n\n   return categories\n\n\ndef category_order_key(cat):\n   """\n   Boolean 扣除顺序采用“小部件优先，auv_body 最后”。\n\n   原先 auv_body 先扣，会提前形成主体内壁面。\n   后续再扣 shaft / duct 时，接触区域会把主体面重新分裂，\n   于是 shaft 可能变成真实的 auv_body 面，duct 可能带上 shaft / propeller 面。\n\n   本顺序先扣更具体的小部件，再扣外包/相邻部件，最后扣主体：\n      propeller -> shaft -> duct -> fin -> sonar -> antenna -> payload -> other -> auv_body\n   """\n   priority = ["propeller", "shaft", "duct", "fin", "sonar", "antenna", "payload", "other", "auv_body"]\n   low = str(cat).lower()\n   b, n = split_category_number(low)\n\n   if low == "auv_body":\n      return (999, 0, low)\n\n   for i in range(len(priority)):\n      base = priority[i]\n\n      if base == "other":\n         if b == "other":\n            return (i, n, low)\n      elif b == base:\n         return (i, n, low)\n\n   return (998, 999, low)\n\n\ndef ordered_category_names(categories):\n   """\n   v76 通用命名版 Boolean 顺序。\n   不再依赖 auv_body/propeller/shaft 等关键词；按部件包围盒/体量从小到大扣除。\n   这样小附体优先，最大主体最后，尽量继承原来“小部件优先、主体最后”的稳定思路。\n   """\n   def category_size(cat):\n      try:\n         bodies = categories[cat]\n         total = 0.0\n         max_box = 0.0\n         for b in bodies:\n            total += body_measure(b)\n            lx, ly, lz = body_dims(b)\n            m = max(abs(lx), abs(ly), abs(lz))\n            if m > max_box:\n               max_box = m\n         return (total, max_box, str(cat).lower())\n      except:\n         return (1.0e99, 1.0e99, str(cat).lower())\n\n   result = []\n\n   for cat in sorted(categories.keys(), key=category_size):\n      if len(categories[cat]) > 0:\n         result.append(cat)\n\n   return result\n\n\n# ============================================================\n# 3. 计算域\n# ============================================================\n\ndef all_bodies_bbox(bodies):\n   xmin, xmax, ymin, ymax, zmin, zmax = body_box_tuple(bodies[0])\n\n   for b in bodies[1:]:\n      bxmin, bxmax, bymin, bymax, bzmin, bzmax = body_box_tuple(b)\n\n      xmin = min(xmin, bxmin)\n      xmax = max(xmax, bxmax)\n\n      ymin = min(ymin, bymin)\n      ymax = max(ymax, bymax)\n\n      zmin = min(zmin, bzmin)\n      zmax = max(zmax, bzmax)\n\n   return xmin, xmax, ymin, ymax, zmin, zmax\n\n\ndef compute_domain_from_auv(bodies):\n   """\n   根据模型真实包围盒边界生成计算域。\n\n   推荐标准：\n      L = xmax - xmin                     # 流向尺度\n      D = ymax - ymin\n      H = zmax - zmin\n      D_ref = max(D, H)                   # 横向统一参考尺度\n\n   若流向为 X：\n      x_min = 模型 xmin - 3L\n      x_max = 模型 xmax + 6L\n      y_min = 模型 ymin - 5D_ref\n      y_max = 模型 ymax + 5D_ref\n      z_min = 模型 zmin - 5D_ref\n      z_max = 模型 zmax + 5D_ref\n\n   这样计算域相对于 AUV 实际外包络边界外扩，而不是相对于全局原点外扩。\n   """\n   xmin, xmax, ymin, ymax, zmin, zmax = all_bodies_bbox(bodies)\n\n   length_x = abs(xmax - xmin)\n   length_y = abs(ymax - ymin)\n   length_z = abs(zmax - zmin)\n\n   if length_x <= 0 and length_y <= 0 and length_z <= 0:\n      raise Exception("模型包围盒尺寸异常，无法自动生成计算域")\n\n   dims = {\n      "x": length_x,\n      "y": length_y,\n      "z": length_z\n   }\n\n   mins = {\n      "x": xmin,\n      "y": ymin,\n      "z": zmin\n   }\n\n   maxs = {\n      "x": xmax,\n      "y": ymax,\n      "z": zmax\n   }\n\n   # 防止某方向极薄导致 0 尺寸。\n   positive_lengths = []\n\n   for k in ["x", "y", "z"]:\n      if dims[k] > 0:\n         positive_lengths.append(dims[k])\n\n   if len(positive_lengths) == 0:\n      raise Exception("无法识别模型有效尺寸")\n\n   min_positive = min(positive_lengths)\n\n   for k in ["x", "y", "z"]:\n      if dims[k] <= 0:\n         dims[k] = min_positive\n\n   if AUTO_FLOW_AXIS_BY_LONGEST:\n      flow_axis = "x"\n\n      if dims["y"] > dims[flow_axis]:\n         flow_axis = "y"\n\n      if dims["z"] > dims[flow_axis]:\n         flow_axis = "z"\n   else:\n      flow_axis = str(FORCED_FLOW_AXIS).lower()\n\n      if flow_axis not in ["x", "y", "z"]:\n         flow_axis = "x"\n\n   cross_axes = []\n\n   for k in ["x", "y", "z"]:\n      if k != flow_axis:\n         cross_axes.append(k)\n\n   flow_length = dims[flow_axis]\n   cross_ref = max(dims[cross_axes[0]], dims[cross_axes[1]])\n\n   extents = {}\n\n   # 流向：按模型真实前后边界外扩。\n   extents[flow_axis] = [\n      mins[flow_axis] - UPSTREAM_LENGTH_RATIO * flow_length,\n      maxs[flow_axis] + DOWNSTREAM_LENGTH_RATIO * flow_length\n   ]\n\n   # 横向：两个横向方向统一用 D_ref = max(D, H) 外扩。\n   for axis in cross_axes:\n      extents[axis] = [\n         mins[axis] - CROSS_HALF_WIDTH_RATIO * cross_ref,\n         maxs[axis] + CROSS_HALF_WIDTH_RATIO * cross_ref\n      ]\n\n   dxmin, dxmax = extents["x"]\n   dymin, dymax = extents["y"]\n   dzmin, dzmax = extents["z"]\n\n   msg("-" * 70)\n   msg("按模型真实包围盒边界生成计算域:")\n   msg("模型 bbox x = [" + str(xmin) + ", " + str(xmax) + "]，Lx = " + str(length_x))\n   msg("模型 bbox y = [" + str(ymin) + ", " + str(ymax) + "]，Ly = " + str(length_y))\n   msg("模型 bbox z = [" + str(zmin) + ", " + str(zmax) + "]，Lz = " + str(length_z))\n   msg("AUTO_FLOW_AXIS_BY_LONGEST = " + str(AUTO_FLOW_AXIS_BY_LONGEST))\n   msg("实际 inlet/outlet 流向轴 = " + str(flow_axis))\n   msg("流向尺度 flow_length = " + str(flow_length))\n   msg("横向统一尺度 D_ref = max(两个横向包围盒尺寸) = " + str(cross_ref))\n   msg("上游外扩倍数 = " + str(UPSTREAM_LENGTH_RATIO))\n   msg("下游外扩倍数 = " + str(DOWNSTREAM_LENGTH_RATIO))\n   msg("横向半宽外扩倍数 = " + str(CROSS_HALF_WIDTH_RATIO))\n\n   if flow_axis == "x":\n      msg("当前采用全局 X 轴作为流向：inlet = x_min，outlet = x_max")\n      msg("计算域公式：")\n      msg("   x_min = bbox_xmin - " + str(UPSTREAM_LENGTH_RATIO) + " * Lx")\n      msg("   x_max = bbox_xmax + " + str(DOWNSTREAM_LENGTH_RATIO) + " * Lx")\n      msg("   y/z 根据 D_ref = max(Ly, Lz) 外扩")\n   else:\n      msg("当前流向不是 X，已按实际流向轴和横向轴自动外扩。")\n\n   msg("domain x = " + str(dxmin) + " ~ " + str(dxmax))\n   msg("domain y = " + str(dymin) + " ~ " + str(dymax))\n   msg("domain z = " + str(dzmin) + " ~ " + str(dzmax))\n   msg("-" * 70)\n\n   return dxmin, dxmax, dymin, dymax, dzmin, dzmax, flow_length, flow_axis\n\n\nDOMAIN_X_MIN_U = None\nDOMAIN_X_MAX_U = None\nDOMAIN_Y_MIN_U = None\nDOMAIN_Y_MAX_U = None\nDOMAIN_Z_MIN_U = None\nDOMAIN_Z_MAX_U = None\nAUV_LENGTH_U = None\nSELECTED_FLOW_AXIS = None\n\n\ndef create_fluid_domain():\n   p1 = Point.Create(DOMAIN_X_MIN_U, DOMAIN_Y_MIN_U, DOMAIN_Z_MIN_U)\n   p2 = Point.Create(DOMAIN_X_MAX_U, DOMAIN_Y_MAX_U, DOMAIN_Z_MAX_U)\n\n   box_feature = BlockBody.Create(p1, p2)\n   fluid_body = box_feature.CreatedBody\n\n   set_body_name(fluid_body, "fluid")\n\n   msg("已创建流体域 fluid")\n\n   return fluid_body\n\n\n# ============================================================\n# 4. 几何面识别\n# ============================================================\n\ndef close_to(a, b):\n   return abs(a - b) <= TOL_U\n\n\ndef get_face_box(face):\n   try:\n      return face.GetBoundingBox(Matrix.Identity)\n   except:\n      pass\n\n   try:\n      return face.Shape.GetBoundingBox(Matrix.Identity)\n   except:\n      pass\n\n   try:\n      return get_master(face).Shape.GetBoundingBox(Matrix.Identity)\n   except:\n      pass\n\n   raise Exception("无法获取 face bounding box")\n\n\ndef face_box_tuple(face):\n   box = get_face_box(face)\n\n   return (\n      box.MinCorner.X,\n      box.MaxCorner.X,\n      box.MinCorner.Y,\n      box.MaxCorner.Y,\n      box.MinCorner.Z,\n      box.MaxCorner.Z\n   )\n\n\ndef face_on_x(face, x_value):\n   xmin, xmax, ymin, ymax, zmin, zmax = face_box_tuple(face)\n   return close_to(xmin, x_value) and close_to(xmax, x_value)\n\n\ndef face_on_y(face, y_value):\n   xmin, xmax, ymin, ymax, zmin, zmax = face_box_tuple(face)\n   return close_to(ymin, y_value) and close_to(ymax, y_value)\n\n\ndef face_on_z(face, z_value):\n   xmin, xmax, ymin, ymax, zmin, zmax = face_box_tuple(face)\n   return close_to(zmin, z_value) and close_to(zmax, z_value)\n\n\n\n\ndef face_on_axis(face, axis, value):\n   axis = str(axis).lower()\n   if axis == "x":\n      return face_on_x(face, value)\n   if axis == "y":\n      return face_on_y(face, value)\n   if axis == "z":\n      return face_on_z(face, value)\n   return False\n\n\ndef get_domain_minmax_by_axis(axis):\n   axis = str(axis).lower()\n   if axis == "x":\n      return DOMAIN_X_MIN_U, DOMAIN_X_MAX_U\n   if axis == "y":\n      return DOMAIN_Y_MIN_U, DOMAIN_Y_MAX_U\n   if axis == "z":\n      return DOMAIN_Z_MIN_U, DOMAIN_Z_MAX_U\n   return DOMAIN_X_MIN_U, DOMAIN_X_MAX_U\n\ndef is_external_domain_face(face):\n   for axis in ["x", "y", "z"]:\n      amin, amax = get_domain_minmax_by_axis(axis)\n      if face_on_axis(face, axis, amin):\n         return True\n      if face_on_axis(face, axis, amax):\n         return True\n   return False\n\n\ndef face_signature(face):\n   """\n   用 bbox 生成面签名，用于判断 Boolean 后新增面。\n   """\n   xmin, xmax, ymin, ymax, zmin, zmax = face_box_tuple(face)\n\n   scale = 1000000.0\n\n   return (\n      int(round(xmin * scale)),\n      int(round(xmax * scale)),\n      int(round(ymin * scale)),\n      int(round(ymax * scale)),\n      int(round(zmin * scale)),\n      int(round(zmax * scale))\n   )\n\n\ndef get_internal_face_signatures(fluid_body):\n   sigs = {}\n\n   for f in get_body_faces(fluid_body):\n      try:\n         if is_external_domain_face(f):\n            continue\n\n         sigs[face_signature(f)] = True\n      except:\n         pass\n\n   return sigs\n\n\n# ============================================================\n# 5. Boolean\n# ============================================================\n\ndef find_current_fluid_body():\n   bodies = get_all_bodies()\n\n   if len(bodies) == 0:\n      raise Exception("当前模型没有 body，无法识别 fluid")\n\n   largest = None\n   largest_value = -1.0\n\n   for b in bodies:\n      value = body_measure(b)\n\n      if value > largest_value:\n         largest_value = value\n         largest = b\n\n   if largest is None:\n      raise Exception("无法识别 fluid body")\n\n   set_body_name(largest, "fluid")\n\n   return largest\n\n\ndef cleanup_keep_only_fluid(fluid_body):\n   fluid_body = find_current_fluid_body()\n\n   for b in list(get_all_bodies()):\n      if b != fluid_body:\n         delete_body(b)\n\n   set_body_name(fluid_body, "fluid")\n\n   return fluid_body\n\n\n\nPROCESSED_CUTTER_BOX_RECORDS = []\n\n\ndef register_processed_cutter_box(category_name, cutter_bodies, face_count):\n   """\n   记录已经成功扣除并创建面的部件 bbox。\n   后续较大的部件（例如 duct、auv_body）过滤时，用这些 bbox 排除已经属于\n   propeller / shaft 等小部件的面，避免一个面被归到多个零件。\n   """\n   global PROCESSED_CUTTER_BOX_RECORDS\n\n   if category_name == "auv_body":\n      return\n\n   if face_count <= 0:\n      return\n\n   try:\n      box = union_body_box_tuple(cutter_bodies)\n      lx, ly, lz = get_box_dims(box)\n      max_len = max(lx, ly, lz)\n      margin = max_len * PART_BOX_MARGIN_RATIO\n\n      try:\n         if margin < TOL_U * 30.0:\n            margin = TOL_U * 30.0\n      except:\n         pass\n\n      box_expanded = expand_box(box, margin)\n      PROCESSED_CUTTER_BOX_RECORDS.append((category_name, box_expanded))\n      msg("已记录已处理部件 bbox: " + category_name + ", margin=" + str(margin))\n\n   except Exception as e:\n      msg("记录已处理部件 bbox 失败: " + category_name + " | " + str(e))\n\n\ndef face_center_inside_processed_box(face):\n   """\n   判断面中心是否落在已处理的小部件 bbox 中。\n   用于防止 duct 吃到 shaft / propeller 面，也防止 auv_body 吃到小部件面。\n   """\n   try:\n      fbox = face_box_tuple(face)\n      fcenter = center_of_box_tuple(fbox)\n\n      for item in PROCESSED_CUTTER_BOX_RECORDS:\n         cat, pbox = item\n         if point_inside_box_tuple(fcenter, pbox):\n            return True, cat\n\n      return False, ""\n\n   except:\n      return False, ""\n\n\ndef union_body_box_tuple(bodies):\n   if bodies is None or len(bodies) == 0:\n      raise Exception("empty body list for union bbox")\n\n   xmin, xmax, ymin, ymax, zmin, zmax = body_box_tuple(bodies[0])\n\n   for b in bodies[1:]:\n      bxmin, bxmax, bymin, bymax, bzmin, bzmax = body_box_tuple(b)\n      xmin = min(xmin, bxmin)\n      xmax = max(xmax, bxmax)\n      ymin = min(ymin, bymin)\n      ymax = max(ymax, bymax)\n      zmin = min(zmin, bzmin)\n      zmax = max(zmax, bzmax)\n\n   return xmin, xmax, ymin, ymax, zmin, zmax\n\n\ndef bbox_intersects(box_a, box_b):\n   axmin, axmax, aymin, aymax, azmin, azmax = box_a\n   bxmin, bxmax, bymin, bymax, bzmin, bzmax = box_b\n\n   if axmax < bxmin:\n      return False\n   if axmin > bxmax:\n      return False\n   if aymax < bymin:\n      return False\n   if aymin > bymax:\n      return False\n   if azmax < bzmin:\n      return False\n   if azmin > bzmax:\n      return False\n\n   return True\n\n\ndef expand_box(box_tuple, margin):\n   xmin, xmax, ymin, ymax, zmin, zmax = box_tuple\n   return (\n      xmin - margin,\n      xmax + margin,\n      ymin - margin,\n      ymax + margin,\n      zmin - margin,\n      zmax + margin\n   )\n\n\ndef center_of_box_tuple(box_tuple):\n   xmin, xmax, ymin, ymax, zmin, zmax = box_tuple\n   return (\n      0.5 * (xmin + xmax),\n      0.5 * (ymin + ymax),\n      0.5 * (zmin + zmax)\n   )\n\n\ndef point_inside_box_tuple(point_tuple, box_tuple):\n   x, y, z = point_tuple\n   xmin, xmax, ymin, ymax, zmin, zmax = box_tuple\n\n   if x < xmin:\n      return False\n   if x > xmax:\n      return False\n   if y < ymin:\n      return False\n   if y > ymax:\n      return False\n   if z < zmin:\n      return False\n   if z > zmax:\n      return False\n\n   return True\n\n\ndef face_box_not_body_big_face(face_box, cutter_box, margin):\n   """\n   保守剔除明显属于 auv_body 的大面。\n   不做过严筛选，避免丢失小部件。\n   """\n   flx, fly, flz = get_box_dims(face_box)\n   clx, cly, clz = get_box_dims(cutter_box)\n\n   if flx < 0:\n      flx = -flx\n   if fly < 0:\n      fly = -fly\n   if flz < 0:\n      flz = -flz\n\n   if clx < 0:\n      clx = -clx\n   if cly < 0:\n      cly = -cly\n   if clz < 0:\n      clz = -clz\n\n   tol = margin * 8.0\n\n   try:\n      if tol < TOL_U * 50.0:\n         tol = TOL_U * 50.0\n   except:\n      pass\n\n   if flx > clx * 4.0 + tol:\n      return False\n   if fly > cly * 4.0 + tol:\n      return False\n   if flz > clz * 4.0 + tol:\n      return False\n\n   return True\n\n\ndef remove_faces_inside_processed_boxes(category_name, faces):\n   """\n   从当前部件候选面里去掉已经属于更早小部件的面。\n   例如 duct 在 shaft / propeller 之后扣除时，容易把这两类面也带进来；\n   这里把面中心落在已处理小部件 bbox 内的面去掉。\n   """\n   if faces is None or len(faces) == 0:\n      return []\n\n   kept = []\n   removed = 0\n   removed_by = {}\n\n   for f in faces:\n      inside, prev_cat = face_center_inside_processed_box(f)\n\n      if inside:\n         removed += 1\n         if prev_cat not in removed_by:\n            removed_by[prev_cat] = 0\n         removed_by[prev_cat] += 1\n      else:\n         kept.append(f)\n\n   if removed > 0:\n      msg(category_name + " 去除已处理小部件重叠面数量 = " + str(removed) + " | " + str(removed_by))\n\n   return kept\n\n\ndef filter_new_internal_faces_by_cutter_box(category_name, cutter_bodies, candidate_faces):\n   """\n   部件新增内壁面过滤。\n\n   关键修正：\n   1. Boolean 顺序已经改为小部件优先，auv_body 最后；\n   2. 对 duct / auv_body 这类较大部件，排除已经属于 propeller / shaft 等小部件的面；\n   3. 非 auv_body 部件仍按当前 cutter bbox 筛选；\n   4. 如果过滤过严会导致 0 面，则回退，避免部件缺失。\n   """\n   if candidate_faces is None or len(candidate_faces) == 0:\n      return []\n\n   # auv_body 最后扣除，只需要排除已经归给小部件的面。\n   if category_name == "auv_body":\n      filtered = remove_faces_inside_processed_boxes(category_name, candidate_faces)\n\n      msg(\n         category_name +\n         " processed-box filter: before=" + str(len(candidate_faces)) +\n         ", after=" + str(len(filtered))\n      )\n\n      if len(filtered) > 0:\n         return filtered\n\n      return candidate_faces\n\n   try:\n      cbox = union_body_box_tuple(cutter_bodies)\n      lx, ly, lz = get_box_dims(cbox)\n      max_len = max(lx, ly, lz)\n\n      margin = max_len * PART_BOX_MARGIN_RATIO\n\n      try:\n         if margin < TOL_U * 30.0:\n            margin = TOL_U * 30.0\n      except:\n         pass\n\n      cbox_expanded = expand_box(cbox, margin)\n\n      bbox_filtered = []\n      center_filtered = []\n      center_size_filtered = []\n\n      for f in candidate_faces:\n         try:\n            fbox = face_box_tuple(f)\n\n            if bbox_intersects(fbox, cbox_expanded):\n               bbox_filtered.append(f)\n\n               fcenter = center_of_box_tuple(fbox)\n\n               if point_inside_box_tuple(fcenter, cbox_expanded):\n                  center_filtered.append(f)\n\n                  if face_box_not_body_big_face(fbox, cbox_expanded, margin):\n                     center_size_filtered.append(f)\n         except:\n            pass\n\n      # 对 duct、sonar 等较大部件，再排除更早处理的 propeller / shaft 面。\n      bbox_filtered = remove_faces_inside_processed_boxes(category_name, bbox_filtered)\n      center_filtered = remove_faces_inside_processed_boxes(category_name, center_filtered)\n      center_size_filtered = remove_faces_inside_processed_boxes(category_name, center_size_filtered)\n\n      msg(\n         category_name +\n         " corrected filter: before=" + str(len(candidate_faces)) +\n         ", bbox=" + str(len(bbox_filtered)) +\n         ", center=" + str(len(center_filtered)) +\n         ", center_size=" + str(len(center_size_filtered)) +\n         ", margin=" + str(margin)\n      )\n\n      if len(center_size_filtered) > 0:\n         return center_size_filtered\n\n      if len(center_filtered) > 0:\n         return center_filtered\n\n      if len(bbox_filtered) > 0:\n         return bbox_filtered\n\n      msg(category_name + " filter returned zero faces, fallback to original candidate faces")\n      return candidate_faces\n\n   except Exception as e:\n      msg(category_name + " filter failed: " + str(e))\n      return candidate_faces\n\n\ndef boolean_subtract_one_category(fluid_body, category_name, cutter_bodies):\n   """\n   按类别分步扣除：\n   1. Boolean 前记录 fluid 已有内部面签名；\n   2. 扣当前类别；\n   3. Boolean 后重新找 fluid；\n   4. 新增内部面 = 当前类别对应的边界面。\n   """\n   if cutter_bodies is None or len(cutter_bodies) == 0:\n      msg("跳过 " + category_name + "，没有 cutter body")\n      return fluid_body, []\n\n   before_sigs = get_internal_face_signatures(fluid_body)\n\n   targets = make_body_selection_from_list([fluid_body])\n   tools = make_body_selection_from_list(cutter_bodies)\n\n   msg("=" * 70)\n   msg("开始分步 Boolean: fluid - " + category_name)\n   msg("=" * 70)\n\n   boolean_success = False\n\n   try:\n      options = MakeSolidsOptions()\n\n      try:\n         options.KeepCutter = True\n      except:\n         pass\n\n      try:\n         options.MergeWhenDone = True\n      except:\n         pass\n\n      try:\n         Combine.Intersect(targets, tools, options, None)\n      except:\n         Combine.Intersect(targets, tools, options)\n\n      boolean_success = True\n      msg("Boolean 成功: " + category_name)\n\n   except Exception as e1:\n      msg("Combine.Intersect 失败: " + category_name)\n      msg("原因: " + str(e1))\n\n   if not boolean_success:\n      try:\n         options = MakeSolidsOptions()\n\n         try:\n            options.KeepCutter = True\n         except:\n            pass\n\n         try:\n            Combine.Split(targets, tools, options, None)\n         except:\n            Combine.Split(targets, tools, options)\n\n         boolean_success = True\n         msg("Boolean 备用成功: " + category_name)\n\n      except Exception as e2:\n         msg("Combine.Split 失败: " + category_name)\n         msg("原因: " + str(e2))\n\n   if not boolean_success:\n      raise Exception("Boolean 扣除失败: " + category_name)\n\n   fluid_body = find_current_fluid_body()\n\n   new_faces = []\n\n   for f in get_body_faces(fluid_body):\n      try:\n         if is_external_domain_face(f):\n            continue\n\n         sig = face_signature(f)\n\n         if sig not in before_sigs:\n            new_faces.append(f)\n\n      except:\n         pass\n\n   new_faces = filter_new_internal_faces_by_cutter_box(category_name, cutter_bodies, new_faces)\n\n   msg(category_name + " 新增内壁面数量 = " + str(len(new_faces)))\n\n   register_processed_cutter_box(category_name, cutter_bodies, len(new_faces))\n\n   return fluid_body, new_faces\n\n\ndef classify_external_faces(fluid_body):\n   inlet_faces = []\n   outlet_faces = []\n   wall_faces = []\n\n   flow_axis = SELECTED_FLOW_AXIS\n   if flow_axis is None:\n      flow_axis = "x"\n\n   flow_min, flow_max = get_domain_minmax_by_axis(flow_axis)\n\n   for f in get_body_faces(fluid_body):\n      try:\n         if face_on_axis(f, flow_axis, flow_min):\n            inlet_faces.append(f)\n\n         elif face_on_axis(f, flow_axis, flow_max):\n            outlet_faces.append(f)\n\n         else:\n            # 非流向轴的四个外边界全部归为 wall。\n            for axis in ["x", "y", "z"]:\n               if axis == flow_axis:\n                  continue\n\n               amin, amax = get_domain_minmax_by_axis(axis)\n\n               if face_on_axis(f, axis, amin):\n                  wall_faces.append(f)\n                  break\n\n               elif face_on_axis(f, axis, amax):\n                  wall_faces.append(f)\n                  break\n\n      except:\n         pass\n\n   return inlet_faces, outlet_faces, wall_faces\n\n\n\n\ndef unit_to_mm(value):\n   """将 SpaceClaim 内部长度单位转换为 mm。"""\n   try:\n      return float(value / MM(1.0))\n   except:\n      pass\n\n   try:\n      return float(value)\n   except:\n      return 0.0\n\n\ndef unit_area_to_mm2(value):\n   """\n   将 SpaceClaim 几何属性中的面积转换为 mm^2。\n   说明：BoundingBox 通过 unit_to_mm 单独换算；Shape.Area / Face.Area 通常返回内部 SI 面积。\n   """\n   try:\n      scale = MM(1.0) * MM(1.0)\n      return float(value / scale)\n   except:\n      pass\n\n   try:\n      return float(value)\n   except:\n      return 0.0\n\n\ndef unit_volume_to_mm3(value):\n   """\n   将 SpaceClaim 几何属性中的体积转换为 mm^3。\n   说明：Shape.Volume 通常返回内部 SI 体积，必须除以 MM(1)^3。\n   """\n   try:\n      scale = MM(1.0) * MM(1.0) * MM(1.0)\n      return float(value / scale)\n   except:\n      pass\n\n   try:\n      return float(value)\n   except:\n      return 0.0\n\n\ndef clean_text_for_table(text):\n   try:\n      s = str(text)\n   except:\n      s = ""\n\n   s = s.replace("\\t", " ")\n   s = s.replace("\\r", " ")\n   s = s.replace("\\n", " ")\n   while "  " in s:\n      s = s.replace("  ", " ")\n   return s.strip()\n\n\ndef format_num(value):\n   try:\n      return "%.6f" % float(value)\n   except:\n      return "0.000000"\n\n\ndef bodies_union_box_tuple(bodies):\n   valid = []\n\n   for b in bodies:\n      try:\n         valid.append(body_box_tuple(b))\n      except:\n         pass\n\n   if len(valid) == 0:\n      return None\n\n   xmin = min([x[0] for x in valid])\n   xmax = max([x[1] for x in valid])\n   ymin = min([x[2] for x in valid])\n   ymax = max([x[3] for x in valid])\n   zmin = min([x[4] for x in valid])\n   zmax = max([x[5] for x in valid])\n\n   return (xmin, xmax, ymin, ymax, zmin, zmax)\n\n\ndef bbox_record_from_tuple(box_tuple):\n   xmin, xmax, ymin, ymax, zmin, zmax = box_tuple\n\n   xmin_mm = unit_to_mm(xmin)\n   xmax_mm = unit_to_mm(xmax)\n   ymin_mm = unit_to_mm(ymin)\n   ymax_mm = unit_to_mm(ymax)\n   zmin_mm = unit_to_mm(zmin)\n   zmax_mm = unit_to_mm(zmax)\n\n   lx_mm = abs(xmax_mm - xmin_mm)\n   ly_mm = abs(ymax_mm - ymin_mm)\n   lz_mm = abs(zmax_mm - zmin_mm)\n\n   cx_mm = 0.5 * (xmin_mm + xmax_mm)\n   cy_mm = 0.5 * (ymin_mm + ymax_mm)\n   cz_mm = 0.5 * (zmin_mm + zmax_mm)\n\n   min_dim_mm = min(lx_mm, ly_mm, lz_mm)\n   max_dim_mm = max(lx_mm, ly_mm, lz_mm)\n\n   return {\n      "xmin_mm": xmin_mm,\n      "xmax_mm": xmax_mm,\n      "ymin_mm": ymin_mm,\n      "ymax_mm": ymax_mm,\n      "zmin_mm": zmin_mm,\n      "zmax_mm": zmax_mm,\n      "length_x_mm": lx_mm,\n      "width_y_mm": ly_mm,\n      "height_z_mm": lz_mm,\n      "center_x_mm": cx_mm,\n      "center_y_mm": cy_mm,\n      "center_z_mm": cz_mm,\n      "min_dimension_mm": min_dim_mm,\n      "max_dimension_mm": max_dim_mm,\n      "bbox_volume_mm3": lx_mm * ly_mm * lz_mm,\n   }\n\n\ndef write_part_bounding_boxes(categories, solid_bodies, output_path):\n   """\n   输出每个部件的轴对齐长方体外壳尺寸。\n\n   注意：这里只测量，不创建任何长方体外壳；\n   长方体边始终与 SpaceClaim 全局 X/Y/Z 轴平行。\n   """\n   try:\n      folder = os.path.dirname(output_path)\n      if folder and not os.path.exists(folder):\n         os.makedirs(folder)\n\n      lines = []\n      lines.append("# Axis-aligned bounding boxes measured in SpaceClaim before Boolean subtraction")\n      lines.append("# Unit: mm")\n      lines.append("# This file only records measurements; no bounding boxes are created in SpaceClaim.")\n      lines.append("# length_x_mm = xmax_mm - xmin_mm; width_y_mm = ymax_mm - ymin_mm; height_z_mm = zmax_mm - zmin_mm")\n      lines.append("")\n\n      header = [\n         "record_type",\n         "part_label",\n         "body_count",\n         "body_index",\n         "xmin_mm",\n         "xmax_mm",\n         "ymin_mm",\n         "ymax_mm",\n         "zmin_mm",\n         "zmax_mm",\n         "length_x_mm",\n         "width_y_mm",\n         "height_z_mm",\n         "center_x_mm",\n         "center_y_mm",\n         "center_z_mm",\n         "min_dimension_mm",\n         "max_dimension_mm",\n         "bbox_volume_mm3",\n         "body_names",\n      ]\n      lines.append("\\t".join(header))\n\n      # 1) 按 v76 分类后的部件名输出 union bounding box。\n      try:\n         order = ordered_category_names(categories)\n      except:\n         order = sorted(categories.keys())\n\n      for cat in order:\n         try:\n            bodies = categories.get(cat, [])\n         except:\n            bodies = []\n\n         if bodies is None or len(bodies) == 0:\n            continue\n\n         box = bodies_union_box_tuple(bodies)\n         if box is None:\n            continue\n\n         rec = bbox_record_from_tuple(box)\n         body_names = []\n         for b in bodies:\n            body_names.append(clean_text_for_table(get_recorded_body_name(b)))\n\n         row = [\n            "PART_SUMMARY",\n            clean_text_for_table(cat),\n            str(len(bodies)),\n            "-",\n            format_num(rec["xmin_mm"]),\n            format_num(rec["xmax_mm"]),\n            format_num(rec["ymin_mm"]),\n            format_num(rec["ymax_mm"]),\n            format_num(rec["zmin_mm"]),\n            format_num(rec["zmax_mm"]),\n            format_num(rec["length_x_mm"]),\n            format_num(rec["width_y_mm"]),\n            format_num(rec["height_z_mm"]),\n            format_num(rec["center_x_mm"]),\n            format_num(rec["center_y_mm"]),\n            format_num(rec["center_z_mm"]),\n            format_num(rec["min_dimension_mm"]),\n            format_num(rec["max_dimension_mm"]),\n            format_num(rec["bbox_volume_mm3"]),\n            " ; ".join(body_names),\n         ]\n         lines.append("\\t".join(row))\n\n         # 2) 如果同一个部件 label 下有多个 body，再逐个 body 输出明细。\n         if len(bodies) > 1:\n            for i, b in enumerate(bodies):\n               try:\n                  body_box = body_box_tuple(b)\n                  body_rec = bbox_record_from_tuple(body_box)\n                  body_name = clean_text_for_table(get_recorded_body_name(b))\n                  row = [\n                     "BODY_DETAIL",\n                     clean_text_for_table(cat),\n                     str(len(bodies)),\n                     str(i),\n                     format_num(body_rec["xmin_mm"]),\n                     format_num(body_rec["xmax_mm"]),\n                     format_num(body_rec["ymin_mm"]),\n                     format_num(body_rec["ymax_mm"]),\n                     format_num(body_rec["zmin_mm"]),\n                     format_num(body_rec["zmax_mm"]),\n                     format_num(body_rec["length_x_mm"]),\n                     format_num(body_rec["width_y_mm"]),\n                     format_num(body_rec["height_z_mm"]),\n                     format_num(body_rec["center_x_mm"]),\n                     format_num(body_rec["center_y_mm"]),\n                     format_num(body_rec["center_z_mm"]),\n                     format_num(body_rec["min_dimension_mm"]),\n                     format_num(body_rec["max_dimension_mm"]),\n                     format_num(body_rec["bbox_volume_mm3"]),\n                     body_name,\n                  ]\n                  lines.append("\\t".join(row))\n               except Exception as e_body:\n                  msg("警告: body bbox 明细输出失败: " + str(e_body))\n\n      # 3) 额外输出全模型实体的 union bbox，方便检查计算域尺度。\n      try:\n         all_box = bodies_union_box_tuple(solid_bodies)\n         if all_box is not None:\n            rec = bbox_record_from_tuple(all_box)\n            row = [\n               "ALL_SOLID_BODIES",\n               "all_solid_bodies",\n               str(len(solid_bodies)),\n               "-",\n               format_num(rec["xmin_mm"]),\n               format_num(rec["xmax_mm"]),\n               format_num(rec["ymin_mm"]),\n               format_num(rec["ymax_mm"]),\n               format_num(rec["zmin_mm"]),\n               format_num(rec["zmax_mm"]),\n               format_num(rec["length_x_mm"]),\n               format_num(rec["width_y_mm"]),\n               format_num(rec["height_z_mm"]),\n               format_num(rec["center_x_mm"]),\n               format_num(rec["center_y_mm"]),\n               format_num(rec["center_z_mm"]),\n               format_num(rec["min_dimension_mm"]),\n               format_num(rec["max_dimension_mm"]),\n               format_num(rec["bbox_volume_mm3"]),\n               "-",\n            ]\n            lines.append("\\t".join(row))\n      except Exception as e_all:\n         msg("警告: 全模型 bbox 输出失败: " + str(e_all))\n\n      f = open(output_path, "w")\n      for line in lines:\n         f.write(line + "\\n")\n      f.close()\n\n      msg("已写出部件轴对齐外壳尺寸文件: " + output_path)\n      return True\n\n   except Exception as e:\n      msg("警告: 写出部件外壳尺寸文件失败: " + str(e))\n      return False\n\n\n\n# ============================================================\n# 5.1 增强几何指标提取：拓扑、面积、体积、小特征、等效厚度、包围盒间距\n# ============================================================\n\ndef unique_entity_append(items, obj):\n   if obj is None:\n      return\n   for old in items:\n      try:\n         if old == obj:\n            return\n      except:\n         pass\n   items.append(obj)\n\n\ndef safe_entity_list(obj, attr_names):\n   out = []\n   for attr in attr_names:\n      try:\n         seq = getattr(obj, attr)\n         for item in seq:\n            unique_entity_append(out, item)\n      except:\n         pass\n   return out\n\n\ndef get_body_edges(body):\n   edges = []\n\n   for obj in [body, get_master(body)]:\n      if obj is None:\n         continue\n      for e in safe_entity_list(obj, ["Edges"]):\n         unique_entity_append(edges, e)\n      try:\n         shape = obj.Shape\n         for e in safe_entity_list(shape, ["Edges"]):\n            unique_entity_append(edges, e)\n      except:\n         pass\n\n   # 兜底：从 faces 收集 edges\n   try:\n      faces = get_body_faces(body)\n      for f in faces:\n         for e in safe_entity_list(f, ["Edges"]):\n            unique_entity_append(edges, e)\n         try:\n            fs = f.Shape\n            for e in safe_entity_list(fs, ["Edges"]):\n               unique_entity_append(edges, e)\n         except:\n            pass\n   except:\n      pass\n\n   return edges\n\n\ndef get_body_vertices(body):\n   vertices = []\n\n   for obj in [body, get_master(body)]:\n      if obj is None:\n         continue\n      for v in safe_entity_list(obj, ["Vertices"]):\n         unique_entity_append(vertices, v)\n      try:\n         shape = obj.Shape\n         for v in safe_entity_list(shape, ["Vertices"]):\n            unique_entity_append(vertices, v)\n      except:\n         pass\n\n   edges = get_body_edges(body)\n   for e in edges:\n      for v in safe_entity_list(e, ["Vertices"]):\n         unique_entity_append(vertices, v)\n      for attr in ["StartVertex", "EndVertex", "Vertex1", "Vertex2"]:\n         try:\n            unique_entity_append(vertices, getattr(e, attr))\n         except:\n            pass\n\n   return vertices\n\n\ndef numeric_value(x):\n   try:\n      return float(x)\n   except:\n      pass\n\n   try:\n      return float(str(x))\n   except:\n      return None\n\n\ndef try_get_numeric_attr(obj, attr_names):\n   if obj is None:\n      return None\n\n   for attr in attr_names:\n      try:\n         v = getattr(obj, attr)\n         val = numeric_value(v)\n         if val is not None:\n            return val\n      except:\n         pass\n\n   for attr in attr_names:\n      try:\n         fn = getattr(obj, attr)\n         v = fn()\n         val = numeric_value(v)\n         if val is not None:\n            return val\n      except:\n         pass\n\n   return None\n\n\ndef entity_bbox_tuple(entity):\n   for obj in [entity, get_master(entity)]:\n      if obj is None:\n         continue\n      try:\n         box = obj.Shape.GetBoundingBox(Matrix.Identity)\n         return (\n            box.MinCorner.X,\n            box.MaxCorner.X,\n            box.MinCorner.Y,\n            box.MaxCorner.Y,\n            box.MinCorner.Z,\n            box.MaxCorner.Z\n         )\n      except:\n         pass\n      try:\n         box = obj.GetBoundingBox(Matrix.Identity)\n         return (\n            box.MinCorner.X,\n            box.MaxCorner.X,\n            box.MinCorner.Y,\n            box.MaxCorner.Y,\n            box.MinCorner.Z,\n            box.MaxCorner.Z\n         )\n      except:\n         pass\n   return None\n\n\ndef face_area_value(face):\n   for obj in [face, get_master(face)]:\n      if obj is None:\n         continue\n\n      val = try_get_numeric_attr(obj, ["Area", "SurfaceArea", "GetArea"])\n      if val is not None and val >= 0:\n         return unit_area_to_mm2(abs(val))\n\n      try:\n         shape = obj.Shape\n         val = try_get_numeric_attr(shape, ["Area", "SurfaceArea", "GetArea"])\n         if val is not None and val >= 0:\n            return unit_area_to_mm2(abs(val))\n      except:\n         pass\n\n   # 兜底：用 face 包围盒投影面积估算一个数量级\n   try:\n      box = entity_bbox_tuple(face)\n      if box is not None:\n         lx, ly, lz = get_box_dims(box)\n         vals = sorted([abs(lx), abs(ly), abs(lz)])\n         return unit_area_to_mm2(vals[1] * vals[2])\n   except:\n      pass\n\n   return None\n\n\ndef edge_length_value(edge):\n   for obj in [edge, get_master(edge)]:\n      if obj is None:\n         continue\n\n      val = try_get_numeric_attr(obj, ["Length", "CurveLength", "GetLength", "EvalLength"])\n      if val is not None and val >= 0:\n         return unit_to_mm(abs(val))\n\n      try:\n         shape = obj.Shape\n         val = try_get_numeric_attr(shape, ["Length", "CurveLength", "GetLength", "EvalLength"])\n         if val is not None and val >= 0:\n            return unit_to_mm(abs(val))\n      except:\n         pass\n\n      try:\n         curve = obj.Curve\n         val = try_get_numeric_attr(curve, ["Length", "CurveLength", "GetLength", "EvalLength"])\n         if val is not None and val >= 0:\n            return unit_to_mm(abs(val))\n      except:\n         pass\n\n   # 兜底：用 edge 包围盒对角线估算\n   try:\n      box = entity_bbox_tuple(edge)\n      if box is not None:\n         lx, ly, lz = get_box_dims(box)\n         return unit_to_mm(math.sqrt(lx * lx + ly * ly + lz * lz))\n   except:\n      pass\n\n   return None\n\n\ndef body_volume_value(body):\n   for obj in [body, get_master(body)]:\n      if obj is None:\n         continue\n\n      try:\n         shape = obj.Shape\n         val = try_get_numeric_attr(shape, ["Volume", "GetVolume"])\n         if val is not None and val >= 0:\n            return unit_volume_to_mm3(abs(val))\n      except:\n         pass\n\n      val = try_get_numeric_attr(obj, ["Volume", "GetVolume"])\n      if val is not None and val >= 0:\n         return unit_volume_to_mm3(abs(val))\n\n   return None\n\n\ndef valid_numbers(values):\n   out = []\n   for v in values:\n      try:\n         fv = float(v)\n         if fv >= 0:\n            out.append(fv)\n      except:\n         pass\n   return out\n\n\ndef percentile_value(values, p):\n   vals = valid_numbers(values)\n   if len(vals) == 0:\n      return -1.0\n\n   vals.sort()\n\n   if len(vals) == 1:\n      return vals[0]\n\n   if p <= 0:\n      return vals[0]\n   if p >= 100:\n      return vals[-1]\n\n   pos = (p / 100.0) * (len(vals) - 1)\n   lo = int(math.floor(pos))\n   hi = int(math.ceil(pos))\n\n   if lo == hi:\n      return vals[lo]\n\n   frac = pos - lo\n   return vals[lo] * (1.0 - frac) + vals[hi] * frac\n\n\ndef sum_valid(values):\n   vals = valid_numbers(values)\n   s = 0.0\n   for v in vals:\n      s += v\n   return s\n\n\ndef aabb_gap(box_a, box_b):\n   axmin, axmax, aymin, aymax, azmin, azmax = box_a\n   bxmin, bxmax, bymin, bymax, bzmin, bzmax = box_b\n\n   dx = 0.0\n   if axmax < bxmin:\n      dx = bxmin - axmax\n   elif bxmax < axmin:\n      dx = axmin - bxmax\n\n   dy = 0.0\n   if aymax < bymin:\n      dy = bymin - aymax\n   elif bymax < aymin:\n      dy = aymin - bymax\n\n   dz = 0.0\n   if azmax < bzmin:\n      dz = bzmin - azmax\n   elif bzmax < azmin:\n      dz = azmin - bzmax\n\n   return math.sqrt(dx * dx + dy * dy + dz * dz)\n\n\ndef compute_part_basic_metrics(bodies):\n   face_count = 0\n   edge_count = 0\n   vertex_count = 0\n\n   face_areas = []\n   edge_lengths = []\n   volumes = []\n   body_names = []\n\n   for b in bodies:\n      body_names.append(clean_text_for_table(get_recorded_body_name(b)))\n\n      faces = get_body_faces(b)\n      edges = get_body_edges(b)\n      vertices = get_body_vertices(b)\n\n      face_count += len(faces)\n      edge_count += len(edges)\n      vertex_count += len(vertices)\n\n      for f in faces:\n         a = face_area_value(f)\n         if a is not None:\n            face_areas.append(a)\n\n      for e in edges:\n         l = edge_length_value(e)\n         if l is not None:\n            edge_lengths.append(l)\n\n      v = body_volume_value(b)\n      if v is not None:\n         volumes.append(v)\n\n   return {\n      "face_count": face_count,\n      "edge_count": edge_count,\n      "vertex_count": vertex_count,\n      "face_areas": face_areas,\n      "edge_lengths": edge_lengths,\n      "volumes": volumes,\n      "body_names": body_names,\n   }\n\n\ndef classify_complexity(aspect_ratio, fill_ratio, edge_p10, face_area_p10, thickness_eq, d_box, face_count, edge_count, nearest_gap):\n   score = 0\n\n   try:\n      if aspect_ratio > 5.0:\n         score += 1\n      if aspect_ratio > 12.0:\n         score += 1\n   except:\n      pass\n\n   try:\n      if fill_ratio >= 0 and fill_ratio < 0.25:\n         score += 2\n      elif fill_ratio >= 0 and fill_ratio < 0.45:\n         score += 1\n   except:\n      pass\n\n   try:\n      if d_box > 0 and edge_p10 > 0 and edge_p10 / d_box < 0.10:\n         score += 2\n      elif d_box > 0 and edge_p10 > 0 and edge_p10 / d_box < 0.20:\n         score += 1\n   except:\n      pass\n\n   try:\n      if d_box > 0 and thickness_eq > 0 and thickness_eq / d_box < 0.15:\n         score += 3\n      elif d_box > 0 and thickness_eq > 0 and thickness_eq / d_box < 0.30:\n         score += 1\n   except:\n      pass\n\n   try:\n      if face_count > 50:\n         score += 2\n      elif face_count > 20:\n         score += 1\n   except:\n      pass\n\n   try:\n      if edge_count > 120:\n         score += 2\n      elif edge_count > 50:\n         score += 1\n   except:\n      pass\n\n   try:\n      if d_box > 0 and nearest_gap >= 0 and nearest_gap / d_box < 0.30:\n         score += 2\n   except:\n      pass\n\n   if score < 0:\n      score = 0\n   if score > 10:\n      score = 10\n\n   if score <= 2:\n      cls = "simple"\n   elif score <= 5:\n      cls = "medium"\n   else:\n      cls = "complex"\n\n   return score, cls\n\n\ndef metrics_row_for_part(record_type, part_label, bodies, body_index_text, other_part_boxes):\n   box = bodies_union_box_tuple(bodies)\n   if box is None:\n      return None\n\n   rec = bbox_record_from_tuple(box)\n   basic = compute_part_basic_metrics(bodies)\n\n   face_areas = basic["face_areas"]\n   edge_lengths = basic["edge_lengths"]\n   volumes = basic["volumes"]\n\n   surface_area = sum_valid(face_areas)\n   solid_volume = sum_valid(volumes)\n\n   if len(volumes) == 0:\n      solid_volume_out = -1.0\n   else:\n      solid_volume_out = solid_volume\n\n   bbox_volume = rec["bbox_volume_mm3"]\n   fill_ratio = -1.0\n   if solid_volume_out >= 0 and bbox_volume > 0:\n      fill_ratio = solid_volume_out / bbox_volume\n\n   thickness_eq = -1.0\n   if solid_volume_out >= 0 and surface_area > 0:\n      thickness_eq = 2.0 * solid_volume_out / surface_area\n\n   edge_min = percentile_value(edge_lengths, 0)\n   edge_p05 = percentile_value(edge_lengths, 5)\n   edge_p10 = percentile_value(edge_lengths, 10)\n   edge_median = percentile_value(edge_lengths, 50)\n   edge_p90 = percentile_value(edge_lengths, 90)\n   edge_max = percentile_value(edge_lengths, 100)\n\n   face_min = percentile_value(face_areas, 0)\n   face_p05 = percentile_value(face_areas, 5)\n   face_p10 = percentile_value(face_areas, 10)\n   face_median = percentile_value(face_areas, 50)\n   face_p90 = percentile_value(face_areas, 90)\n   face_max = percentile_value(face_areas, 100)\n\n   d_box = rec["min_dimension_mm"]\n   aspect_ratio = -1.0\n   if d_box > 0:\n      aspect_ratio = rec["max_dimension_mm"] / d_box\n\n   feature_size_p10 = -1.0\n   candidates = []\n   if edge_p10 > 0:\n      candidates.append(edge_p10)\n   if face_p10 > 0:\n      candidates.append(math.sqrt(face_p10))\n   if thickness_eq > 0:\n      candidates.append(thickness_eq)\n   if len(candidates) > 0:\n      feature_size_p10 = min(candidates)\n\n   small_edge_count = 0\n   small_face_count = 0\n   small_len_limit = -1.0\n   small_area_limit = -1.0\n   if d_box > 0:\n      small_len_limit = 0.10 * d_box\n      small_area_limit = small_len_limit * small_len_limit\n      for l in edge_lengths:\n         try:\n            if l <= small_len_limit:\n               small_edge_count += 1\n         except:\n            pass\n      for a in face_areas:\n         try:\n            if a <= small_area_limit:\n               small_face_count += 1\n         except:\n            pass\n\n   nearest_gap = -1.0\n   nearest_label = "-"\n   for other_label, other_box in other_part_boxes:\n      if other_label == part_label:\n         continue\n      try:\n         g = unit_to_mm(aabb_gap(box, other_box))\n         if nearest_gap < 0 or g < nearest_gap:\n            nearest_gap = g\n            nearest_label = other_label\n      except:\n         pass\n\n   score, cls = classify_complexity(\n      aspect_ratio,\n      fill_ratio,\n      edge_p10,\n      face_p10,\n      thickness_eq,\n      d_box,\n      basic["face_count"],\n      basic["edge_count"],\n      nearest_gap,\n   )\n\n   thin_feature_flag = "no"\n   try:\n      if d_box > 0 and thickness_eq > 0 and thickness_eq / d_box < 0.20:\n         thin_feature_flag = "yes"\n      if fill_ratio >= 0 and fill_ratio < 0.25:\n         thin_feature_flag = "yes"\n   except:\n      pass\n\n   row = [\n      record_type,\n      clean_text_for_table(part_label),\n      str(len(bodies)),\n      str(body_index_text),\n      format_num(rec["xmin_mm"]),\n      format_num(rec["xmax_mm"]),\n      format_num(rec["ymin_mm"]),\n      format_num(rec["ymax_mm"]),\n      format_num(rec["zmin_mm"]),\n      format_num(rec["zmax_mm"]),\n      format_num(rec["length_x_mm"]),\n      format_num(rec["width_y_mm"]),\n      format_num(rec["height_z_mm"]),\n      format_num(rec["center_x_mm"]),\n      format_num(rec["center_y_mm"]),\n      format_num(rec["center_z_mm"]),\n      format_num(rec["min_dimension_mm"]),\n      format_num(rec["max_dimension_mm"]),\n      format_num(bbox_volume),\n      format_num(aspect_ratio),\n      str(basic["face_count"]),\n      str(basic["edge_count"]),\n      str(basic["vertex_count"]),\n      format_num(surface_area),\n      format_num(solid_volume_out),\n      format_num(fill_ratio),\n      format_num(edge_min),\n      format_num(edge_p05),\n      format_num(edge_p10),\n      format_num(edge_median),\n      format_num(edge_p90),\n      format_num(edge_max),\n      str(small_edge_count),\n      format_num(face_min),\n      format_num(face_p05),\n      format_num(face_p10),\n      format_num(face_median),\n      format_num(face_p90),\n      format_num(face_max),\n      str(small_face_count),\n      format_num(thickness_eq),\n      format_num(feature_size_p10),\n      format_num(nearest_gap),\n      clean_text_for_table(nearest_label),\n      str(score),\n      cls,\n      thin_feature_flag,\n      " ; ".join(basic["body_names"]),\n   ]\n\n   return row\n\n\ndef write_part_geometry_metrics(categories, solid_bodies, output_path):\n   """\n   输出增强几何指标，用于后续 Fluent Meshing 自动局部面网格尺寸控制。\n\n   注意：\n   1) thickness_equiv_2v_over_area_mm = 2 * solid_volume / surface_area，\n      是薄片等效厚度估计，不是严格最小厚度。\n   2) bbox_gap_to_nearest_part_mm 是部件轴对齐包围盒之间的距离，\n      不是严格曲面-曲面最小距离。\n   3) feature_size_p10_est_mm 是由 edge_length_p10、sqrt(face_area_p10)、\n      thickness_equiv_2v_over_area_mm 取较小值形成的小特征估计。\n   """\n   try:\n      folder = os.path.dirname(output_path)\n      if folder and not os.path.exists(folder):\n         os.makedirs(folder)\n\n      lines = []\n      lines.append("# Enhanced geometry metrics measured in SpaceClaim before Boolean subtraction")\n      lines.append("# Unit: mm, mm2, mm3")\n      lines.append("# Length properties from SpaceClaim are converted by unit_to_mm; area by unit_area_to_mm2; volume by unit_volume_to_mm3.")\n      lines.append("# This file only records measurements; no geometry is created.")\n      lines.append("# thickness_equiv_2v_over_area_mm = 2 * solid_volume_mm3 / surface_area_mm2; it is an equivalent-thickness estimate, not exact minimum thickness.")\n      lines.append("# bbox_gap_to_nearest_part_mm is AABB-to-AABB gap, not exact surface-to-surface gap.")\n      lines.append("# feature_size_p10_est_mm = min(edge_length_p10_mm, sqrt(face_area_p10_mm2), thickness_equiv_2v_over_area_mm) when available.")\n      lines.append("")\n\n      header = [\n         "record_type",\n         "part_label",\n         "body_count",\n         "body_index",\n         "xmin_mm",\n         "xmax_mm",\n         "ymin_mm",\n         "ymax_mm",\n         "zmin_mm",\n         "zmax_mm",\n         "length_x_mm",\n         "width_y_mm",\n         "height_z_mm",\n         "center_x_mm",\n         "center_y_mm",\n         "center_z_mm",\n         "min_dimension_mm",\n         "max_dimension_mm",\n         "bbox_volume_mm3",\n         "aspect_ratio_max_over_min",\n         "face_count",\n         "edge_count",\n         "vertex_count",\n         "surface_area_mm2",\n         "solid_volume_mm3",\n         "fill_ratio_solid_over_bbox",\n         "edge_length_min_mm",\n         "edge_length_p05_mm",\n         "edge_length_p10_mm",\n         "edge_length_median_mm",\n         "edge_length_p90_mm",\n         "edge_length_max_mm",\n         "small_edge_count_len_lt_0p1D",\n         "face_area_min_mm2",\n         "face_area_p05_mm2",\n         "face_area_p10_mm2",\n         "face_area_median_mm2",\n         "face_area_p90_mm2",\n         "face_area_max_mm2",\n         "small_face_count_area_lt_0p01D2",\n         "thickness_equiv_2v_over_area_mm",\n         "feature_size_p10_est_mm",\n         "bbox_gap_to_nearest_part_mm",\n         "nearest_part_label",\n         "complexity_score_0_10",\n         "complexity_class",\n         "thin_feature_flag",\n         "body_names",\n      ]\n      lines.append("\\t".join(header))\n\n      try:\n         order = ordered_category_names(categories)\n      except:\n         order = sorted(categories.keys())\n\n      part_boxes = []\n      for cat in order:\n         try:\n            bodies = categories.get(cat, [])\n            if bodies is None or len(bodies) == 0:\n               continue\n            box = bodies_union_box_tuple(bodies)\n            if box is not None:\n               part_boxes.append((cat, box))\n         except:\n            pass\n\n      for cat in order:\n         try:\n            bodies = categories.get(cat, [])\n         except:\n            bodies = []\n\n         if bodies is None or len(bodies) == 0:\n            continue\n\n         row = metrics_row_for_part("PART_SUMMARY", cat, bodies, "-", part_boxes)\n         if row is not None:\n            lines.append("\\t".join(row))\n\n         if len(bodies) > 1:\n            for i, b in enumerate(bodies):\n               row = metrics_row_for_part("BODY_DETAIL", cat, [b], i, part_boxes)\n               if row is not None:\n                  lines.append("\\t".join(row))\n\n      # 全部实体的整体统计\n      try:\n         all_part_boxes = [("all_solid_bodies", bodies_union_box_tuple(solid_bodies))]\n         row = metrics_row_for_part("ALL_SOLID_BODIES", "all_solid_bodies", solid_bodies, "-", all_part_boxes)\n         if row is not None:\n            lines.append("\\t".join(row))\n      except Exception as e_all:\n         msg("警告: 全模型增强几何指标输出失败: " + str(e_all))\n\n      f = open(output_path, "w")\n      for line in lines:\n         f.write(line + "\\n")\n      f.close()\n\n      msg("已写出增强几何指标文件: " + output_path)\n      return True\n\n   except Exception as e:\n      msg("警告: 写出增强几何指标文件失败: " + str(e))\n      return False\n\n\ndef write_surface_labels(labels):\n   """\n   将本次实际创建的 AUV 表面命名写出给 Fluent 脚本。\n   Fluent 后续用该文件通用识别边界层和阻力报告面。\n   """\n   try:\n      folder = os.path.dirname(SURFACE_LABELS_PATH)\n      if folder and not os.path.exists(folder):\n         os.makedirs(folder)\n\n      f = open(SURFACE_LABELS_PATH, "w")\n      for label in labels:\n         f.write(str(label) + "\\n")\n      f.close()\n\n      msg("已写出 AUV 表面标签文件: " + SURFACE_LABELS_PATH)\n   except Exception as e:\n      msg("警告: 写出 AUV 表面标签文件失败: " + str(e))\n\n\n# ============================================================\n# 6. Named Selection\n# ============================================================\n\ndef delete_named_selection_if_exists(name):\n   try:\n      NamedSelection.Delete(name)\n   except:\n      pass\n\n\ndef rename_last_group(target_name):\n   try:\n      groups = WindowHelper.GetGroups()\n      last_group = groups[-1]\n\n      try:\n         old_name = last_group.GetName()\n      except:\n         old_name = last_group.Name\n\n      NamedSelection.Rename(old_name, target_name)\n      return True\n   except:\n      pass\n\n   for old_name in [\n      "Group1", "Group2", "Group3", "Group4", "Group5",\n      "Group6", "Group7", "Group8", "Group9", "Group10",\n      "Group11", "Group12", "Group13", "Group14", "Group15",\n      "Group16", "Group17", "Group18", "Group19", "Group20",\n      "Group21", "Group22", "Group23", "Group24", "Group25",\n      "Group26", "Group27", "Group28", "Group29", "Group30",\n      "Group31", "Group32", "Group33", "Group34", "Group35"\n   ]:\n      try:\n         NamedSelection.Rename(old_name, target_name)\n         return True\n      except:\n         pass\n\n   return False\n\n\ndef create_face_named_selection(name, faces):\n   if faces is None or len(faces) == 0:\n      msg("未创建面命名 " + name + "，因为没有识别到面")\n      return False\n\n   delete_named_selection_if_exists(name)\n\n   try:\n      primary_selection = FaceSelection.Create(faces)\n   except:\n      primary_selection = Selection.Create(faces)\n\n   secondary_selection = Selection.Empty()\n\n   NamedSelection.Create(primary_selection, secondary_selection)\n\n   ok = rename_last_group(name)\n\n   if ok:\n      msg("已创建面命名: " + name + " | 面数量 = " + str(len(faces)))\n   else:\n      msg("Named Selection 创建了，但重命名失败: " + name)\n\n   return ok\n\n\ndef create_body_named_selection(name, body):\n   if body is None:\n      msg("未创建体命名 " + name + "，body 为空")\n      return False\n\n   delete_named_selection_if_exists(name)\n\n   try:\n      primary_selection = BodySelection.Create(body)\n   except:\n      primary_selection = Selection.Create(body)\n\n   secondary_selection = Selection.Empty()\n\n   NamedSelection.Create(primary_selection, secondary_selection)\n\n   ok = rename_last_group(name)\n\n   if ok:\n      msg("已创建体命名: " + name)\n   else:\n      msg("Body Named Selection 创建了，但重命名失败: " + name)\n\n   return ok\n\n\n# ============================================================\n# 7. 主程序\n# ============================================================\n\ntry:\n   msg("=" * 70)\n   msg("SpaceClaim v76 原始部件名称面命名 + 计算域构建脚本启动")\n   msg("=" * 70)\n\n   clear_current_document()\n\n   open_geometry(INPUT_PATH)\n\n   print_body_debug_info("导入后 Body 状态")\n\n   all_bodies = get_all_bodies()\n\n   if len(all_bodies) == 0:\n      raise Exception("导入后没有识别到任何 body")\n\n   solid_bodies = []\n\n   for b in all_bodies:\n      if is_solid_body(b):\n         solid_bodies.append(b)\n\n   if len(solid_bodies) == 0:\n      msg("没有检测到封闭实体，暂时使用所有 body")\n      solid_bodies = all_bodies\n\n   categories = classify_parts(solid_bodies)\n\n   write_part_bounding_boxes(categories, solid_bodies, PART_BBOX_PATH)\n   write_part_geometry_metrics(categories, solid_bodies, PART_GEOMETRY_METRICS_PATH)\n\n   if USE_AUTO_DOMAIN:\n      dxmin, dxmax, dymin, dymax, dzmin, dzmax, flow_length, flow_axis = compute_domain_from_auv(solid_bodies)\n\n      DOMAIN_X_MIN_U = dxmin\n      DOMAIN_X_MAX_U = dxmax\n      DOMAIN_Y_MIN_U = dymin\n      DOMAIN_Y_MAX_U = dymax\n      DOMAIN_Z_MIN_U = dzmin\n      DOMAIN_Z_MAX_U = dzmax\n      AUV_LENGTH_U = flow_length\n      SELECTED_FLOW_AXIS = flow_axis\n\n   else:\n      DOMAIN_X_MIN_U = u(MANUAL_X_MIN)\n      DOMAIN_X_MAX_U = u(MANUAL_X_MAX)\n      DOMAIN_Y_MIN_U = u(MANUAL_Y_MIN)\n      DOMAIN_Y_MAX_U = u(MANUAL_Y_MAX)\n      DOMAIN_Z_MIN_U = u(MANUAL_Z_MIN)\n      DOMAIN_Z_MAX_U = u(MANUAL_Z_MAX)\n      AUV_LENGTH_U = DOMAIN_X_MAX_U - DOMAIN_X_MIN_U\n      SELECTED_FLOW_AXIS = "x"\n\n      msg("使用手动固定计算域")\n\n   # ---------- 创建 fluid ----------\n   fluid_body = create_fluid_domain()\n\n   # ---------- 通用分步 Boolean + 立即创建命名 ----------\n   internal_counts = {}\n   created_surface_labels = []\n\n   category_order = ordered_category_names(categories)\n\n   msg("-" * 70)\n   msg("Boolean 扣除顺序: " + ", ".join(category_order))\n   msg("-" * 70)\n\n   for cat in category_order:\n      if cat not in categories or len(categories[cat]) == 0:\n         msg("跳过 " + cat + "，没有对应 body")\n         continue\n\n      fluid_body, cat_faces = boolean_subtract_one_category(\n         fluid_body,\n         cat,\n         categories[cat]\n      )\n\n      internal_counts[cat] = len(cat_faces)\n\n      if len(cat_faces) > 0:\n         create_face_named_selection(cat, cat_faces)\n         created_surface_labels.append(cat)\n      else:\n         msg("未创建 " + cat + "，因为没有识别到新增内壁面")\n\n   # 将实际创建的 AUV 表面标签传给 Fluent\n   write_surface_labels(created_surface_labels)\n\n   # ---------- 清理，只保留 fluid ----------\n   fluid_body = cleanup_keep_only_fluid(fluid_body)\n\n   # ---------- 外边界 ----------\n   inlet_faces, outlet_faces, wall_faces = classify_external_faces(fluid_body)\n\n   msg("-" * 70)\n   msg("最终边界面识别结果:")\n   msg("inlet faces = " + str(len(inlet_faces)))\n   msg("outlet faces = " + str(len(outlet_faces)))\n   msg("wall faces = " + str(len(wall_faces)))\n   for cat in sorted(internal_counts.keys()):\n      msg(cat + " faces = " + str(internal_counts[cat]))\n   msg("-" * 70)\n\n   create_face_named_selection("inlet", inlet_faces)\n   create_face_named_selection("outlet", outlet_faces)\n   create_face_named_selection("wall", wall_faces)\n\n   create_body_named_selection("fluid", fluid_body)\n\n   save_scdoc(SAVE_PATH)\n\n   msg("=" * 70)\n   msg("完成: 按模型原始部件名称创建内壁面命名，并完成计算域生成")\n   msg("=" * 70)\n\nexcept Exception as e:\n   msg("=" * 70)\n   msg("脚本执行失败")\n   msg(str(e))\n   msg("=" * 70)\n   raise\n'


# ============================================================
# 4. 外层通用工具函数
# ============================================================

def log(text: str = "") -> None:
    print(text, flush=True)


def replace_python_assignment(script_text: str, var_name: str, value) -> str:
    """
    替换 SpaceClaim 脚本中的单行或多行变量赋值。
    """
    lines = script_text.splitlines(True)
    start_index = None
    indent_text = ""

    pattern = re.compile(r"^(\s*)" + re.escape(var_name) + r"\s*=")

    for index, line in enumerate(lines):
        match = pattern.match(line)
        if match:
            start_index = index
            indent_text = match.group(1)
            break

    if start_index is None:
        return script_text

    line = lines[start_index]
    rhs = line.split("=", 1)[1]

    bracket_pairs = {"{": "}", "[": "]", "(": ")"}
    open_chars = set(bracket_pairs.keys())
    close_chars = set(bracket_pairs.values())

    def bracket_delta(text_line: str) -> int:
        delta = 0
        for ch in text_line:
            if ch in open_chars:
                delta += 1
            elif ch in close_chars:
                delta -= 1
        return delta

    balance = bracket_delta(rhs)
    end_index = start_index + 1

    if balance > 0:
        while end_index < len(lines):
            balance += bracket_delta(lines[end_index])
            end_index += 1
            if balance <= 0:
                break

    lines[start_index:end_index] = [f"{indent_text}{var_name} = {repr(value)}\n"]
    return "".join(lines)


def require_file(path: str, label: str) -> Path:
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(f"{label} 不存在：{p}")
    if not p.is_file():
        raise FileNotFoundError(f"{label} 不是文件：{p}")
    return p


def file_mtime(path: Path) -> float:
    try:
        return path.stat().st_mtime
    except FileNotFoundError:
        return 0.0


def file_size(path: Path) -> int:
    try:
        return path.stat().st_size
    except FileNotFoundError:
        return 0


def is_output_fresh(output_path: Path, old_mtime: float, start_time: float) -> bool:
    if not output_path.exists():
        return False
    if file_size(output_path) <= 0:
        return False
    mt = file_mtime(output_path)
    return mt > old_mtime and mt >= start_time - 2.0


def terminate_process(proc: subprocess.Popen, name: str) -> None:
    if proc.poll() is not None:
        return

    log(f"准备关闭 {name} ...")

    try:
        proc.terminate()
        proc.wait(timeout=20)
        log(f"{name} 已正常关闭。")
    except Exception:
        log(f"{name} 未正常关闭，强制结束。")
        try:
            proc.kill()
        except Exception:
            pass


# ============================================================
# 5. SpaceClaim 阶段：从 x_t 到 .scdoc + 几何指标文件
# ============================================================

def write_runtime_spaceclaim_script() -> Path:
    temp_dir = Path(TEMP_SCRIPT_DIR)
    temp_dir.mkdir(parents=True, exist_ok=True)

    sc_text = _EMBEDDED_SC_SCRIPT

    replacements = [
        ("INPUT_PATH", INPUT_XT),
        ("SAVE_PATH", OUTPUT_SCDOC),
        ("SURFACE_LABELS_PATH", SURFACE_LABELS_PATH),
        ("PART_BBOX_PATH", PART_BBOX_PATH),
        ("PART_GEOMETRY_METRICS_PATH", PART_GEOMETRY_METRICS_PATH),
        ("USE_AUTO_DOMAIN", USE_AUTO_DOMAIN),
        ("AUTO_FLOW_AXIS_BY_LONGEST", AUTO_FLOW_AXIS_BY_LONGEST),
        ("FORCED_FLOW_AXIS", FORCED_FLOW_AXIS),
        ("UPSTREAM_LENGTH_RATIO", UPSTREAM_LENGTH_RATIO),
        ("DOWNSTREAM_LENGTH_RATIO", DOWNSTREAM_LENGTH_RATIO),
        ("CROSS_HALF_WIDTH_RATIO", CROSS_HALF_WIDTH_RATIO),
        ("MANUAL_X_MIN", MANUAL_X_MIN),
        ("MANUAL_X_MAX", MANUAL_X_MAX),
        ("MANUAL_Y_MIN", MANUAL_Y_MIN),
        ("MANUAL_Y_MAX", MANUAL_Y_MAX),
        ("MANUAL_Z_MIN", MANUAL_Z_MIN),
        ("MANUAL_Z_MAX", MANUAL_Z_MAX),
        ("PART_BOX_MARGIN_RATIO", PART_BOX_MARGIN_RATIO),
    ]

    for name, value in replacements:
        sc_text = replace_python_assignment(sc_text, name, value)

    runtime_py = temp_dir / f"runtime_spaceclaim_to_metrics_{datetime.now().strftime('%Y%m%d_%H%M%S')}.py"
    runtime_py.write_text(sc_text, encoding="utf-8")

    log(f"已写出 SpaceClaim 运行脚本：{runtime_py}")
    return runtime_py


def wait_for_sc_outputs(proc: subprocess.Popen,
                        scdoc: Path,
                        metrics: Path,
                        old_scdoc_mtime: float,
                        old_metrics_mtime: float,
                        start_time: float) -> bool:
    deadline = time.time() + SPACECLAIM_TIMEOUT_SEC
    last_report = 0.0

    while time.time() < deadline:
        scdoc_ok = is_output_fresh(scdoc, old_scdoc_mtime, start_time)
        metrics_ok = is_output_fresh(metrics, old_metrics_mtime, start_time)

        if scdoc_ok and metrics_ok:
            log(f"检测到新的 .scdoc：{scdoc}")
            log(f"检测到新的增强几何指标文件：{metrics}")
            time.sleep(AFTER_SAVE_GRACE_SEC)
            return True

        if proc.poll() is not None:
            if scdoc_ok and metrics_ok:
                log("SpaceClaim 已退出，且输出文件已更新。")
                return True

            log("SpaceClaim 已退出，但没有同时检测到 .scdoc 和增强几何指标文件。")
            log(f"scdoc_ok={scdoc_ok}, metrics_ok={metrics_ok}")
            return False

        if time.time() - last_report > 15:
            log("等待 SpaceClaim 输出 .scdoc 和增强几何指标文件 ...")
            last_report = time.time()

        time.sleep(2)

    log("等待 SpaceClaim 超时。")
    terminate_process(proc, "SpaceClaim")
    return False


def run_spaceclaim_stage() -> None:
    sc_exe = require_file(SPACECLAIM_EXE, "SpaceClaim.exe")
    input_xt = require_file(INPUT_XT, "输入 x_t 模型")

    Path(WORK_DIR).mkdir(parents=True, exist_ok=True)

    scdoc = Path(OUTPUT_SCDOC)
    metrics = Path(PART_GEOMETRY_METRICS_PATH)

    old_scdoc_mtime = file_mtime(scdoc)
    old_metrics_mtime = file_mtime(metrics)

    log("=" * 80)
    log("阶段 1：SpaceClaim 建模、命名、计算域与几何指标提取")
    log("=" * 80)
    log(f"输入模型：{input_xt}")
    log(f"输出 scdoc：{scdoc}")
    log(f"输出 surface labels：{SURFACE_LABELS_PATH}")
    log(f"输出 bounding boxes：{PART_BBOX_PATH}")
    log(f"输出 geometry metrics：{metrics}")

    runtime_py = write_runtime_spaceclaim_script()

    cmd = [str(sc_exe), f"/RunScript={str(runtime_py)}"]

    log("SpaceClaim 启动命令：")
    log(" ".join([f'"{x}"' if " " in x else x for x in cmd]))

    start_time = time.time()
    proc = subprocess.Popen(cmd, cwd=str(sc_exe.parent))

    ok = wait_for_sc_outputs(
        proc=proc,
        scdoc=scdoc,
        metrics=metrics,
        old_scdoc_mtime=old_scdoc_mtime,
        old_metrics_mtime=old_metrics_mtime,
        start_time=start_time,
    )

    if ok:
        if TERMINATE_SPACECLAIM_AFTER_SAVE:
            terminate_process(proc, "SpaceClaim")
        log("阶段 1 完成：SpaceClaim 输出成功。")
        return

    terminate_process(proc, "SpaceClaim")
    raise RuntimeError(
        "SpaceClaim 没有成功输出 .scdoc 和增强几何指标文件。\n"
        f"运行脚本：{runtime_py}\n"
        f"期望几何指标文件：{metrics}"
    )


# ============================================================
# 6. Target Mesh Size 阶段：读取几何指标并输出 target size
# ============================================================

def safe_float(value, default=-1.0):
    try:
        if value is None:
            return default
        text = str(value).strip()
        if text == "" or text == "-":
            return default
        return float(text)
    except Exception:
        return default


def positive(value):
    v = safe_float(value, -1.0)
    return v if v > 0 else None


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def nice_size(x):
    """按数量级规整，不引入固定 0.5/1/10 mm 阈值。"""
    x = float(x)
    if x <= 0:
        return x
    exponent = math.floor(math.log10(x))
    step = 10.0 ** (exponent - 1)  # 约两位有效数字
    return round(x / step) * step


def round_size(x):
    return nice_size(x)


def format_num(x):
    try:
        return f"{float(x):.6f}"
    except Exception:
        return "-"


def get_box_dimensions(row):
    lx = positive(row.get("length_x_mm"))
    ly = positive(row.get("width_y_mm"))
    lz = positive(row.get("height_z_mm"))
    dims = [v for v in [lx, ly, lz] if v is not None]

    dmin = positive(row.get("min_dimension_mm"))
    dmax = positive(row.get("max_dimension_mm"))
    if dmin is None and dims:
        dmin = min(dims)
    if dmax is None and dims:
        dmax = max(dims)

    if lx is None:
        lx = dmax if dmax is not None else -1.0
    if ly is None:
        ly = dmin if dmin is not None else -1.0
    if lz is None:
        lz = dmin if dmin is not None else -1.0
    if dmin is None:
        dmin = min([v for v in [lx, ly, lz] if v > 0] or [1.0])
    if dmax is None:
        dmax = max([v for v in [lx, ly, lz] if v > 0] or [dmin])

    aspect = positive(row.get("aspect_ratio_max_over_min"))
    if aspect is None:
        aspect = dmax / dmin if dmin > 0 else 1.0
    return lx, ly, lz, dmin, dmax, aspect


def _row_volume(row):
    v = positive(row.get("solid_volume_mm3"))
    if v is not None:
        return v
    v = positive(row.get("bbox_volume_mm3"))
    return v if v is not None else 0.0


def build_adaptive_context(rows):
    """从整艘 AUV 的几何统计建立无名称依赖的尺度上下文。"""
    if not rows:
        raise RuntimeError("没有 PART_SUMMARY，无法建立自适应网格上下文。")

    # 优先读 ALL_SOLID_BODIES；没有时用所有 part bbox 的全局并集近似。
    all_row = None
    try:
        p = Path(PART_GEOMETRY_METRICS_PATH)
        data_lines = []
        for line in p.read_text(encoding="utf-8").splitlines():
            if line.strip() and not line.lstrip().startswith("#"):
                data_lines.append(line)
        if data_lines:
            for row in csv.DictReader(data_lines, delimiter="\t"):
                if str(row.get("record_type", "")).strip() == "ALL_SOLID_BODIES":
                    all_row = row
                    break
    except Exception:
        all_row = None

    if all_row is not None:
        dims = [
            positive(all_row.get("length_x_mm")),
            positive(all_row.get("width_y_mm")),
            positive(all_row.get("height_z_mm")),
        ]
        dims = [x for x in dims if x is not None]
    else:
        # 退化兜底：取各 part 在三个方向的最大范围。
        dims = [
            max([positive(r.get("length_x_mm")) or 0.0 for r in rows]),
            max([positive(r.get("width_y_mm")) or 0.0 for r in rows]),
            max([positive(r.get("height_z_mm")) or 0.0 for r in rows]),
        ]
        dims = [x for x in dims if x > 0]

    if len(dims) < 2:
        raise RuntimeError("模型整体包围盒维度不足，无法自适应定标。")

    dims_sorted = sorted(dims)
    L_model = dims_sorted[-1]
    D_ref = dims_sorted[-2]
    if D_ref <= 0:
        D_ref = max(dims_sorted[0], L_model * 0.1)

    # 主体不看名字：优先选实体体积最大者；体积不可用时选 bbox 体积最大者。
    dominant = max(rows, key=_row_volume)
    dominant_label = str(dominant.get("part_label", "")).strip()
    total_volume = sum(max(0.0, _row_volume(r)) for r in rows)
    dominant_fraction = (_row_volume(dominant) / total_volume) if total_volume > 0 else 1.0

    main_h = min(MAIN_L_FRACTION * L_model, MAIN_D_FRACTION * D_ref)
    global_floor = max(
        GLOBAL_FLOOR_D_FRACTION * D_ref,
        GLOBAL_FLOOR_L_FRACTION * L_model,
        main_h / 200.0,
    )
    main_h = max(main_h, 4.0 * global_floor)

    # 最长轴即默认来流方向，与 SpaceClaim AUTO_FLOW_AXIS_BY_LONGEST 保持一致。
    if all_row is not None:
        axis_dims = {
            "x": positive(all_row.get("length_x_mm")) or 0.0,
            "y": positive(all_row.get("width_y_mm")) or 0.0,
            "z": positive(all_row.get("height_z_mm")) or 0.0,
        }
        flow_axis = max(axis_dims, key=axis_dims.get)
    else:
        flow_axis = "auto_longest"
    env_axis = os.environ.get("AUV_FLOW_AXIS", "").strip().lower()
    if env_axis in ["x", "y", "z"]:
        flow_axis = env_axis

    global_center = {"x": 0.0, "y": 0.0, "z": 0.0}
    if all_row is not None:
        global_center = {
            "x": safe_float(all_row.get("center_x_mm"), 0.0),
            "y": safe_float(all_row.get("center_y_mm"), 0.0),
            "z": safe_float(all_row.get("center_z_mm"), 0.0),
        }

    ctx = {
        "L_model_mm": L_model,
        "D_ref_mm": D_ref,
        "global_center_mm": global_center,
        "dominant_label": dominant_label,
        "dominant_volume_fraction": dominant_fraction,
        "main_target_base_mm": main_h,
        "global_floor_mm": global_floor,
        "flow_axis": flow_axis,
        "part_count": len(rows),
    }
    return ctx


def infer_role(label, row, ctx):
    """完全基于几何量；label 只作为 ID，不参与判断。"""
    if str(label) == str(ctx["dominant_label"]):
        return "dominant_body", "largest_solid_volume_or_bbox"

    lx, ly, lz, dmin, dmax, aspect = get_box_dimensions(row)

    # 识别被拆成 nose/mid/tail 等多个实体的主体壳体段。
    # 不看名字，只看：沿流向占比、两个横向方向都具有实体厚度、且靠近整艇中心线。
    axis = str(ctx.get("flow_axis", "x"))
    dims_by_axis = {"x": lx, "y": ly, "z": lz}
    centers = {
        "x": safe_float(row.get("center_x_mm"), 0.0),
        "y": safe_float(row.get("center_y_mm"), 0.0),
        "z": safe_float(row.get("center_z_mm"), 0.0),
    }
    flow_extent = max(0.0, dims_by_axis.get(axis, dmax))
    cross_axes = [a for a in ["x", "y", "z"] if a != axis]
    cross_dims = [max(0.0, dims_by_axis.get(a, 0.0)) for a in cross_axes]
    cross_min = min(cross_dims) if cross_dims else 0.0
    gc = ctx.get("global_center_mm", {"x": 0.0, "y": 0.0, "z": 0.0})
    offset2 = 0.0
    for a in cross_axes:
        offset2 += (centers[a] - float(gc.get(a, 0.0))) ** 2
    centerline_offset = math.sqrt(offset2)

    if (
        flow_extent >= 0.08 * float(ctx["L_model_mm"]) and
        cross_min >= 0.15 * float(ctx["D_ref_mm"]) and
        centerline_offset <= 0.60 * float(ctx["D_ref_mm"])
    ):
        return "body_segment", "geometry_centerline_body_segment"

    thin_flag = str(row.get("thin_feature_flag", "")).strip().lower() == "yes"
    complexity = safe_float(row.get("complexity_score_0_10"), 0.0)
    thickness = positive(row.get("thickness_equiv_2v_over_area_mm"))
    feature = positive(row.get("feature_size_p10_est_mm"))

    thin_ratio = None
    if thickness is not None and dmin > 0:
        thin_ratio = thickness / dmin

    if thin_flag or complexity >= COMPLEXITY_SCORE_THRESHOLD or (thin_ratio is not None and thin_ratio < THIN_FEATURE_RATIO_THRESHOLD):
        return "thin_complex_appendage", "geometry_complexity_thickness"

    if aspect >= SLENDER_ASPECT_RATIO:
        return "slender_appendage", "geometry_aspect_ratio"

    # 如果局部 P10 特征远小于本部件尺度，也按复杂附体处理。
    if feature is not None and dmin > 0 and feature / dmin < 0.12:
        return "thin_complex_appendage", "geometry_small_feature_ratio"

    return "regular_appendage", "geometry_regular"


def _append_positive(candidates, name, value):
    try:
        v = float(value)
        if v > 0 and math.isfinite(v):
            candidates.append((name, v))
    except Exception:
        pass


def adaptive_target_for_role(row, role, ctx):
    _, _, _, dmin, _, _ = get_box_dimensions(row)
    main_h = float(ctx["main_target_base_mm"])
    floor_h = float(ctx["global_floor_mm"])

    edge_p10 = positive(row.get("edge_length_p10_mm"))
    thickness = positive(row.get("thickness_equiv_2v_over_area_mm"))
    feature = positive(row.get("feature_size_p10_est_mm"))
    gap = positive(row.get("bbox_gap_to_nearest_part_mm"))

    if role in ["dominant_body", "body_segment"]:
        # 分段艇体使用与主体同量级的尺度，避免被误当成小附体过度加密。
        return main_h, "min(0.01*L,0.10*Dref)", "scale_relative_main_body"

    candidates = []
    if role == "thin_complex_appendage":
        _append_positive(candidates, "0.20*h_main", COMPLEX_MAIN_FACTOR * main_h)
        _append_positive(candidates, "Dmin/15", dmin / COMPLEX_DMIN_DIV)
        if edge_p10 is not None:
            _append_positive(candidates, "0.40*edge_p10", COMPLEX_EDGE_FACTOR * edge_p10)
        if thickness is not None:
            _append_positive(candidates, "0.40*thickness_eq", COMPLEX_THICKNESS_FACTOR * thickness)
        if feature is not None:
            _append_positive(candidates, "0.40*feature_p10", COMPLEX_FEATURE_FACTOR * feature)
        if gap is not None:
            _append_positive(candidates, "0.30*nearest_gap", COMPLEX_GAP_FACTOR * gap)
        rule = "geometry_thin_complex"
    elif role == "slender_appendage":
        _append_positive(candidates, "0.30*h_main", SLENDER_MAIN_FACTOR * main_h)
        _append_positive(candidates, "Dmin/10", dmin / SLENDER_DMIN_DIV)
        if feature is not None:
            _append_positive(candidates, "0.60*feature_p10", SLENDER_FEATURE_FACTOR * feature)
        if gap is not None:
            _append_positive(candidates, "0.35*nearest_gap", SLENDER_GAP_FACTOR * gap)
        rule = "geometry_slender"
    else:
        _append_positive(candidates, "0.50*h_main", REGULAR_MAIN_FACTOR * main_h)
        _append_positive(candidates, "Dmin/8", dmin / REGULAR_DMIN_DIV)
        if feature is not None:
            _append_positive(candidates, "0.80*feature_p10", REGULAR_FEATURE_FACTOR * feature)
        if gap is not None:
            _append_positive(candidates, "0.40*nearest_gap", REGULAR_GAP_FACTOR * gap)
        rule = "geometry_regular"

    if not candidates:
        return max(floor_h, 0.5 * main_h), "fallback_relative_main", rule

    limiter, raw = min(candidates, key=lambda x: x[1])
    target = clamp(raw, floor_h, main_h)
    return target, limiter, rule


def compute_target(row, ctx):
    label = str(row.get("part_label", "")).strip()
    role, role_source = infer_role(label, row, ctx)
    base, limiter, rule = adaptive_target_for_role(row, role, ctx)
    final = nice_size(max(ctx["global_floor_mm"], base * MESH_LEVEL_SCALE))

    lx, ly, lz, dmin, dmax, aspect = get_box_dimensions(row)
    return {
        "part_label": label,
        "target_mesh_size_mm": final,
        "part_role": role,
        "role_source": role_source,
        "target_rule": rule,
        "dominant_limiter": limiter,
        "length_x_mm": lx,
        "width_y_mm": ly,
        "height_z_mm": lz,
        "D_min_mm": dmin,
        "D_max_mm": dmax,
        "aspect_ratio": aspect,
        "edge_length_p10_mm": safe_float(row.get("edge_length_p10_mm"), -1.0),
        "thickness_equiv_2v_over_area_mm": safe_float(row.get("thickness_equiv_2v_over_area_mm"), -1.0),
        "feature_size_p10_est_mm": safe_float(row.get("feature_size_p10_est_mm"), -1.0),
        "bbox_gap_to_nearest_part_mm": safe_float(row.get("bbox_gap_to_nearest_part_mm"), -1.0),
        "face_count": safe_float(row.get("face_count"), -1.0),
        "edge_count": safe_float(row.get("edge_count"), -1.0),
        "complexity_score": safe_float(row.get("complexity_score_0_10"), -1.0),
        "complexity_class": str(row.get("complexity_class", "")),
        "thin_feature_flag": str(row.get("thin_feature_flag", "")),
        "fill_ratio": safe_float(row.get("fill_ratio_solid_over_bbox"), -1.0),
        "solid_volume_mm3": safe_float(row.get("solid_volume_mm3"), -1.0),
        "base_target_before_scale_mm": base,
    }


def read_metrics(path):
    p = Path(path)

    if not p.exists():
        raise FileNotFoundError(
            f"找不到几何指标文件：{p}\n"
            "SpaceClaim 阶段应先输出该文件，请检查前一阶段是否成功。"
        )

    lines = []
    with p.open("r", encoding="utf-8", errors="ignore", newline="") as f:
        for line in f:
            s = line.strip()
            if not s:
                continue
            if s.startswith("#"):
                continue
            lines.append(line)

    if not lines:
        raise RuntimeError(f"几何指标文件为空：{p}")

    reader = csv.DictReader(lines, delimiter="\t")

    rows = []
    for row in reader:
        if str(row.get("record_type", "")).strip() != "PART_SUMMARY":
            continue

        label = str(row.get("part_label", "")).strip()

        if not label:
            continue

        if label.lower() in ["inlet", "outlet", "wall", "fluid", "all_solid_bodies"]:
            continue

        rows.append(row)

    if not rows:
        raise RuntimeError("没有读取到 PART_SUMMARY 部件。")

    return rows


def write_target_file(results):
    p = Path(TARGET_MESH_SIZE_PATH)
    p.parent.mkdir(parents=True, exist_ok=True)

    lines = []
    lines.append("# Fluent Meshing Target Mesh Size")
    lines.append("# Source geometry metrics: " + PART_GEOMETRY_METRICS_PATH)
    lines.append("# Unit: mm")
    lines.append("# This file is for Add Local Sizing / Face Size.")
    lines.append("# Generated by v109 geometry-driven adaptive meshing")
    lines.append("# Component names are identifiers only; sizing is geometry-driven.")
    lines.append("# MESH_LEVEL_SCALE = " + str(MESH_LEVEL_SCALE))
    lines.append("")

    header = [
        "part_label",
        "target_mesh_size_mm",
        "part_role",
        "target_rule",
        "dominant_limiter",
        "length_x_mm",
        "width_y_mm",
        "height_z_mm",
        "D_min_mm",
        "D_max_mm",
        "aspect_ratio",
        "role_source",
    ]
    lines.append("\t".join(header))

    for r in results:
        row = [
            r["part_label"],
            format_num(r["target_mesh_size_mm"]),
            r["part_role"],
            r["target_rule"],
            r["dominant_limiter"],
            format_num(r["length_x_mm"]),
            format_num(r["width_y_mm"]),
            format_num(r["height_z_mm"]),
            format_num(r["D_min_mm"]),
            format_num(r["D_max_mm"]),
            format_num(r["aspect_ratio"]),
            r["role_source"],
        ]
        lines.append("\t".join(row))

    p.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_diag_file(results):
    p = Path(TARGET_MESH_SIZE_DIAG_PATH)

    lines = []
    lines.append("# Target Mesh Size diagnostics")
    lines.append("# Unit: mm")
    lines.append("# Generated by v109 geometry-driven adaptive meshing")
    lines.append("")

    header = [
        "part_label",
        "target_mesh_size_mm",
        "base_target_before_scale_mm",
        "part_role",
        "target_rule",
        "dominant_limiter",
        "edge_length_p10_mm",
        "thickness_equiv_2v_over_area_mm",
        "feature_size_p10_est_mm",
        "face_count",
        "edge_count",
        "complexity_score",
        "complexity_class",
        "thin_feature_flag",
        "fill_ratio",
    ]
    lines.append("\t".join(header))

    for r in results:
        row = [
            r["part_label"],
            format_num(r["target_mesh_size_mm"]),
            format_num(r["base_target_before_scale_mm"]),
            r["part_role"],
            r["target_rule"],
            r["dominant_limiter"],
            format_num(r["edge_length_p10_mm"]),
            format_num(r["thickness_equiv_2v_over_area_mm"]),
            format_num(r["feature_size_p10_est_mm"]),
            format_num(r["face_count"]),
            format_num(r["edge_count"]),
            format_num(r["complexity_score"]),
            r["complexity_class"],
            r["thin_feature_flag"],
            format_num(r["fill_ratio"]),
        ]
        lines.append("\t".join(row))

    p.write_text("\n".join(lines) + "\n", encoding="utf-8")


def print_target_results(results):
    log()
    log("=" * 80)
    log("阶段 2 完成：Target Mesh Size 结果")
    log("=" * 80)

    for r in results:
        role_cn = ROLE_CN.get(r["part_role"], r["part_role"])
        log(
            f"{r['part_label']:<12s} {role_cn:<18s} "
            f"target={r['target_mesh_size_mm']:>7.3f} mm | "
            f"limiter={r['dominant_limiter']}"
        )

    log("=" * 80)
    log()


def compute_target_mesh_size_stage() -> None:
    log()
    log("=" * 80)
    log("阶段 2：几何特征驱动自适应 Target Mesh Size")
    log("=" * 80)
    log("读取： " + PART_GEOMETRY_METRICS_PATH)
    log("输出： " + TARGET_MESH_SIZE_PATH)

    rows = read_metrics(PART_GEOMETRY_METRICS_PATH)
    ctx = build_adaptive_context(rows)
    results = [compute_target(row, ctx) for row in rows]
    results.sort(key=lambda x: x["target_mesh_size_mm"])

    write_target_file(results)
    write_diag_file(results)
    print_target_results(results)

    summary = dict(ctx)
    summary["input_model"] = str(INPUT_MODEL)
    summary["work_dir"] = WORK_DIR
    summary["mesh_level_scale"] = MESH_LEVEL_SCALE
    summary["boundary_layers"] = 5
    summary["mesh_quality_strategy"] = "v112: curvature12_gap5_local_refinement_surface_improve_BLtransition0.30"
    summary["parts"] = [
        {
            "label": r["part_label"],
            "role": r["part_role"],
            "role_source": r["role_source"],
            "target_mesh_size_mm": r["target_mesh_size_mm"],
            "limiter": r["dominant_limiter"],
        }
        for r in results
    ]
    Path(ADAPTIVE_SUMMARY_PATH).write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    log("自适应全局尺度:")
    log("  L_model = {:.6g} mm".format(ctx["L_model_mm"]))
    log("  D_ref   = {:.6g} mm".format(ctx["D_ref_mm"]))
    log("  dominant body = {}".format(ctx["dominant_label"]))
    log("  main target base = {:.6g} mm".format(ctx["main_target_base_mm"]))
    log("  global mesh floor = {:.6g} mm".format(ctx["global_floor_mm"]))
    log("  flow axis = {}".format(ctx["flow_axis"]))
    log("Adaptive summary： " + ADAPTIVE_SUMMARY_PATH)
    log("Target Mesh Size： " + TARGET_MESH_SIZE_PATH)
    log("诊断文件： " + TARGET_MESH_SIZE_DIAG_PATH)


# ============================================================
# 融合流程阶段 A：先执行 v102，确保 v105 所需文件已经生成
# ============================================================

if __name__ == "__main__":
    log("=" * 80)
    log("v114 阶段 A：任意 CAD -> SpaceClaim -> 几何特征提取")
    log("=" * 80)
    run_spaceclaim_stage()
    compute_target_mesh_size_stage()

# 保存 v102 动态输出路径，供下面的 v105 网格阶段直接使用。
PIPELINE_WORK_DIR = WORK_DIR
PIPELINE_OUTPUT_SCDOC = OUTPUT_SCDOC
PIPELINE_SURFACE_LABELS_PATH = SURFACE_LABELS_PATH
PIPELINE_PART_GEOMETRY_METRICS_PATH = PART_GEOMETRY_METRICS_PATH
PIPELINE_TARGET_MESH_SIZE_PATH = TARGET_MESH_SIZE_PATH

if __name__ == "__main__":
    log("=" * 80)
    log("v114 自适应几何分析完成，开始 Fluent Meshing + Solver 全流程")
    log("  geometry      = " + PIPELINE_OUTPUT_SCDOC)
    log("  surface labels = " + PIPELINE_SURFACE_LABELS_PATH)
    log("  geometry metrics = " + PIPELINE_PART_GEOMETRY_METRICS_PATH)
    log("  target mesh size = " + PIPELINE_TARGET_MESH_SIZE_PATH)
    log("=" * 80)


# ============================================================
# 融合流程阶段 B：以下保留 v105 Fluent Meshing 网格逻辑
# v112：在 v112 基础上增加局部体加密缓冲、Surface Mesh 预改善，并优化曲率/间隙/边界层过渡
# ============================================================

import os
import time
import re
import csv
import ansys.fluent.core as pyfluent

print("=" * 70)
print(" Fluent 全流程启动：v114 | v112 自适应高质量网格 + Solver 多速度求解")
print("=" * 70)

# ========================================================
# 1. 参数区：网格阶段
# ========================================================

GEOMETRY_PATH = PIPELINE_OUTPUT_SCDOC
FLUENT_EXE_PATH = r"D:\Program Files\ANSYS Inc\v241\fluent\ntbin\win64\fluent.exe"
WORK_DIR = PIPELINE_WORK_DIR
SURFACE_LABELS_PATH = PIPELINE_SURFACE_LABELS_PATH

_cores_env = os.environ.get("AUV_CORES", "").strip()
PROCESSOR_COUNT = int(RUNTIME_ARGS.cores) if RUNTIME_ARGS.cores is not None else (int(_cores_env) if _cores_env else 6)
if PROCESSOR_COUNT <= 0:
   raise ValueError(f"--cores 必须 > 0，收到: {PROCESSOR_COUNT}")

# v116：Meshing 与 Solver 使用独立 Fluent 进程。
# 只改变二者连接方式，不改变用户参考代码的求解逻辑。
USE_FRESH_SOLVER_SESSION = True
FRESH_SOLVER_STARTUP_WAIT_SEC = 8
FRESH_SOLVER_AFTER_READ_MESH_WAIT_SEC = 5
CLOSE_MESHING_BEFORE_SOLVER = True


# --------------------------------------------------------
# 局部面尺寸，单位 mm
# v103 关键修改：
# 不再在 Fluent Meshing 脚本内部写死 LOCAL_SIZE_RULES；
# 直接读取 v102 / v101 输出的 auv320_target_mesh_size.txt。
#
# target_mesh_size_mm 对应 Fluent Meshing:
# Add Local Sizing -> Face Size -> Target Mesh Size
# --------------------------------------------------------

TARGET_MESH_SIZE_PATH = PIPELINE_TARGET_MESH_SIZE_PATH
PART_GEOMETRY_METRICS_PATH = PIPELINE_PART_GEOMETRY_METRICS_PATH


def read_target_mesh_size_rules(path):
   """
   读取 target mesh size 文件，生成：
      {part_label: target_mesh_size_mm}

   支持 v101/v102 输出格式：
      part_label    target_mesh_size_mm    part_role ...
   自动跳过 # 注释行和空行。
   """
   rules = {}

   if not os.path.exists(path):
      raise FileNotFoundError(
         "找不到 Target Mesh Size 文件: " + str(path) + "\n"
         "请先运行 run_all_v102_sc_to_target_mesh_size_one_click.py，"
         "或者确认 auv320_target_mesh_size.txt 已存在。"
      )

   data_lines = []
   with open(path, "r", encoding="utf-8", errors="ignore", newline="") as f:
      for line in f:
         s = line.strip()
         if not s:
            continue
         if s.startswith("#"):
            continue
         data_lines.append(line)

   if len(data_lines) == 0:
      raise RuntimeError("Target Mesh Size 文件没有有效数据: " + str(path))

   reader = csv.DictReader(data_lines, delimiter="\t")

   for row in reader:
      label = str(row.get("part_label", "")).strip()
      size_text = str(row.get("target_mesh_size_mm", "")).strip()

      if not label:
         continue

      if label.lower() in ["inlet", "outlet", "wall", "fluid", "all_solid_bodies"]:
         continue

      try:
         size = float(size_text)
      except Exception:
         print("   跳过无效 target size: label={} value={}".format(label, size_text))
         continue

      if size <= 0:
         print("   跳过非正 target size: label={} value={}".format(label, size_text))
         continue

      rules[label] = size

   if len(rules) == 0:
      raise RuntimeError("没有从 Target Mesh Size 文件读取到任何有效部件尺寸: " + str(path))

   print("=" * 70)
   print("已读取 Target Mesh Size 文件:")
   print("  " + str(path))
   print("LOCAL_SIZE_RULES =")
   for k in sorted(rules.keys()):
      print("  {} : {} mm".format(k, rules[k]))
   print("=" * 70)

   return rules


LOCAL_SIZE_RULES = read_target_mesh_size_rules(TARGET_MESH_SIZE_PATH)


def _safe_float(value, default=None):
   try:
      if value is None:
         return default
      s = str(value).strip()
      if s == "" or s == "-":
         return default
      return float(s)
   except Exception:
      return default


def _clamp(value, lower, upper):
   return max(lower, min(upper, value))


def _round_down_to_base(value, base):
   try:
      base = float(base)
      value = float(value)
      if base <= 0:
         return value
      if value < base:
         return value
      return int(value / base) * base
   except Exception:
      return value


def read_all_solid_bbox_from_metrics(path):
   """
   从 auv320_part_geometry_metrics.txt 读取整体外包围盒尺寸。

   优先读取：
      record_type = ALL_SOLID_BODIES

   返回：
      {
         "length_x_mm": ...,
         "width_y_mm": ...,
         "height_z_mm": ...
      }
   """
   if not os.path.exists(path):
      print("   未找到几何指标文件，SURF_MAX_SIZE 将使用 target size 兜底规则: " + str(path))
      return None

   data_lines = []
   with open(path, "r", encoding="utf-8", errors="ignore", newline="") as f:
      for line in f:
         s = line.strip()
         if not s:
            continue
         if s.startswith("#"):
            continue
         data_lines.append(line)

   if len(data_lines) == 0:
      print("   几何指标文件为空，SURF_MAX_SIZE 将使用 target size 兜底规则: " + str(path))
      return None

   reader = csv.DictReader(data_lines, delimiter="\t")

   part_rows = []

   for row in reader:
      rec_type = str(row.get("record_type", "")).strip()

      if rec_type == "ALL_SOLID_BODIES":
         lx = _safe_float(row.get("length_x_mm"))
         wy = _safe_float(row.get("width_y_mm"))
         hz = _safe_float(row.get("height_z_mm"))

         if lx is not None and wy is not None and hz is not None:
            return {
               "length_x_mm": lx,
               "width_y_mm": wy,
               "height_z_mm": hz,
            }

      if rec_type == "PART_SUMMARY":
         part_rows.append(row)

   # 如果没有 ALL_SOLID_BODIES，就用 PART_SUMMARY 的极值合并估算。
   if len(part_rows) > 0:
      xmin_list = []
      xmax_list = []
      ymin_list = []
      ymax_list = []
      zmin_list = []
      zmax_list = []

      for row in part_rows:
         xmin = _safe_float(row.get("xmin_mm"))
         xmax = _safe_float(row.get("xmax_mm"))
         ymin = _safe_float(row.get("ymin_mm"))
         ymax = _safe_float(row.get("ymax_mm"))
         zmin = _safe_float(row.get("zmin_mm"))
         zmax = _safe_float(row.get("zmax_mm"))

         if None not in [xmin, xmax, ymin, ymax, zmin, zmax]:
            xmin_list.append(xmin)
            xmax_list.append(xmax)
            ymin_list.append(ymin)
            ymax_list.append(ymax)
            zmin_list.append(zmin)
            zmax_list.append(zmax)

      if len(xmin_list) > 0:
         return {
            "length_x_mm": max(xmax_list) - min(xmin_list),
            "width_y_mm": max(ymax_list) - min(ymin_list),
            "height_z_mm": max(zmax_list) - min(zmin_list),
         }

   print("   未能从几何指标文件读取整体外包围盒，SURF_MAX_SIZE 将使用兜底规则。")
   return None


def _model_L_D_from_bbox(bbox):
   dims = [
      float(bbox["length_x_mm"]),
      float(bbox["width_y_mm"]),
      float(bbox["height_z_mm"]),
   ]
   dims = sorted([abs(x) for x in dims if abs(x) > 0])
   if len(dims) < 2:
      raise RuntimeError("ALL_SOLID_BODIES bbox 维度不足")
   return dims[-1], dims[-2]


def compute_surface_size_controls(target_size_rules, metrics_path):
   """完全按当前模型相对尺度计算 Surface Mesh Min/Max。"""
   h_values = [float(v) for v in target_size_rules.values() if _safe_float(v) and _safe_float(v) > 0]
   if len(h_values) == 0:
      raise RuntimeError("LOCAL_SIZE_RULES 为空，无法计算 Surface Mesh 全局尺寸。")

   h_local_min = min(h_values)
   h_body = max(h_values)
   surf_min = max(0.50 * h_local_min, h_local_min / 4.0)

   bbox = read_all_solid_bbox_from_metrics(metrics_path)
   if bbox is not None:
      L_model, D_ref = _model_L_D_from_bbox(bbox)
      # 对当前 auv320（约 L=3157,D=320,h_body≈30）仍会落在约 600 mm 量级，
      # 但换尺度后同比缩放。
      surf_max_raw = min(0.20 * L_model, 2.0 * D_ref, 20.0 * h_body)
      lower = max(5.0 * h_body, 0.25 * D_ref)
      upper = max(lower, min(0.35 * L_model, 3.0 * D_ref, 30.0 * h_body))
      surf_max = clamp(surf_max_raw, lower, upper)
   else:
      L_model = -1.0
      D_ref = -1.0
      surf_max = 20.0 * h_body

   surf_min = nice_size(surf_min)
   surf_max = nice_size(max(surf_max, 2.0 * h_body))

   print("=" * 70)
   print("自适应 Surface Mesh 全局尺寸:")
   print("  h_local_min = {} mm".format(h_local_min))
   print("  h_body/max target = {} mm".format(h_body))
   print("  L_model = {} mm".format(L_model))
   print("  D_ref   = {} mm".format(D_ref))
   print("  SURF_MIN_SIZE = {} mm".format(surf_min))
   print("  SURF_MAX_SIZE = {} mm".format(surf_max))
   print("=" * 70)
   return float(surf_min), float(surf_max)


SURF_MIN_SIZE, SURF_MAX_SIZE = compute_surface_size_controls(
   target_size_rules=LOCAL_SIZE_RULES,
   metrics_path=PART_GEOMETRY_METRICS_PATH,
)


def compute_volume_max_cell_length(target_size_rules, metrics_path, surf_max_size):
   """体网格最大尺度按 L/D/主体 target 同比缩放；写入方式仍沿用 v105。"""
   h_values = [float(v) for v in target_size_rules.values() if _safe_float(v) and _safe_float(v) > 0]
   if len(h_values) == 0:
      raise RuntimeError("LOCAL_SIZE_RULES 为空，无法计算 Volume Max Cell Length。")
   h_body = max(h_values)

   bbox = read_all_solid_bbox_from_metrics(metrics_path)
   if bbox is not None:
      L_model, D_ref = _model_L_D_from_bbox(bbox)
      raw = min(0.20 * L_model, 2.0 * D_ref, 20.0 * h_body)
      lower = max(float(surf_max_size), 5.0 * h_body)
      upper = max(lower, min(0.35 * L_model, 3.0 * D_ref, 30.0 * h_body))
      vol_max = clamp(raw, lower, upper)
   else:
      L_model = -1.0
      D_ref = -1.0
      vol_max = float(surf_max_size)

   vol_max = nice_size(max(vol_max, float(surf_max_size)))
   print("=" * 70)
   print("自适应 Volume Mesh Max Cell Length 目标值:")
   print("  L_model = {} mm | D_ref = {} mm | h_body = {} mm".format(L_model, D_ref, h_body))
   print("  SURF_MAX_SIZE = {} mm".format(surf_max_size))
   print("  VOL_MAX_SIZE  = {} mm".format(vol_max))
   print("=" * 70)
   return float(vol_max)


# 兜底列表：如果 SpaceClaim 没有写出 SURFACE_LABELS_PATH，就使用这些默认 label。
FALLBACK_AUV_SURFACE_LABELS = list(LOCAL_SIZE_RULES.keys())

# 运行时会由 SpaceClaim 写出的 try_surface_labels.txt 自动填充。
AUV_SURFACE_LABELS = []

# 与旧函数兼容，真正运行时会由 build_local_face_sizings() 更新。
LOCAL_FACE_SIZINGS = []

# 全局表面网格控制
# SURF_MIN_SIZE / SURF_MAX_SIZE 已由 compute_surface_size_controls() 自动计算。
SURF_GROWTH_RATE = 1.1
CURVATURE_ANGLE = 12
CELLS_PER_GAP = 5

# 边界层控制
ENABLE_BOUNDARY_LAYER = True
BL_LAYERS = 5
BL_TRANS_RATIO = 0.30
BL_GROWTH_RATE = 1.1

# --------------------------------------------------------
# v112 高质量网格控制（Fluent 2024 R1 稳定版）
# --------------------------------------------------------
# 1) Create Local Refinement Regions：围绕艇体和附体自动建立两级体加密缓冲区。
#    这些不是额外 CAD 实体，而是 Fluent Meshing 内部的 bounding-box refinement regions。
ENABLE_LOCAL_REFINEMENT_REGIONS = False
# v112/v241: disabled in the main workflow because the user's actual Fluent 2024 R1
# transcript showed repeated Cortex SEGMENTATION VIOLATION immediately after the
# local-sizing/refinement insertion stage. Re-enable only after a v241-specific
# Create Local Refinement Regions state has been verified from the live task state.

# 附体两级缓冲：inner 约 2*h_surface，outer 约 5*h_surface；
# 为避免普通附体外层过粗，分别限制为主体目标尺寸的 0.5 / 0.75。
APPENDAGE_INNER_SIZE_FACTOR = 2.0
APPENDAGE_OUTER_SIZE_FACTOR = 5.0
APPENDAGE_INNER_MAX_MAIN_FACTOR = 0.50
APPENDAGE_OUTER_MAX_MAIN_FACTOR = 0.75

# bounding box 外扩：至少为若干倍局部目标尺寸，同时按部件 bbox 比例外扩。
APPENDAGE_INNER_PAD_TARGET_FACTOR = 3.0
APPENDAGE_OUTER_PAD_TARGET_FACTOR = 8.0
APPENDAGE_INNER_MIN_RATIO = 0.15
APPENDAGE_OUTER_MIN_RATIO = 0.40
REFINEMENT_MAX_PAD_RATIO = 2.0

# 主体附近也建立两级缓冲，避免 30 mm 级艇体表面直接过渡到数百 mm 远场 tetra。
BODY_INNER_SIZE_FACTOR = 1.50
BODY_OUTER_SIZE_FACTOR = 3.00
BODY_INNER_PAD_RATIO = 0.10
BODY_OUTER_PAD_RATIO = 0.35

# 2) Surface Mesh 后、Describe Geometry 前执行 Improve Surface Mesh。
ENABLE_IMPROVE_SURFACE_MESH = True
SURFACE_FACE_QUALITY_LIMIT = 0.40
SURFACE_IMPROVE_ITERATIONS = 5

# 体网格控制
VOL_FILL_TYPE = "tetrahedral"
VOL_MIN_SIZE = float(SURF_MIN_SIZE)  # 自适应：不再固定 15 mm

# 体网格最大单元边长，自动函数化：
# 对应 Fluent Meshing 中 tetrahedron / polyhedron / poly-hexcore volume fill 的 Max Cell Length。
VOL_MAX_SIZE = compute_volume_max_cell_length(
   target_size_rules=LOCAL_SIZE_RULES,
   metrics_path=PART_GEOMETRY_METRICS_PATH,
   surf_max_size=SURF_MAX_SIZE,
)

# 体网格质量改进控制
ENABLE_IMPROVE_MESH_QUALITY = True
MESH_QUALITY_METHOD = "Orthogonal"
MESH_QUALITY_LIMIT = 0.30
MESH_QUALITY_MIN_ANGLE = 0
MESH_QUALITY_IMPROVE_ITERATIONS = 10
MESH_QUALITY_SMOOTH_REMAINING = "yes"
# 兼容旧变量名：旧代码中 percent 实际对应 GUI 里的“单元质量限制”。
MESH_QUALITY_IMPROVE_PERCENT = MESH_QUALITY_LIMIT

# 区域设置：
# fluid 必须是流体区；其它 fluid_1、fluid_2 等内部封闭区全部设置为 dead。
PREFERRED_FLUID_REGION_NAME = "fluid"

# 外边界和 AUV 表面
OUTER_BOUNDARY_LABELS = ["inlet", "outlet", "wall", "fluid"]
OUTER_WALL_LABELS = ["wall"]
REPORT_WALL_LABELS = FALLBACK_AUV_SURFACE_LABELS

# ========================================================
# 2. 参数区：求解器阶段
# ========================================================

TARGET_MATERIAL = "water-liquid"

PLANES_TO_CREATE = [
   {"name": "xoy", "method": "xy-plane", "coord": 0.0},
   {"name": "xoz", "method": "zx-plane", "coord": 0.0},
]

# ========================================================
# 多速度批量计算设置
# ========================================================
# 后续只需要改这个列表，就能控制要计算的速度大小和数量。
# 例如：VELOCITY_LIST = [2.0] 表示只算 1 个速度；
#     VELOCITY_LIST = [1.0, 2.0, 3.0, 4.0] 表示算 4 个速度；
#     VELOCITY_LIST = [1,2,3,4,5,6,7,8,9,10] 表示算 10 个速度。
# 单位：m/s
_velocity_values = RUNTIME_ARGS.velocities
if _velocity_values is None:
   _velocity_values = _parse_velocity_env(os.environ.get("AUV_VELOCITIES", ""))
if not _velocity_values:
   _velocity_values = [2.0, 4.0]
VELOCITY_LIST = _normalize_velocity_list(_velocity_values)

# 当前速度变量由程序循环自动赋值；不要在循环外手动改它。
INLET_VELOCITY = VELOCITY_LIST[0]
REPORT_NAME = "force_drag"
REPORT_SWAY_Y_NAME = "force_sway_y"
REPORT_HEAVE_Z_NAME = "force_heave_z"
REPORT_DRAG_PRESSURE_NAME = "force_drag_pressure"
REPORT_DRAG_FRICTION_NAME = "force_drag_friction"
REPORT_XOY_STATIC_PRESSURE_NAME = "xoy_static_pressure"
REPORT_SURFACE_PRESSURE_AVG_NAME = "surface_pressure_vertex_average"
REPORT_SURFACE_PRESSURE_MAX_NAME = "surface_pressure_vertex_maximum"
REPORT_FILE_EXT = ".out"
_iterations_env = os.environ.get("AUV_ITERATIONS", "").strip()
ITERATIONS = int(RUNTIME_ARGS.iterations) if RUNTIME_ARGS.iterations is not None else (int(_iterations_env) if _iterations_env else 100)
if ITERATIONS <= 0:
   raise ValueError(f"--iterations 必须 > 0，收到: {ITERATIONS}")

# 残差收敛标准：continuity、x/y/z-velocity、k、omega 统一设置。
RESIDUAL_ABS_CRITERIA = 1.0e-7

# 计算完成后保持 Fluent GUI 打开，方便查看结果
KEEP_FLUENT_OPEN = bool(getattr(RUNTIME_ARGS, "keep_fluent_open", False))

# 体网格高级选项：对应截图中的设置
VOL_SOLVER = "Fluent"
VOL_BUFFER_LAYERS = 2
VOL_PEEL_LAYERS = 1
VOL_QUALITY_METHOD = "Orthogonal"
VOL_QUALITY_IMPROVE_LIMIT = 0.05
VOL_USE_SIZE_FIELD = "no"
VOL_POLY_MAX_CELL_SKEW_ANGLE = 30
VOL_AVOID_ONE_EIGHT_TRANSITION = "yes"
VOL_CHECK_SELF_PROXIMITY = "no"

CONTOURS_TO_CREATE = [
   {"name": "xoy_vel", "field": "velocity-magnitude", "surface": "xoy"},
   {"name": "xoz_vel", "field": "velocity-magnitude", "surface": "xoz"},
   {"name": "xoy_static_pressure", "field": "pressure", "surface": "xoy"},
   {"name": "xoz_static_pressure", "field": "pressure", "surface": "xoz"},
]

# ========================================================
# 2.1 后处理图片保存设置
# ========================================================
# 图片会保存在 WORK_DIR，也就是 force_drag.out 相同目录。
SAVE_CONTOUR_IMAGES = not bool(getattr(RUNTIME_ARGS, "skip_contours", False))
PICTURE_X_RESOLUTION = 1920
PICTURE_Y_RESOLUTION = 1080

# 视角说明：
# plus_z  : 从 +Z 方向看向 xoy 平面，+Z 轴垂直屏幕并朝向观察者。
# minus_y : 从 -Y 方向看向 xoz 平面，-Y 轴垂直屏幕并朝向观察者。
CONTOUR_IMAGE_SPECS = [
   {"contour": "xoy_vel", "file": "xoy.png", "view": "plus_z"},
   {"contour": "xoy_static_pressure", "file": "xoy_static_pressure.png", "view": "plus_z"},
   {"contour": "xoz_vel", "file": "xoz.png", "view": "minus_y"},
   {"contour": "xoz_static_pressure", "file": "xoz_static_pressure.png", "view": "minus_y"},
]

# ========================================================
# 3. 通用工具函数
# ========================================================

if not os.path.exists(WORK_DIR):
   os.makedirs(WORK_DIR)


def apply_task_state(task, state_dict, label=""):
   """兼容不同 PyFluent/Workflow 参数写法。"""
   try:
      task.Arguments.set_state(state_dict)
      return True
   except Exception as e1:
      try:
         task.Arguments = state_dict
         return True
      except Exception as e2:
         if label:
            print(f"   set_state 失败: {label} | {e1} | {e2}")
         return False


def try_update_task(task, methods):
   """按顺序尝试 Workflow task 的刷新/执行方法。"""
   for method in methods:
      try:
         if method == "UpdateChildTasks_SetupTypeChanged":
            task.UpdateChildTasks(SetupTypeChanged=True)
         else:
            getattr(task, method)()
         return True
      except Exception:
         pass
   return False


def get_task_state(task):
   try:
      return task.Arguments.get_state()
   except Exception:
      return None


def print_task_state(task, title):
   try:
      print(f"\n--- {title} state ---")
      print(task.Arguments.get_state())
      print("--- end state ---\n")
   except Exception as e:
      print(f"   无法读取 {title} state: {e}")


def flatten_strings(obj):
   found = []
   if isinstance(obj, str):
      found.append(obj)
   elif isinstance(obj, (list, tuple, set)):
      for item in obj:
         found.extend(flatten_strings(item))
   elif isinstance(obj, dict):
      for key, value in obj.items():
         found.extend(flatten_strings(key))
         found.extend(flatten_strings(value))
   return found


def is_region_like_name(text):
   if not isinstance(text, str):
      return False
   t = text.strip()
   if not t:
      return False
   low = t.lower()
   if low == "fluid":
      return True
   if low.startswith("fluid_"):
      tail = low.replace("fluid_", "", 1)
      return tail.isdigit()
   return False


def discover_region_names(update_regions_task):
   """从 Update Regions state 中尽量发现 fluid / fluid_1 / fluid_2 等区域。"""
   state = get_task_state(update_regions_task)
   candidates = []
   for item in flatten_strings(state):
      if is_region_like_name(item) and item not in candidates:
         candidates.append(item)

   # 兜底：当前模型通常出现这三个。
   for name in ["fluid", "fluid_1", "fluid_2"]:
      if name not in candidates:
         candidates.append(name)

   return candidates



def read_auv_surface_labels():
   labels = []

   try:
      if os.path.exists(SURFACE_LABELS_PATH):
         with open(SURFACE_LABELS_PATH, "r", encoding="utf-8") as f:
            for line in f:
               item = line.strip()
               if item and item not in labels:
                  labels.append(item)
   except Exception as e:
      print(f"   读取 AUV 表面标签文件失败: {SURFACE_LABELS_PATH} | {e}")

   if len(labels) == 0:
      labels = list(FALLBACK_AUV_SURFACE_LABELS)

   print(f"   ▶ 当前 AUV 表面标签 = {labels}")
   return labels


def is_other_surface_label(label):
   low = str(label).lower().strip()

   if not low.startswith("other"):
      return False

   tail = low.replace("other", "", 1)

   if tail == "":
      return False

   for ch in tail:
      if not ch.isdigit():
         return False

   return True


def get_auv_surface_base(label):
   low = str(label).lower().strip()

   if low.endswith("-fluid") or "-fluid-" in low or low.endswith("_fluid") or "_fluid_" in low:
      return None

   if low.startswith("region"):
      return None

   if low == "auv_body":
      return "auv_body"

   bases = ["shaft", "propeller", "duct", "fin", "sonar", "antenna", "payload"]

   for base in bases:
      if low == base:
         return base

      if low.startswith(base):
         tail = low[len(base):]

         while len(tail) > 0 and tail[0] in ["_", "-", " "]:
            tail = tail[1:]

         if tail != "":
            ok = True
            for ch in tail:
               if not ch.isdigit():
                  ok = False
                  break

            if ok:
               return base

   if is_other_surface_label(low):
      return "other"

   return None


def build_local_face_sizings(labels):
   """
   v103：
   根据当前模型自适应 target_mesh_size.txt 中的 exact part_label 创建局部 Face Size。
   不再通过 auv_body / shaft / propeller 等旧规则映射。
   """
   sizings = []

   # target 文件是主依据；surface_labels 文件只用于确认模型中出现过哪些 label。
   available = []
   for label in labels:
      label = str(label).strip()
      if label and label not in available:
         available.append(label)

   if len(available) == 0:
      available = list(LOCAL_SIZE_RULES.keys())

   available_lower = {str(x).lower(): str(x) for x in available}

   for target_label, target_size in LOCAL_SIZE_RULES.items():
      # 优先用 surface_labels 中真实出现的大小写；找不到时仍按 target_label 尝试。
      actual_label = available_lower.get(str(target_label).lower(), str(target_label))

      sizings.append({
         "name": "size_" + str(actual_label),
         "labels": [str(actual_label)],
         "size": float(target_size),
      })

   print("   ▶ 本次局部面尺寸来自 Target Mesh Size 文件：")
   for s in sizings:
      print("     {} -> labels={} -> Target Mesh Size = {} mm".format(
         s["name"], s["labels"], s["size"]
      ))

   return sizings

def surface_labels_from_settings():
   global AUV_SURFACE_LABELS

   if len(AUV_SURFACE_LABELS) == 0:
      AUV_SURFACE_LABELS = read_auv_surface_labels()

   return list(AUV_SURFACE_LABELS)


def boundary_layer_labels_from_settings():
   """
   通用排除法：
   除 inlet / outlet / wall / fluid 以外，SpaceClaim 实际写出的所有 AUV 表面都加边界层。
   """
   exclude = set([x.lower() for x in OUTER_BOUNDARY_LABELS])

   labels = []

   for label in surface_labels_from_settings():
      if label.lower() not in exclude and label not in labels:
         labels.append(label)

   return labels


def get_task_by_name(workflow, task_name):
   try:
      return workflow.TaskObject[task_name]
   except Exception:
      return None


def find_task_by_exact_or_contains(workflow, name):
   try:
      return workflow.TaskObject[name]
   except Exception:
      pass

   try:
      for key in workflow.TaskObject.keys():
         key_str = str(key)
         if key_str.lower() == name.lower() or name.lower() in key_str.lower():
            return workflow.TaskObject[key]
   except Exception:
      pass

   return None


def patch_size_in_state_object(obj, target_size):
   """
   只修改“明确表示目标尺寸”的字段，避免把 GrowthRate 等参数误改。
   兼容字段名包括：
   TargetMeshSize, Target Mesh Size, TargetSize, BOISize, BOIFaceSize, FaceSize, Size。
   """
   changed = False

   def patch(value):
      nonlocal changed

      if isinstance(value, dict):
         out = {}
         for k, v in value.items():
            kl = str(k).lower().replace(" ", "").replace("_", "").replace("-", "")

            is_size_key = (
               "targetmeshsize" in kl or
               "targetsize" in kl or
               "boisize" in kl or
               "boifacesize" in kl or
               "facesize" in kl or
               kl in ["size", "meshsizemm", "targetmeshsizemm"]
            )

            # 不要误改 GrowthRate、Curvature、CellsPerGap
            bad_key = (
               "growth" in kl or
               "rate" in kl or
               "curvature" in kl or
               "angle" in kl or
               "gap" in kl or
               "layer" in kl
            )

            if is_size_key and not bad_key:
               if isinstance(v, (int, float)):
                  out[k] = float(target_size)
                  changed = True
               elif isinstance(v, str):
                  try:
                     float(v)
                     out[k] = str(float(target_size))
                     changed = True
                  except Exception:
                     out[k] = patch(v)
               else:
                  out[k] = patch(v)
            else:
               out[k] = patch(v)
         return out

      if isinstance(value, list):
         return [patch(x) for x in value]

      if isinstance(value, tuple):
         return tuple(patch(x) for x in value)

      return value

   patched = patch(obj)
   return patched, changed


def patch_label_list_in_state_object(obj, labels):
   """在已有 state 中，尽量把标签选择字段改成目标 labels。"""
   changed = False

   def patch(value):
      nonlocal changed

      if isinstance(value, dict):
         out = {}
         for k, v in value.items():
            kl = str(k).lower().replace(" ", "").replace("_", "").replace("-", "")
            is_label_key = (
               "boifacelabellist" in kl or
               "labellist" in kl or
               "labelselection" in kl or
               "boundarylabellist" in kl or
               "boundarylabel" in kl
            )
            if is_label_key:
               out[k] = list(labels)
               changed = True
            else:
               out[k] = patch(v)
         return out

      if isinstance(value, list):
         return [patch(x) for x in value]

      if isinstance(value, tuple):
         return tuple(patch(x) for x in value)

      return value

   patched = patch(obj)
   return patched, changed


# ========================================================
# 4. Workflow 设置函数
# ========================================================

def add_face_local_sizing(workflow, name, labels, size):
   """
   Add Local Sizing 的关键修正版。

   日志里出现：
      A local size of 15.192623 was added ...
      Illegitimate input ... TargetMeshSize

   原因是：Fluent 2024R1 的 Watertight Add Local Sizing 命令
   不接受 TargetMeshSize 这个字段；GUI 中的 Target Mesh Size
   在 workflow state 里对应的是 BOISize。

   因此这里严格按原始 fluent solution.py 的流程创建子任务，
   但在 AddChildAndUpdate 前把 BOISize 写入父任务。
   不再写 TargetMeshSize，避免非法输入。
   """
   print(f"   ▶ 添加局部尺寸: {name} | labels={labels} | target={size} mm")

   parent = workflow.TaskObject["Add Local Sizing"]

   local_growth_rate = float(SURF_GROWTH_RATE)

   parent_state_candidates = [
      {
         "AddChild": "yes",
         "BOIControlName": name,
         "BOIExecution": "Face Size",
         "BOIFaceLabelList": labels,
         "BOISize": float(size),
         "GrowthRate": local_growth_rate,
         "Growth Rate": local_growth_rate,
         "BOIGrowthRate": local_growth_rate,
         "SizeGrowthRate": local_growth_rate,
         "SizingGrowthRate": local_growth_rate,
      },
      {
         "AddChild": "yes",
         "BOIControlName": name,
         "BOIExecution": "Face Size",
         "CompleteFaceLabelList": labels,
         "BOIFaceLabelList": labels,
         "BOISize": float(size),
         "GrowthRate": local_growth_rate,
         "Growth Rate": local_growth_rate,
         "BOIGrowthRate": local_growth_rate,
         "SizeGrowthRate": local_growth_rate,
         "SizingGrowthRate": local_growth_rate,
      },
      # 兜底：如果 BOISize 不被当前版本接受，至少先创建任务。
      {
         "AddChild": "yes",
         "BOIControlName": name,
         "BOIExecution": "Face Size",
         "BOIFaceLabelList": labels,
         "GrowthRate": local_growth_rate,
         "Growth Rate": local_growth_rate,
         "BOIGrowthRate": local_growth_rate,
         "SizeGrowthRate": local_growth_rate,
         "SizingGrowthRate": local_growth_rate,
      },
   ]

   ok = False
   last_err = None
   for st in parent_state_candidates:
      try:
         apply_task_state(parent, st, f"{name} parent")
         parent.AddChildAndUpdate()
         ok = True
         break
      except Exception as e:
         last_err = e

   if not ok:
      print(f"   局部尺寸子任务创建失败，跳过: {name} | {last_err}")
      return False

   child = find_task_by_exact_or_contains(workflow, name)
   if child is None:
      print(f"   未找到子任务 {name}，跳过该局部尺寸。")
      return False

   # 读取子任务 state。只允许修改 BOISize / BOIFaceSize / FaceSize，禁止 TargetMeshSize。
   state = get_task_state(child)
   if isinstance(state, dict):
      patched = dict(state)
      # 保留已有字段，同时强制写入 Fluent 2024R1 可识别的尺寸字段。
      patched["BOISize"] = float(size)
      if "BOIFaceSize" in patched:
         patched["BOIFaceSize"] = float(size)
      if "FaceSize" in patched:
         patched["FaceSize"] = float(size)

      # 关键修正：所有 Local Sizing 的 Growth Rate 统一跟随 SURF_GROWTH_RATE。
      # 之前 GUI 中仍可见 1.2，是因为 Add Local Sizing 子任务没有显式写入 Growth Rate。
      patched["GrowthRate"] = local_growth_rate
      patched["Growth Rate"] = local_growth_rate
      patched["BOIGrowthRate"] = local_growth_rate
      patched["SizeGrowthRate"] = local_growth_rate
      patched["SizingGrowthRate"] = local_growth_rate

      if "TargetMeshSize" in patched:
         # 这个字段会触发 Illegitimate input，必须删掉。
         patched.pop("TargetMeshSize", None)
      try:
         apply_task_state(child, patched, f"{name} child BOISize patch")
      except Exception:
         pass

   print(f"   {name} 已创建，目标面尺寸 = {size} mm，局部 Growth Rate = {local_growth_rate}。")
   print_task_state(child, f"{name} 当前")
   return True

def read_part_summary_metrics_for_refinement(path):
   """读取 PART_SUMMARY，供 v112 自动创建局部体加密缓冲区。"""
   rows = {}
   if not os.path.exists(path):
      return rows

   data_lines = []
   with open(path, "r", encoding="utf-8", errors="ignore", newline="") as f:
      for line in f:
         ss = line.strip()
         if not ss or ss.startswith("#"):
            continue
         data_lines.append(line)

   if not data_lines:
      return rows

   for row in csv.DictReader(data_lines, delimiter="\t"):
      if str(row.get("record_type", "")).strip() != "PART_SUMMARY":
         continue
      label = str(row.get("part_label", "")).strip()
      if not label:
         continue
      rows[label] = row
   return rows


def read_target_mesh_metadata(path):
   """读取 target 文件中的 target + role；role 来自几何判定，不依赖名称。"""
   meta = {}
   if not os.path.exists(path):
      return meta
   data_lines = []
   with open(path, "r", encoding="utf-8", errors="ignore", newline="") as f:
      for line in f:
         ss = line.strip()
         if not ss or ss.startswith("#"):
            continue
         data_lines.append(line)
   if not data_lines:
      return meta
   for row in csv.DictReader(data_lines, delimiter="\t"):
      label = str(row.get("part_label", "")).strip()
      if not label:
         continue
      size = _safe_float(row.get("target_mesh_size_mm"))
      if size is None or size <= 0:
         continue
      meta[label] = {
         "target": float(size),
         "role": str(row.get("part_role", "")).strip(),
      }
   return meta


def _bbox_axis_values(row, axis):
   if axis == "x":
      return _safe_float(row.get("xmin_mm")), _safe_float(row.get("xmax_mm"))
   if axis == "y":
      return _safe_float(row.get("ymin_mm")), _safe_float(row.get("ymax_mm"))
   return _safe_float(row.get("zmin_mm")), _safe_float(row.get("zmax_mm"))


def _refinement_bbox_object(row, target_size, pad_target_factor, min_ratio, forced_ratio=None):
   """
   Fluent Create Local Refinement Regions -> Bounding Box 控制。
   坐标取 SpaceClaim 实测 bbox；ratio 根据局部 target 和部件尺度自动计算。
   """
   out = {"SizeRelativeLength": "Ratio relative to geometry size"}
   for axis, cap in [("x", "X"), ("y", "Y"), ("z", "Z")]:
      vmin, vmax = _bbox_axis_values(row, axis)
      if vmin is None or vmax is None:
         vmin, vmax = 0.0, 0.0
      dim = max(abs(vmax - vmin), float(target_size), 1.0e-9)
      if forced_ratio is None:
         pad_mm = max(float(pad_target_factor) * float(target_size), float(min_ratio) * dim)
         ratio = pad_mm / dim
      else:
         ratio = float(forced_ratio)
      ratio = max(float(min_ratio), min(float(REFINEMENT_MAX_PAD_RATIO), ratio))
      out[cap + "min"] = float(vmin)
      out[cap + "max"] = float(vmax)
      out[cap + "minRatio"] = float(ratio)
      out[cap + "maxRatio"] = float(ratio)
   return out


def _zero_cylinder_object():
   return {
      "HeightBackInc": 0,
      "HeightFrontInc": 0,
      "HeightNode": "none",
      "Node1": "none",
      "Node2": "none",
      "Node3": "none",
      "Options": "3 Arc Nodes",
      "Radius1": 0,
      "Radius2": 0,
   }


def _default_offset_object():
   return {
      "AspectRatio": 5,
      "BoundaryLayerHeight": 4,
      "BoundaryLayerLevels": 1,
      "CrossWakeGrowthFactor": 1.1,
      "DefeaturingSize": max(float(SURF_MIN_SIZE), 1.0e-6),
      "EdgeSelectionList": [],
      "FirstHeight": 0.01,
      "FlipDirection": False,
      "FlowDirection": "X",
      "LastRatioPercentage": 20,
      "MptMethodType": "Automatic",
      "NumberOfLayers": 4,
      "OffsetMethodType": "uniform",
      "Rate": 1.2,
      "ShowCoordinates": True,
      "WakeGrowthFactor": 2,
      "WakeLevels": 1,
      "X": 0,
      "Y": 0,
      "Z": 0,
   }


def _get_create_local_refinement_task(workflow):
   """Fluent 2024 R1/v241：按旧版 workflow 的 Insert Next Task 路线插入任务。"""
   # 如果已经存在，直接复用。
   for task_name in [
      "Create Local Refinement Regions",
      "Create Local Refinement Region",
      "Local Refinement Regions",
      "Local Refinement Region",
   ]:
      try:
         return workflow.TaskObject[task_name]
      except Exception:
         pass

   add_local = workflow.TaskObject["Add Local Sizing"]
   attempts = [
      lambda: add_local.InsertNextTask(CommandName="CreateLocalRefinementRegions"),
      lambda: add_local.InsertNextTask(CommandName="Create Local Refinement Regions"),
      lambda: add_local.InsertNextTask(command_name="CreateLocalRefinementRegions"),
      lambda: add_local.InsertNextTask(command_name="Create Local Refinement Regions"),
   ]
   errors = []
   for fn in attempts:
      try:
         fn()
         time.sleep(1.0)
         break
      except Exception as e:
         errors.append(str(e))
   else:
      print("   ⚠ 无法插入 Create Local Refinement Regions: " + " | ".join(errors))
      return None

   for task_name in [
      "Create Local Refinement Regions",
      "Create Local Refinement Region",
      "Local Refinement Regions",
      "Local Refinement Region",
   ]:
      try:
         return workflow.TaskObject[task_name]
      except Exception:
         pass
   print("   ⚠ 已尝试插入，但无法取得 Create Local Refinement Regions 任务对象。")
   return None


def _execute_refinement_region(ref_task, name, label, size, bbox):
   """创建一个 bounding-box 型局部体加密区。失败返回 False，不中断整套自动化。"""
   creation_methods = ["Box", "Bounding Box"]
   last_err = None
   for method in creation_methods:
      state = {
         "RefinementRegionsName": str(name),
         "CreationMethod": method,
         "BOIMaxSize": float(size),
         "BOISizeName": str(name) + "_size",
         "SelectionType": "label",
         "ZoneSelectionList": [],
         "ZoneLocation": [],
         "LabelSelectionList": [str(label)],
         "ObjectSelectionList": [],
         "ZoneSelectionSingle": [],
         "ObjectSelectionSingle": [],
         "BoundingBoxObject": bbox,
         "OffsetObject": _default_offset_object(),
         "CylinderObject": _zero_cylinder_object(),
      }
      try:
         ref_task.Arguments.set_state(state)
         ref_task.Execute()
         print("   ✓ Local Refinement Region: {} | label={} | max={} mm".format(name, label, size))
         return True
      except Exception as e:
         last_err = e
   print("   ⚠ Local Refinement Region 创建失败: {} | {}".format(name, last_err))
   return False


def build_and_apply_adaptive_refinement_regions(workflow):
   """
   v112 保留的两级缓冲函数（主流程默认关闭，等待 v241 专项验证）：
   - 主体：1.5*h_main / 3.0*h_main；
   - 附体：2*h_surface / 5*h_surface，并由主体尺度封顶；
   - bbox 外扩完全依据当前部件几何与 target，不使用部件名称语义。
   """
   if not ENABLE_LOCAL_REFINEMENT_REGIONS:
      print("   Local Refinement Regions 已关闭。")
      return 0

   rows = read_part_summary_metrics_for_refinement(PART_GEOMETRY_METRICS_PATH)
   meta = read_target_mesh_metadata(TARGET_MESH_SIZE_PATH)
   if not rows or not meta:
      print("   ⚠ 缺少 metrics/target metadata，跳过 Local Refinement Regions。")
      return 0

   main_targets = [m["target"] for m in meta.values() if m.get("role") in ["dominant_body", "body_segment"]]
   main_h = max(main_targets) if main_targets else max(m["target"] for m in meta.values())

   ref_task = _get_create_local_refinement_task(workflow)
   if ref_task is None:
      return 0

   created = 0
   # 主体先做两级近体缓冲。
   for label, m in meta.items():
      role = m.get("role", "")
      if role not in ["dominant_body", "body_segment"]:
         continue
      row = rows.get(label)
      if row is None:
         continue
      h = float(m["target"])
      inner_size = nice_size(max(h, BODY_INNER_SIZE_FACTOR * h))
      outer_size = nice_size(max(inner_size, BODY_OUTER_SIZE_FACTOR * h))
      bbox_inner = _refinement_bbox_object(row, h, 0.0, BODY_INNER_PAD_RATIO, forced_ratio=BODY_INNER_PAD_RATIO)
      bbox_outer = _refinement_bbox_object(row, h, 0.0, BODY_OUTER_PAD_RATIO, forced_ratio=BODY_OUTER_PAD_RATIO)
      created += int(_execute_refinement_region(ref_task, "ref_{}_body_inner".format(label), label, inner_size, bbox_inner))
      created += int(_execute_refinement_region(ref_task, "ref_{}_body_outer".format(label), label, outer_size, bbox_outer))

   # 所有附体都按几何 target 建立两级缓冲；名字只是 label ID。
   for label, m in meta.items():
      role = m.get("role", "")
      if role in ["dominant_body", "body_segment"]:
         continue
      row = rows.get(label)
      if row is None:
         continue
      h = float(m["target"])
      inner_size = min(APPENDAGE_INNER_SIZE_FACTOR * h, APPENDAGE_INNER_MAX_MAIN_FACTOR * main_h)
      outer_size = min(APPENDAGE_OUTER_SIZE_FACTOR * h, APPENDAGE_OUTER_MAX_MAIN_FACTOR * main_h)
      inner_size = nice_size(max(h, inner_size))
      outer_size = nice_size(max(inner_size, outer_size))
      bbox_inner = _refinement_bbox_object(
         row, h, APPENDAGE_INNER_PAD_TARGET_FACTOR, APPENDAGE_INNER_MIN_RATIO
      )
      bbox_outer = _refinement_bbox_object(
         row, h, APPENDAGE_OUTER_PAD_TARGET_FACTOR, APPENDAGE_OUTER_MIN_RATIO
      )
      created += int(_execute_refinement_region(ref_task, "ref_{}_inner".format(label), label, inner_size, bbox_inner))
      created += int(_execute_refinement_region(ref_task, "ref_{}_outer".format(label), label, outer_size, bbox_outer))

   print("   Local Refinement Regions 创建完成，成功数量 = {}".format(created))
   return created


def set_surface_mesh_controls(workflow):
   """
   Fluent 2024 R1 / v241 verified surface-mesh controls.

   The user's actual v241 transcript reported these exact allowed enum values:
      SizeFunctions: Curvature, Proximity, Curvature & Proximity
      ScopeProximityTo: edges, faces, faces-and-edges

   Therefore do not use the newer/human-readable spellings
   "Curvature and Proximity" or "Faces and Edges".
   """
   task = workflow.TaskObject["Generate the Surface Mesh"]

   # First use the exact v241 enum strings observed in the Fluent error message.
   primary_state = {
      "CFDSurfaceMeshControls": {
         "MinSize": SURF_MIN_SIZE,
         "MaxSize": SURF_MAX_SIZE,
         "GrowthRate": SURF_GROWTH_RATE,
         "CurvatureNormalAngle": CURVATURE_ANGLE,
         "CellsPerGap": CELLS_PER_GAP,
         "SizeFunctions": "Curvature & Proximity",
         "ScopeProximityTo": "faces-and-edges",
      }
   }

   if not apply_task_state(task, primary_state, "Generate Surface Mesh v241 exact enums"):
      # Conservative fallback: keep sizing values but let Fluent retain its own
      # SizeFunctions / proximity scope defaults.
      fallback_state = {
         "CFDSurfaceMeshControls": {
            "MinSize": SURF_MIN_SIZE,
            "MaxSize": SURF_MAX_SIZE,
            "GrowthRate": SURF_GROWTH_RATE,
            "CurvatureNormalAngle": CURVATURE_ANGLE,
            "CellsPerGap": CELLS_PER_GAP,
         }
      }
      if not apply_task_state(task, fallback_state, "Generate Surface Mesh fallback"):
         raise RuntimeError("Generate the Surface Mesh 参数设置失败")

   print("   表面网格参数已设置（Fluent 2024 R1 / v241）：")
   print(f"     MinSize = {SURF_MIN_SIZE} mm")
   print(f"     MaxSize = {SURF_MAX_SIZE} mm")
   print(f"     GrowthRate = {SURF_GROWTH_RATE}")
   print(f"     CurvatureNormalAngle = {CURVATURE_ANGLE} deg")
   print(f"     CellsPerGap = {CELLS_PER_GAP}")
   print("     SizeFunctions = Curvature & Proximity")
   print("     ScopeProximityTo = faces-and-edges")

def _call_tui_path_if_exists(session, path_items, *args):
   try:
      obj = session
      for item in path_items:
         obj = getattr(obj, item)
      obj(*args)
      return True
   except Exception:
      return False


def _meshing_tui_load_string(session, text):
   try:
      escaped = str(text).replace('\\', '/').replace('"', '\\"')
      session.scheme_eval.eval(f'(ti-menu-load-string "{escaped}\\n")')
      return True
   except Exception:
      pass

   try:
      session.execute_tui(str(text))
      return True
   except Exception:
      return False


def _mesh_quality_smooth_value():
   val = str(MESH_QUALITY_SMOOTH_REMAINING).strip().lower()

   if val in ["yes", "true", "1", "on"]:
      return "yes"

   return "no"


def _mesh_quality_console_marker(session, message):
   """
   把关键标记直接写进 Fluent/Meshing console。
   这样你检查 fluent-*.trn 时，能确定代码是否真的运行到了网格质量改进步骤。
   """
   try:
      msg = str(message).replace("\\", "/").replace('"', "'")
      session.scheme_eval.eval(f'(format #t "\\n[V71-MESH-QUALITY] {msg}\\n")')
   except Exception:
      pass


def _read_recent_meshing_transcripts(start_time=None, max_files=6):
   """
   读取 WORK_DIR 下最近的 fluent-*.trn。
   这些文件是判断 GUI/TUI 操作是否真正执行的最可靠依据。
   """
   raw = ""

   try:
      candidates = []

      for fn in os.listdir(WORK_DIR):
         low = fn.lower()

         if low.startswith("fluent-") and low.endswith(".trn"):
            p = os.path.join(WORK_DIR, fn)

            try:
               mt = os.path.getmtime(p)
            except Exception:
               continue

            if start_time is None or mt >= start_time - 15.0:
               candidates.append((mt, p))

      candidates.sort(reverse=True)

      for mt, p in candidates[:max_files]:
         try:
            with open(p, "r", encoding="utf-8", errors="ignore") as f:
               txt = f.read()
            raw += f"\n\n===== mesh quality transcript: {p} =====\n"
            raw += txt[-2500000:]
         except Exception:
            pass

   except Exception:
      pass

   return raw


def _parse_min_orthogonal_quality_from_text(text):
   """
   从 Fluent Meshing 日志中提取最后一次出现的最小 Orthogonal Quality。
   支持：
      The mesh has a minimum Orthogonal Quality of:  0.20
      final minimum Orthogonal Quality is 0.300
      Minimum = 0.059614618
      minimum quality ... 0.20159749
   """
   if not text:
      return None

   patterns = [
      r"final\s+minimum\s+Orthogonal\s+Quality\s+is\s+([-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)",
      r"minimum\s+Orthogonal\s+Quality\s+of:\s*([-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)",
      r"Minimum\s*=\s*([-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)",
      r"minimum\s+quality\s+cell\s+count\s*\n[-\s]+\n\s*\S+\s+\d+\s+\d+\s+([-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)",
   ]

   values = []

   for pat in patterns:
      try:
         for m in re.finditer(pat, str(text), flags=re.I):
            try:
               values.append(float(m.group(1)))
            except Exception:
               pass
      except Exception:
         pass

   if values:
      return values[-1]

   return None


def _write_mesh_quality_status(status):
   """
   保存网格质量改进状态到工作目录，方便不看终端时检查。
   """
   try:
      path = os.path.join(WORK_DIR, "mesh_quality_improvement_status.out")

      with open(path, "w", encoding="utf-8") as f:
         for key, value in status.items():
            f.write(str(key) + " = " + str(value) + "\n")

      print(f"   网格质量改进状态已保存: {path}")
   except Exception as e:
      print(f"   网格质量改进状态文件保存失败: {e}")


def _evaluate_volume_quality_gui(meshing_session):
   """
   参考你手动日志中的命令，执行 Mesh -> Quality -> Evaluate Volume Quality。
   """
   cmds = [
      '(cx-gui-do cx-activate-item "Ribbon*Frame1*Frame2(Task Page)*Table1*Table3(Mesh)*PushButton2(  Quality)")',
      '(cx-gui-do cx-activate-item "MenuBar*PopupMenuQuality*Evaluate Volume Quality")',
   ]

   ok = False

   for cmd in cmds:
      try:
         meshing_session.scheme_eval.eval(cmd)
         ok = True
         time.sleep(0.5)
      except Exception as e:
         print(f"   Evaluate Volume Quality GUI 命令失败: {cmd} | {e}")

   time.sleep(1.0)

   return ok


def _manual_gui_sequence_volume_quality_improve(meshing_session, start_time):
   """
   参考你上传的手动 fluent-20260703-161500-6260.trn 中实际出现的 GUI 序列。

   该日志中，生成体网格后先执行 Evaluate Volume Quality，
   然后执行 Quality / Check / Diagnostics Tools / Close / Quality，
   后续出现：
      cells (quality < 0.3) = 0
      Volume Quality improvement ...
      final minimum Orthogonal Quality is 0.300
   """
   journal_path = os.path.join(WORK_DIR, "runtime_v71_improve_volume_quality_gui.jou").replace(chr(92), "/")

   gui_cmds = [
      '(cx-gui-do cx-activate-item "Ribbon*Frame1*Frame2(Task Page)*Table1*Table3(Mesh)*PushButton2(  Quality)")',
      '(cx-gui-do cx-activate-item "MenuBar*PopupMenuQuality*Evaluate Volume Quality")',
      '(cx-gui-do cx-activate-item "Ribbon*Frame1*Frame2(Task Page)*Table1*Table3(Mesh)*PushButton2(  Quality)")',
      '(cx-gui-do cx-activate-item "Ribbon*Frame1*Frame2(Task Page)*Table1*Table3(Mesh)*PushButton1(  Check)")',
      '(cx-gui-do cx-activate-item "Ribbon*Frame1*Frame2(Task Page)*Table1*Table3(Mesh)*PushButton2(  Quality)")',
      '(cx-gui-do cx-activate-item "Ribbon*Frame1*Frame2(Task Page)*Table1*Table3(Mesh)*PushButton2(  Quality)")',
      '(cx-gui-do cx-activate-item "MenuBar*PopupMenuQuality*Diagnostics Tools...")',
      '(cx-gui-do cx-activate-item "Diagnostics Tools*PanelButtons*PushButton1(Close)")',
      '(cx-gui-do cx-activate-item "ToolBar*View*autoscale")',
      '(cx-gui-do cx-activate-item "Ribbon*Frame1*Frame2(Task Page)*Table1*Table3(Mesh)*PushButton2(  Quality)")',
   ]

   try:
      with open(journal_path, "w", encoding="utf-8") as f:
         for cmd in gui_cmds:
            f.write(cmd + "\n")

      meshing_session.tui.file.read_journal(journal_path)
      time.sleep(3.0)
      print(f"   已按手动日志序列执行体网格质量改进 GUI journal: {journal_path}")
      return True

   except Exception as e:
      print(f"   手动日志序列 GUI journal 执行失败: {e}")

   # 如果 read_journal 失败，再逐条 scheme_eval，失败不中断。
   ok = False
   for cmd in gui_cmds:
      try:
         meshing_session.scheme_eval.eval(cmd)
         ok = True
         time.sleep(0.5)
      except Exception:
         pass

   if ok:
      print("   已逐条尝试执行手动日志序列体网格质量改进 GUI 命令。")
      time.sleep(2.0)

   return ok


def _try_execute_volume_quality_task(workflow, method, quality_limit, min_angle, iterations, smooth):
   """
   Workflow Task 优先路线。若当前版本暴露了 Improve Mesh Quality 任务，就直接执行。
   """
   task_names = [
      "Improve Mesh Quality",
      "Improve the Mesh Quality",
      "Improve Volume Mesh Quality",
      "Improve the Volume Mesh Quality",
      "Improve Volume Quality",
      "Improve the Volume Quality",
      "Improve Volume Mesh",
      "Improve the Volume Mesh",
      "Improve Mesh",
   ]

   state_variants = [
      {
         "QualityMethod": method,
         "QualityLimit": quality_limit,
         "MinimumAngle": min_angle,
         "Iterations": iterations,
         "SmoothRemaining": smooth,
      },
      {
         "Quality Method": method,
         "Cell Quality Limit": quality_limit,
         "Quality Improve Min Angle": min_angle,
         "Quality Improve Iterations": iterations,
         "Smooth Remaining Defective Cells": smooth,
      },
      {
         "QualityMeasure": method,
         "CellQualityLimit": quality_limit,
         "QualityImproveMinimumAngle": min_angle,
         "QualityImproveIterations": iterations,
         "SmoothRemainingDefectiveCells": smooth,
      },
   ]

   # 在日志/终端打印 workflow task 名称，避免“有没有这个任务”不清楚。
   try:
      task_keys = [str(k) for k in workflow.TaskObject.keys()]
      print("   当前 workflow task 列表 = " + str(task_keys))
   except Exception:
      pass

   for task_name in task_names:
      try:
         task = workflow.TaskObject[task_name]
      except Exception:
         continue

      print(f"   找到体网格质量改进 workflow task: {task_name}")

      for st in state_variants:
         try:
            apply_task_state(task, st, task_name)
         except Exception:
            pass

      try:
         print_task_state(task, task_name + " 当前 state")
      except Exception:
         pass

      try:
         task.Execute()
         print(f"   体网格质量改进已通过 workflow task 执行: {task_name}")
         return True
      except Exception as e:
         print(f"   workflow task 执行失败: {task_name} | {e}")

   return False


def _try_tui_volume_quality_improve(meshing_session, method, quality_limit, min_angle, iterations, smooth):
   """
   TUI 兜底。不同 Fluent 版本交互提示不同，因此保留多种可能命令。
   """
   command_sequences = [
      f"/mesh/quality/improve\n{method}\n{quality_limit}\n{min_angle}\n{iterations}\n{smooth}",
      f"/mesh/quality/improve-quality\n{method}\n{quality_limit}\n{min_angle}\n{iterations}\n{smooth}",
      f"/mesh/quality/improve-mesh-quality\n{method}\n{quality_limit}\n{min_angle}\n{iterations}\n{smooth}",
      f"/mesh/improve-mesh-quality\n{method}\n{quality_limit}\n{min_angle}\n{iterations}\n{smooth}",
      f"/mesh/repair-improve/improve-quality\n{method}\n{quality_limit}\n{min_angle}\n{iterations}\n{smooth}",
   ]

   for cmd in command_sequences:
      if _meshing_tui_load_string(meshing_session, cmd):
         print("   已尝试通过 TUI 执行体网格质量改进。")
         time.sleep(2.0)
         return True

   return False


def _mesh_quality_improvement_proof(text, quality_limit):
   """
   判断日志里是否真的出现了体网格质量改进输出。
   只有出现 Volume Quality improvement / final minimum Orthogonal Quality 等关键词，
   才认为改进命令真正执行过。
   """
   if not text:
      return False

   low = str(text).lower()

   if "volume quality improvement" in low:
      return True

   if "final minimum orthogonal quality" in low:
      return True

   # 同时兼容 0.2 / 0.20 / 0.3 / 0.30 等显示方式。
   q1 = str(float(quality_limit)).rstrip("0").rstrip(".")
   q2 = f"{float(quality_limit):.1f}"
   q3 = f"{float(quality_limit):.2f}"

   if f"cells (quality < {q1})" in low:
      return True
   if f"cells (quality < {q2})" in low:
      return True
   if f"cells (quality < {q3})" in low:
      return True

   return False


def improve_volume_mesh_quality_after_generation(meshing_session, workflow):
   """
   V73 校验版体网格质量改进。

   当前验证版仍保留：
      MESH_QUALITY_LIMIT = 0.30

   但修正两点：
      1. 不再把“已尝试通过 TUI”当作真正成功；
      2. 优先执行参考手动日志的 GUI 序列，并用日志关键词确认是否真的出现
         Volume Quality improvement / final minimum Orthogonal Quality。

   注意：
      0.30 是验证用阈值，不保证所有几何都能被 Fluent 改到 0.30。
      如果真的执行了改进但最终仍低于 0.30，本版会记录状态并继续流程，
      不再直接中断后续 Fluent 求解和后处理。
   """
   if not ENABLE_IMPROVE_MESH_QUALITY:
      print("   跳过体网格质量改进：ENABLE_IMPROVE_MESH_QUALITY=False")
      return False

   method = str(MESH_QUALITY_METHOD)
   quality_limit = float(MESH_QUALITY_LIMIT)
   min_angle = float(MESH_QUALITY_MIN_ANGLE)
   iterations = int(MESH_QUALITY_IMPROVE_ITERATIONS)
   smooth = _mesh_quality_smooth_value()
   start_time = time.time()

   status = {
      "enabled": True,
      "method": method,
      "quality_limit": quality_limit,
      "min_angle": min_angle,
      "iterations": iterations,
      "smooth_remaining": smooth,
      "executed_improvement_command": False,
      "already_satisfied_before_improve": False,
      "verified_final_quality": False,
      "minimum_quality_before": None,
      "minimum_quality_after": None,
      "proof_in_transcript": False,
      "workflow_attempted": False,
      "gui_sequence_attempted": False,
      "tui_attempted": False,
      "continue_even_if_target_not_reached": True,
   }

   print("   ▶ V73 执行并校验体网格质量改进")
   print(f"     质量方法 = {method}")
   print(f"     单元质量限制 = {quality_limit}")
   print(f"     质量改进最小角度[度] = {min_angle}")
   print(f"     质量优化迭代次数 = {iterations}")
   print(f"     允许对剩余缺陷单元特征进行平滑处理 = {smooth}")

   _mesh_quality_console_marker(
      meshing_session,
      f"START method={method}, quality_limit={quality_limit}, min_angle={min_angle}, iterations={iterations}, smooth={smooth}"
   )

   # 1. 先评估当前质量。
   try:
      _evaluate_volume_quality_gui(meshing_session)
   except Exception as e:
      print(f"   初始 Evaluate Volume Quality 异常: {e}")

   time.sleep(1.5)
   before_text = _read_recent_meshing_transcripts(start_time=start_time)
   before_min = _parse_min_orthogonal_quality_from_text(before_text)
   status["minimum_quality_before"] = before_min

   if before_min is not None:
      print(f"   初始 minimum Orthogonal Quality = {before_min}")

   if before_min is not None and before_min >= quality_limit:
      status["already_satisfied_before_improve"] = True
      status["minimum_quality_after"] = before_min
      status["verified_final_quality"] = True
      _mesh_quality_console_marker(
         meshing_session,
         f"ALREADY SATISFIED: minimum Orthogonal Quality={before_min} >= {quality_limit}; no cells require improvement under this limit"
      )
      _write_mesh_quality_status(status)
      return True

   # 2. 未达标时，优先走 workflow task。
   proof = False

   try:
      status["workflow_attempted"] = True
      workflow_ok = _try_execute_volume_quality_task(
         workflow=workflow,
         method=method,
         quality_limit=quality_limit,
         min_angle=min_angle,
         iterations=iterations,
         smooth=smooth,
      )
      time.sleep(2.0)
      mid_text = _read_recent_meshing_transcripts(start_time=start_time)
      proof = _mesh_quality_improvement_proof(mid_text, quality_limit)
      if workflow_ok and proof:
         status["executed_improvement_command"] = True
         print("   workflow 路线已在日志中确认 Volume Quality improvement。")
   except Exception as e:
      print(f"   workflow 体网格质量改进异常: {e}")

   # 3. 如果 workflow 没有日志证明，执行参考手动日志的 GUI 序列。
   if not proof:
      try:
         status["gui_sequence_attempted"] = True
         gui_ok = _manual_gui_sequence_volume_quality_improve(
            meshing_session=meshing_session,
            start_time=start_time,
         )
         time.sleep(2.5)
         mid_text = _read_recent_meshing_transcripts(start_time=start_time)
         proof = _mesh_quality_improvement_proof(mid_text, quality_limit)
         if gui_ok and proof:
            status["executed_improvement_command"] = True
            print("   GUI 手动日志序列已在日志中确认 Volume Quality improvement。")
      except Exception as e:
         print(f"   手动日志 GUI 序列体网格质量改进异常: {e}")

   # 4. TUI 只作为最后兜底；并且必须有日志证明才算执行成功。
   if not proof:
      try:
         status["tui_attempted"] = True
         tui_ok = _try_tui_volume_quality_improve(
            meshing_session=meshing_session,
            method=method,
            quality_limit=quality_limit,
            min_angle=min_angle,
            iterations=iterations,
            smooth=smooth,
         )
         time.sleep(2.0)
         mid_text = _read_recent_meshing_transcripts(start_time=start_time)
         proof = _mesh_quality_improvement_proof(mid_text, quality_limit)
         if tui_ok and proof:
            status["executed_improvement_command"] = True
            print("   TUI 路线已在日志中确认 Volume Quality improvement。")
         elif tui_ok:
            print("   TUI 命令仅被尝试，但日志中没有 Volume Quality improvement；不把它当作成功。")
      except Exception as e:
         print(f"   TUI 体网格质量改进异常: {e}")

   status["proof_in_transcript"] = bool(proof)

   # 5. 再评估最终质量。
   try:
      _evaluate_volume_quality_gui(meshing_session)
   except Exception as e:
      print(f"   最终 Evaluate Volume Quality 异常: {e}")

   time.sleep(2.0)
   after_text = _read_recent_meshing_transcripts(start_time=start_time)
   after_min = _parse_min_orthogonal_quality_from_text(after_text)
   status["minimum_quality_after"] = after_min

   if after_min is not None:
      print(f"   最终 minimum Orthogonal Quality = {after_min}")

   if after_min is not None and after_min >= quality_limit:
      status["verified_final_quality"] = True
      _mesh_quality_console_marker(
         meshing_session,
         f"PASS final minimum Orthogonal Quality={after_min} >= {quality_limit}, proof_in_transcript={proof}"
      )
      _write_mesh_quality_status(status)
      return True

   # 6. 如果 0.30 没达到，不中断主流程，但明确写状态。
   #    这是为了验证脚本是否启动，而不是把验证阈值当作正式生产阈值。
   if proof:
      _mesh_quality_console_marker(
         meshing_session,
         f"EXECUTED BUT TARGET NOT REACHED: final minimum Orthogonal Quality={after_min}, required={quality_limit}"
      )
      print("   体网格质量改进已执行，但最终质量未达到当前验证阈值。脚本继续运行。")
   else:
      _mesh_quality_console_marker(
         meshing_session,
         f"NO PROOF OF VOLUME QUALITY IMPROVEMENT: final minimum Orthogonal Quality={after_min}, required={quality_limit}"
      )
      print("   没有在日志中确认 Volume Quality improvement。脚本继续运行，但状态文件会记录失败。")

   _write_mesh_quality_status(status)
   return False


def improve_surface_mesh_if_available(workflow):
   """
   v112：Generate Surface Mesh 后主动插入/执行 Improve Surface Mesh，
   而不是等体网格生成后再补救。
   """
   if not ENABLE_IMPROVE_SURFACE_MESH:
      print("   Improve Surface Mesh 已关闭。")
      return False

   task = find_task_by_exact_or_contains(workflow, "Improve Surface Mesh")
   if task is None:
      try:
         workflow.TaskObject["Generate the Surface Mesh"].InsertNextTask(CommandName="ImproveSurfaceMesh")
         time.sleep(1.0)
      except Exception:
         try:
            workflow.TaskObject["Generate the Surface Mesh"].InsertNextTask(CommandName="Improve Surface Mesh")
            time.sleep(1.0)
         except Exception as e:
            print("   ⚠ 无法插入 Improve Surface Mesh: {}".format(e))

      task = find_task_by_exact_or_contains(workflow, "Improve Surface Mesh")

   if task is None:
      print("   ⚠ 当前 workflow 中仍没有 Improve Surface Mesh，跳过预改善。")
      return False

   states = [
      {
         "FaceQualityLimit": float(SURFACE_FACE_QUALITY_LIMIT),
         "ImproveSurfaceMeshPreferences": {
            "SIQualityIterations": int(SURFACE_IMPROVE_ITERATIONS),
         },
      },
      {
         "FaceQualityLimit": float(SURFACE_FACE_QUALITY_LIMIT),
         "MaxIterations": int(SURFACE_IMPROVE_ITERATIONS),
      },
      {
         "FaceQualityLimit": float(SURFACE_FACE_QUALITY_LIMIT),
      },
      {},
   ]

   last_err = None
   for st in states:
      try:
         if st:
            apply_task_state(task, st, "Improve Surface Mesh")
         task.Execute()
         print("   ✓ Improve Surface Mesh 已执行：FaceQualityLimit={}，iterations≈{}".format(
            SURFACE_FACE_QUALITY_LIMIT, SURFACE_IMPROVE_ITERATIONS
         ))
         return True
      except Exception as e:
         last_err = e

   print("   ⚠ Improve Surface Mesh 执行失败，继续后续流程: {}".format(last_err))
   return False


def describe_geometry_with_voids(workflow):
   """
   本几何是 SpaceClaim 已扣除后的单一 fluid 体，但 Fluent Surface Mesh
   会检测到内部 AUV solid void/dead regions。日志显示：
      After Surface mesh, the model consists of 1 fluid/solid regions and 2 voids.
   因此这里不能继续使用 only fluid regions with no voids。
   应按 Watertight 的 void 工作流描述，并把 fluid-fluid wall 转 internal，
   后续在 Update Regions 中显式指定 fluid/dead。
   """
   task = workflow.TaskObject["Describe Geometry"]

   state = {
      "SetupType": "The geometry consists of both fluid and solid regions and/or voids",
      "WallToInternal": "Yes",
      "InvokeShareTopology": "No",
      "Multizone": "No",
   }

   if not apply_task_state(task, state, "Describe Geometry"):
      raise RuntimeError("Describe Geometry 参数设置失败")

   task.UpdateChildTasks(SetupTypeChanged=True)
   task.Execute()

   print("   Describe Geometry 已设置为：both fluid/solid/voids + WallToInternal=Yes + ShareTopology=No + Multizone=No")

def update_regions_fluid_dead(workflow):
   task = workflow.TaskObject["Update Regions"]

   # 先执行一次，让 Fluent 生成当前 region 表。
   task.Execute()

   region_names = discover_region_names(task)
   region_types = []

   for name in region_names:
      if str(name).lower() == PREFERRED_FLUID_REGION_NAME.lower():
         region_types.append("fluid")
      else:
         region_types.append("dead")

   print("   ▶ 区域类型目标设置：")
   for n, t in zip(region_names, region_types):
      print(f"     {n} -> {t}")

   old_types = ["fluid"] * len(region_names)

   candidate_states = [
      {
         "OldRegionNameList": region_names,
         "OldRegionTypeList": old_types,
         "RegionNameList": region_names,
         "RegionTypeList": region_types,
      },
      {
         "RegionNameList": region_names,
         "RegionTypeList": region_types,
      },
      {
         "RegionNames": region_names,
         "RegionTypes": region_types,
      },
   ]

   applied = False
   for st in candidate_states:
      if apply_task_state(task, st, "Update Regions fluid/dead"):
         applied = True
         break

   if not applied:
      print("   Update Regions state 写入失败，将使用 Fluent 当前默认区域类型。")
      print_task_state(task, "Update Regions 当前")
   else:
      task.Execute()
      print("   Update Regions 已执行 fluid/dead 显式设置。")
      print_task_state(task, "Update Regions 设置后")


def add_boundary_layers(workflow):
   if not ENABLE_BOUNDARY_LAYER:
      print("   跳过边界层：ENABLE_BOUNDARY_LAYER=False")
      return

   labels = boundary_layer_labels_from_settings()
   print(f"   ▶ 边界层目标面：{labels}")

   parent = workflow.TaskObject["Add Boundary Layers"]

   # 参照原始代码：只给已经验证的边界层基础参数，不在父任务中塞 BoundaryLabelList。
   base_states = [
      {
         "AddChild": "yes",
         "NumberOfLayers": BL_LAYERS,
         "TransitionRatio": BL_TRANS_RATIO,
         "Rate": BL_GROWTH_RATE,
      },
      {
         "AddChild": "yes",
         "NumberOfLayers": BL_LAYERS,
         "TransitionRatio": BL_TRANS_RATIO,
         "GrowthRate": BL_GROWTH_RATE,
      },
   ]

   ok = False
   for st in base_states:
      try:
         if apply_task_state(parent, st, "Add Boundary Layers parent"):
            parent.AddChildAndUpdate()
            ok = True
            break
      except Exception:
         pass

   if not ok:
      raise RuntimeError("Add Boundary Layers 子任务创建失败")

   child = find_task_by_exact_or_contains(workflow, "smooth-transition")
   if child is not None:
      state = get_task_state(child)
      if state is not None:
         patched_state, label_changed = patch_label_list_in_state_object(state, labels)
         # 只在 state 里存在边界标签字段时才写回，避免重复产生 /AddBoundaryLayers illegitimate input。
         if label_changed:
            apply_task_state(child, patched_state, "Boundary Layer child labels")
            try_update_task(child, ["UpdateChildTasks", "Execute"])
            print("   Boundary Layer 子任务已尝试限定到 AUV 表面。")
         else:
            print("   当前 Fluent 子任务 state 中没有可识别的 boundary label 字段。")
            print("     该版本会按 Fluent 默认方式对 fluid region 的 wall 边界加 prism。")
      else:
         print("   无法读取 Boundary Layer 子任务 state。")

   print(f"   边界层层数 = {BL_LAYERS}, TransitionRatio = {BL_TRANS_RATIO}, GrowthRate = {BL_GROWTH_RATE}")


def patch_volume_state_from_existing(state):
   """优先按当前 Fluent 暴露的真实字段名修改体网格参数。"""
   if not isinstance(state, dict):
      return state, False

   changed = False

   def patch(obj):
      nonlocal changed
      if isinstance(obj, dict):
         out = {}
         for k, v in obj.items():
            kl = str(k).lower().replace(" ", "").replace("_", "").replace("-", "")
            nv = v
            if kl in ["volumefill", "fillvolumemesh", "volumefilltype"]:
               nv = VOL_FILL_TYPE; changed = True
            elif kl in ["solver", "volumemeshsolver"]:
               nv = VOL_SOLVER; changed = True
            elif "buffer" in kl and "layer" in kl:
               nv = VOL_BUFFER_LAYERS; changed = True
            elif "peel" in kl and "layer" in kl:
               nv = VOL_PEEL_LAYERS; changed = True
            elif "qualitymethod" in kl or ("quality" in kl and "method" in kl):
               nv = VOL_QUALITY_METHOD; changed = True
            elif "qualityimprove" in kl and ("limit" in kl or "threshold" in kl):
               nv = VOL_QUALITY_IMPROVE_LIMIT; changed = True
            elif "usesizefield" in kl or "usesizingfield" in kl or "usesizefunction" in kl:
               nv = VOL_USE_SIZE_FIELD; changed = True
            elif "poly" in kl and "skew" in kl and "angle" in kl:
               nv = VOL_POLY_MAX_CELL_SKEW_ANGLE; changed = True
            elif "avoid" in kl and ("18" in kl or "oneeight" in kl or "transition" in kl):
               nv = VOL_AVOID_ONE_EIGHT_TRANSITION; changed = True
            elif "self" in kl and "prox" in kl:
               nv = VOL_CHECK_SELF_PROXIMITY; changed = True
            elif kl in ["minsize", "volminsize", "minimumsize"]:
               nv = VOL_MIN_SIZE; changed = True
            elif kl in ["maxsize", "volmaxsize", "maximumsize", "hexmaxcelllength"]:
               nv = VOL_MAX_SIZE; changed = True
            out[k] = patch(nv)
         return out
      if isinstance(obj, list):
         return [patch(x) for x in obj]
      if isinstance(obj, tuple):
         return tuple(patch(x) for x in obj)
      return obj
   return patch(state), changed


def _patch_tetra_state(obj, max_len, growth_rate):
   """
   递归修补 tetrahedral 体网格字段。
   目标：
      Max Cell Length [mm] = SURF_MAX_SIZE
      Growth Rate = SURF_GROWTH_RATE
   """
   if isinstance(obj, dict):
      out = {}
      for key, value in obj.items():
         key_text = str(key).lower().replace(" ", "").replace("_", "").replace("-", "")

         if key_text in [
            "maxcelllength",
            "maximumcelllength",
            "maxsize",
            "tetmaxcelllength",
            "hexmaxcelllength",
            "maxelementsize",
         ]:
            out[key] = float(max_len)
         elif ("max" in key_text) and ("cell" in key_text or "length" in key_text or "size" in key_text):
            out[key] = float(max_len)
         elif key_text in [
            "growthrate",
            "growth",
            "tetgrowthrate",
            "volumegrowthrate",
         ]:
            out[key] = float(growth_rate)
         elif ("growth" in key_text) and ("rate" in key_text):
            out[key] = float(growth_rate)
         else:
            out[key] = _patch_tetra_state(value, max_len, growth_rate)

      return out

   if isinstance(obj, list):
      return [_patch_tetra_state(x, max_len, growth_rate) for x in obj]

   return obj


def _try_set_argument_attr(obj, attr_name, value):
   try:
      setattr(obj, attr_name, value)
      return True
   except Exception:
      return False


def _try_set_argument_item(obj, key_name, value):
   try:
      obj[key_name] = value
      return True
   except Exception:
      return False


def _volume_max_key_variants():
   return [
      "MaxCellLength",
      "Max Cell Length",
      "Max Cell Length [mm]",
      "MaximumCellLength",
      "Maximum Cell Length",
      "Maximum Cell Length [mm]",
      "MaxCellSize",
      "Max Cell Size",
      "MaximumCellSize",
      "Maximum Cell Size",
      "MaxSize",
      "MaximumSize",
      "Maximum Size",
      "MaxElementSize",
      "MaximumElementSize",
      "MaxLength",
      "Max Length",
      "TetMaxCellLength",
      "Tet Max Cell Length",
      "TetMaxCellSize",
      "Tet Max Cell Size",
      "MaxTetCellLength",
      "Max Tet Cell Length",
      "MaxTetCellSize",
      "Max Tet Cell Size",
      "TetraMaxCellLength",
      "Tetra Max Cell Length",
      "TetrahedralMaxCellLength",
      "Tetrahedral Max Cell Length",
   ]


def _volume_growth_key_variants():
   return [
      "GrowthRate",
      "Growth Rate",
      "TetraGrowthRate",
      "Tetra Growth Rate",
      "TetGrowthRate",
      "Tet Growth Rate",
      "VolumeGrowthRate",
      "Volume Growth Rate",
   ]


def build_tetra_candidate_states(max_len, growth_rate):
   """
   生成多组候选字段名。
   重点解决 Fluent Meshing tetrahedral 界面里的：
      Max Cell Length [mm]
   不跟随 SURF_MAX_SIZE 的问题。
   """
   max_len = float(max_len)
   growth_rate = float(growth_rate)

   max_keys = _volume_max_key_variants()
   growth_keys = _volume_growth_key_variants()

   candidates = []

   # 最小字段组合，避免因为未知字段过多而整体失败。
   for mk in max_keys:
      candidates.append({
         "VolumeFill": "tetrahedral",
         mk: max_len,
      })
      candidates.append({
         "VolumeFill": "tetrahedral",
         mk: max_len,
         "GrowthRate": growth_rate,
      })

   for gk in growth_keys:
      candidates.append({
         "VolumeFill": "tetrahedral",
         gk: growth_rate,
         "MaxCellLength": max_len,
      })

   # 常见嵌套字段。
   nested_names = [
      "VolumeFillControls",
      "TetrahedralControls",
      "TetraControls",
      "TetControls",
      "VolumeMeshControls",
      "Controls",
   ]

   for nested in nested_names:
      for mk in max_keys:
         candidates.append({
            "VolumeFill": "tetrahedral",
            nested: {
               mk: max_len,
               "GrowthRate": growth_rate,
            },
         })

   # 完整组合放在后面。
   full_nested = {}
   for mk in max_keys:
      full_nested[mk] = max_len
   for gk in growth_keys:
      full_nested[gk] = growth_rate

   candidates.append({
      "VolumeFill": "tetrahedral",
      "Solver": VOL_SOLVER,
      "GrowthRate": growth_rate,
      "MaxCellLength": max_len,
      "MaxSize": max_len,
      "VolumeFillControls": full_nested,
   })

   candidates.append({
      "FillWith": "tetrahedral",
      "Growth Rate": growth_rate,
      "Max Cell Length [mm]": max_len,
   })

   candidates.append({
      "Fill With": "tetrahedral",
      "Growth Rate": growth_rate,
      "Max Cell Length [mm]": max_len,
   })

   return candidates


def apply_generate_volume_mesh_tetra_settings(task, max_len, growth_rate):
   """
   强制设置 tetrahedral 体网格：

      Growth Rate = SURF_GROWTH_RATE
      Max Cell Length [mm] = SURF_MAX_SIZE

   v54 中 Growth Rate 能改成 1.1，但 Max Cell Length 仍保持 Fluent 默认值。
   本版增加更多字段名、嵌套字段、属性赋值和 item 赋值方式。
   """
   max_len = float(max_len)
   growth_rate = float(growth_rate)

   applied_any = False

   # 1. 先设置 Fill With = tetrahedral。
   fill_states = [
      {"VolumeFill": "tetrahedral"},
      {"FillWith": "tetrahedral"},
      {"Fill With": "tetrahedral"},
   ]

   for st in fill_states:
      if apply_task_state(task, st, "Generate Volume Mesh tetrahedral fill"):
         applied_any = True

   # 不在这里 UpdateChildTasks，否则有些版本会把尺寸重新恢复默认值。

   # 2. 用当前真实 state 递归修补已有 max/growth 字段。
   try:
      current_state = get_task_state(task)
      if isinstance(current_state, dict):
         patched = _patch_tetra_state(current_state, max_len, growth_rate)
         patched["VolumeFill"] = "tetrahedral"

         # 若当前 state 没有 MaxCellLength，也补上几个最常见字段。
         patched["MaxCellLength"] = max_len
         patched["MaxSize"] = max_len
         patched["GrowthRate"] = growth_rate

         if "VolumeFillControls" not in patched or not isinstance(patched.get("VolumeFillControls"), dict):
            patched["VolumeFillControls"] = {}

         patched["VolumeFillControls"]["MaxCellLength"] = max_len
         patched["VolumeFillControls"]["Max Cell Length"] = max_len
         patched["VolumeFillControls"]["Max Cell Length [mm]"] = max_len
         patched["VolumeFillControls"]["GrowthRate"] = growth_rate
         patched["VolumeFillControls"]["Growth Rate"] = growth_rate

         if apply_task_state(task, patched, "Generate Volume Mesh tetra current-state patch"):
            applied_any = True
   except Exception as e:
      print(f"   当前 state 递归修补失败: {e}")

   # 3. 多候选字段名强制写入。
   for st in build_tetra_candidate_states(max_len, growth_rate):
      if apply_task_state(task, st, "Generate Volume Mesh tetra max-cell-length candidates"):
         applied_any = True

   # 4. 直接属性赋值。PyFluent 某些版本这里能生效。
   attr_names = [
      "MaxCellLength",
      "MaximumCellLength",
      "MaxCellSize",
      "MaximumCellSize",
      "MaxSize",
      "MaximumSize",
      "MaxElementSize",
      "MaxLength",
      "TetMaxCellLength",
      "TetMaxCellSize",
      "MaxTetCellLength",
      "MaxTetCellSize",
      "TetraMaxCellLength",
      "TetrahedralMaxCellLength",
   ]

   growth_attr_names = [
      "GrowthRate",
      "TetraGrowthRate",
      "TetGrowthRate",
      "VolumeGrowthRate",
   ]

   for attr in attr_names:
      if _try_set_argument_attr(task.Arguments, attr, max_len):
         print(f"   已尝试通过属性设置体网格最大尺寸: {attr} = {max_len}")
         applied_any = True

   for attr in growth_attr_names:
      if _try_set_argument_attr(task.Arguments, attr, growth_rate):
         print(f"   已尝试通过属性设置体网格增长率: {attr} = {growth_rate}")
         applied_any = True

   # 5. 如果 Arguments 支持字典式赋值，也写一遍带空格的 GUI 字段名。
   for key in _volume_max_key_variants():
      if _try_set_argument_item(task.Arguments, key, max_len):
         print(f"   已尝试通过 item 设置体网格最大尺寸: {key} = {max_len}")
         applied_any = True

   for key in _volume_growth_key_variants():
      if _try_set_argument_item(task.Arguments, key, growth_rate):
         print(f"   已尝试通过 item 设置体网格增长率: {key} = {growth_rate}")
         applied_any = True

   # 6. 最后再读一次 state 并打印，方便检查 Max Cell Length 是否真的进入 state。
   try:
      state_after = get_task_state(task)
      state_text = str(state_after)

      print("   tetrahedral 体网格最大尺寸目标值 = " + str(max_len))
      print("   tetrahedral 体网格增长率目标值 = " + str(growth_rate))

      if str(max_len) not in state_text:
         print("   警告：当前 task state 文本中没有发现目标 Max Cell Length 数值。")
         print("   这说明当前 Fluent 版本可能没有通过 PyFluent 暴露该 GUI 字段。")
         print("   但脚本已尝试所有常见字段名；请以 GUI 中 Max Cell Length [mm] 为最终判断。")
   except Exception:
      pass

   return applied_any


def set_volume_mesh_controls(workflow):
   task = workflow.TaskObject["Generate the Volume Mesh"]

   fill_type = str(VOL_FILL_TYPE).lower().strip()

   if fill_type in ["tetrahedral", "tetrahedra", "tet"]:
      max_len = float(VOL_MAX_SIZE)
      growth_rate = float(SURF_GROWTH_RATE)

      applied = apply_generate_volume_mesh_tetra_settings(task, max_len, growth_rate)

      if not applied:
         raise RuntimeError("Generate the Volume Mesh tetrahedral 参数设置失败")

      print("   体网格参数已设置为 tetrahedral：")
      print(f"     Solver = {VOL_SOLVER}")
      print("     Fill With = tetrahedral")
      print(f"     Growth Rate = {SURF_GROWTH_RATE}")
      print(f"     Max Cell Length [mm] = {SURF_MAX_SIZE}")
      print("     BoundaryLayer = disabled")
      print("     注意：体网格最大尺寸跟随 SURF_MAX_SIZE，体网格增长率跟随 SURF_GROWTH_RATE")
      print_task_state(task, "Generate Volume Mesh 当前")
      return

   # 非 tetrahedral 时保留原 poly/hexcore 参数逻辑，但所有增长率统一为 SURF_GROWTH_RATE。
   current_state = get_task_state(task)
   patched_state, changed = patch_volume_state_from_existing(current_state)
   candidate_states = []
   if changed:
      candidate_states.append(patched_state)
   candidate_states.append({
      "VolumeFill": VOL_FILL_TYPE,
      "Solver": VOL_SOLVER,
      "BufferLayers": VOL_BUFFER_LAYERS,
      "PeelLayers": VOL_PEEL_LAYERS,
      "QualityMethod": VOL_QUALITY_METHOD,
      "QualityImproveLimit": VOL_QUALITY_IMPROVE_LIMIT,
      "UseSizeField": VOL_USE_SIZE_FIELD,
      "PolyMaxCellSkewAngle": VOL_POLY_MAX_CELL_SKEW_ANGLE,
      "Avoid1/8Transition": VOL_AVOID_ONE_EIGHT_TRANSITION,
      "CheckSelfProximity": VOL_CHECK_SELF_PROXIMITY,
      "MinSize": VOL_MIN_SIZE,
      "MaxSize": SURF_MAX_SIZE,
      "GrowthRate": SURF_GROWTH_RATE,
      "VolumeFillControls": {
         "HexMaxCellLength": SURF_MAX_SIZE,
         "BufferLayers": VOL_BUFFER_LAYERS,
         "PeelLayers": VOL_PEEL_LAYERS,
         "GrowthRate": SURF_GROWTH_RATE,
      },
   })
   candidate_states.append({
      "VolumeFill": VOL_FILL_TYPE,
      "VolumeMeshSolver": VOL_SOLVER,
      "BufferLayers": VOL_BUFFER_LAYERS,
      "PeelLayers": VOL_PEEL_LAYERS,
      "QualityMethod": VOL_QUALITY_METHOD,
      "QualityImproveLimit": VOL_QUALITY_IMPROVE_LIMIT,
      "UseSizeField": VOL_USE_SIZE_FIELD,
      "PolyMaxCellSkewAngle": VOL_POLY_MAX_CELL_SKEW_ANGLE,
      "AvoidOneEightTransition": VOL_AVOID_ONE_EIGHT_TRANSITION,
      "CheckSelfProximity": VOL_CHECK_SELF_PROXIMITY,
      "MinSize": VOL_MIN_SIZE,
      "MaxSize": SURF_MAX_SIZE,
      "GrowthRate": SURF_GROWTH_RATE,
   })
   candidate_states.append({
      "VolumeFill": VOL_FILL_TYPE,
      "VolumeFillControls": {
         "HexMaxCellLength": SURF_MAX_SIZE,
         "GrowthRate": SURF_GROWTH_RATE,
      },
   })
   applied = False
   for st in candidate_states:
      if apply_task_state(task, st, "Generate Volume Mesh"):
         applied = True
         break
   if not applied:
      raise RuntimeError("Generate the Volume Mesh 参数设置失败")
   print("   体网格参数已设置：")
   print(f"     Solver = {VOL_SOLVER}")
   print(f"     VolumeFill = {VOL_FILL_TYPE}")
   print(f"     MaxSize = {SURF_MAX_SIZE} mm")
   print(f"     GrowthRate = {SURF_GROWTH_RATE}")
   print_task_state(task, "Generate Volume Mesh 当前")


# ========================================================
# 5. 求解器辅助函数
# ========================================================

def tui_path_exists(obj, name):
   try:
      getattr(obj, name)
      return True
   except Exception:
      return False


def solver_tui(session, command):
   """单条 TUI 命令保底执行。优先使用 scheme_eval，兼容 Fluent 2024R1。"""
   cmd = str(command).strip()
   if not cmd:
      return True
   try:
      escaped = cmd.replace('\\', '/').replace('"', '\\"')
      session.scheme_eval.eval(f'(ti-menu-load-string "{escaped}\n")')
      return True
   except Exception:
      pass

   try:
      session.execute_tui(cmd)
      return True
   except Exception as e:
      print(f"   TUI 命令失败: {cmd} | {e}")
      return False


def write_and_read_journal(session, commands, journal_name):
   """把多条 TUI 指令写成 journal 后一次性执行，比逐条 execute_tui 更稳定。"""
   journal_path = os.path.join(WORK_DIR, journal_name).replace('\\', '/')
   with open(journal_path, 'w', encoding='utf-8') as f:
      for cmd in commands:
         f.write(str(cmd).rstrip() + '\n')
   print(f"   ▶ 执行 journal: {journal_path}")
   session.tui.file.read_journal(journal_path)
   return journal_path








def normalize_path_for_fluent(path):
   """所有传给 Fluent TUI/Journal 的 Windows 路径统一使用正斜杠。

   重要：类似 D:\\zidonghua\\new2 的路径如果直接送入 TUI，\\n 可能被解释为换行，
   从而把路径拆断。本函数是 v114 的统一路径入口。
   """
   return str(path).replace(chr(92), "/")


def safe_write_mesh_v241(meshing_session, mesh_path, wait_seconds=20):
   """Fluent 2024 R1 稳定写网格：正斜杠路径 + 禁止覆盖交互 + 落盘校验。"""
   local_path = os.path.abspath(str(mesh_path))
   fluent_path = normalize_path_for_fluent(local_path)

   folder = os.path.dirname(local_path)
   if folder and not os.path.exists(folder):
      os.makedirs(folder)

   # 避免 Fluent TUI 弹出 OK to overwrite? 导致自动化被交互提示卡死。
   if os.path.exists(local_path):
      try:
         os.remove(local_path)
         print(f"   已删除同名旧网格，避免 overwrite 交互: {local_path}")
      except Exception as e:
         raise RuntimeError(f"无法删除旧网格文件，不能安全自动写入: {local_path} | {e}")

   print(f"   Fluent 写网格路径(已规范化): {fluent_path}")
   meshing_session.tui.file.write_mesh(fluent_path)

   # write_mesh 返回后通常已经落盘，但额外轮询，避免网络盘/磁盘刷新延迟。
   t0 = time.time()
   while time.time() - t0 < float(wait_seconds):
      if os.path.exists(local_path) and os.path.getsize(local_path) > 0:
         print(f"   网格文件落盘确认: {local_path} | {os.path.getsize(local_path)} bytes")
         return local_path
      time.sleep(0.5)

   raise RuntimeError(
      "Fluent write_mesh 已返回，但没有检测到有效网格文件。"
      f" local={local_path} | fluent={fluent_path}"
   )



def _unique_keep_order(items):
   out = []
   for item in items:
      if item is None:
         continue
      s = str(item)
      if s not in out:
         out.append(s)
   return out


def safe_get_solver_zone_names(solver_session):
   """
   通用读取 Solver 中全部 zone 名称。
   """
   names = []

   roots = [
      "setup.boundary_conditions",
      "settings.setup.boundary_conditions",
   ]

   for root_path in roots:
      try:
         obj = solver_session
         for part in root_path.split("."):
            obj = getattr(obj, part)

         for attr in [
            "velocity_inlet", "pressure_outlet", "wall", "symmetry",
            "pressure_inlet", "outflow", "interior", "interface"
         ]:
            try:
               container = getattr(obj, attr)
               names.extend(list(container.keys()))
            except Exception:
               pass
      except Exception:
         pass

   try:
      zone_info = solver_session.fields.field_info.get_surfaces_info()
      if isinstance(zone_info, dict):
         names.extend(list(zone_info.keys()))
   except Exception:
      pass

   return _unique_keep_order(names)


def get_bc_names(solver_session, bc_type):
   """
   读取指定边界类型下的 zone 名称。
   兼容 velocity_inlet / pressure_outlet 以及 Fluent 中的 velocity-inlet / pressure-outlet。
   """
   candidates = [
      str(bc_type),
      str(bc_type).replace("-", "_"),
      str(bc_type).replace("_", "-"),
   ]

   names = []
   roots = [
      "setup.boundary_conditions",
      "settings.setup.boundary_conditions",
   ]

   for root_path in roots:
      try:
         obj = solver_session
         for part in root_path.split("."):
            obj = getattr(obj, part)

         for cand in candidates:
            try:
               container = getattr(obj, cand)
               names.extend(list(container.keys()))
            except Exception:
               pass
      except Exception:
         pass

   # 如果 settings API 没取到，按名称兜底。
   if not names:
      all_names = safe_get_solver_zone_names(solver_session)
      low_type = str(bc_type).lower().replace("_", "-")
      if low_type in ["velocity-inlet", "velocityinlet"]:
         names = [z for z in all_names if "inlet" in z.lower()]
      elif low_type in ["pressure-outlet", "pressureoutlet"]:
         names = [z for z in all_names if "outlet" in z.lower()]
      elif low_type == "wall":
         names = [z for z in all_names if z.lower() in ["wall", "auv_body", "shaft", "propeller"] or "wall" in z.lower()]
      elif low_type == "symmetry":
         names = [z for z in all_names if "sym" in z.lower()]

   return _unique_keep_order(names)


def find_zones_by_labels(zone_names, labels):
   """
   根据 label 从 zone_names 中匹配实际 Fluent zone。
   精确匹配优先，然后做包含匹配。
   """
   zones = _unique_keep_order(zone_names)
   labels = [str(x) for x in labels]

   result = []

   for label in labels:
      low = label.lower()

      for z in zones:
         if z.lower() == low and z not in result:
            result.append(z)

      for z in zones:
         zl = z.lower()
         if (low in zl or zl in low) and z not in result:
            result.append(z)

   return result


def compute_report_value_now(solver_session, report_name):
   """
   计算 Report Definition 当前值。
   优先用 settings API/TUI 触发计算，再从 transcript 中解析最后一个数值。
   """
   transcript_path = os.path.join(WORK_DIR, f"tmp_compute_{report_name}.trn").replace("/", os.sep)

   try:
      if os.path.exists(transcript_path):
         os.remove(transcript_path)
   except Exception:
      pass

   # settings API 尝试。
   try:
      rd = solver_session.settings.solution.report_definitions
      for container_name in ["drag", "force", "lift", "surface", "surface_report", "surface_reports"]:
         try:
            container = getattr(rd, container_name)
            if _container_has_key(container, report_name):
               obj = container[report_name]
               for method in ["compute", "Compute", "evaluate", "Evaluate"]:
                  try:
                     value = getattr(obj, method)()
                     if isinstance(value, (int, float)):
                        return float(value)
                     parsed = _parse_numeric_from_any(value)
                     if parsed is not None:
                        return parsed
                  except Exception:
                     pass
         except Exception:
            pass
   except Exception:
      pass

   # TUI + transcript 兜底。
   commands = [
      f"/solve/report-definitions/compute {report_name}",
      f"/solve/report-definitions/compute {report_name} quit",
   ]

   try:
      solver_tui(solver_session, f'/file/start-transcript "{normalize_path_for_fluent(transcript_path)}"')
      time.sleep(0.2)
      for cmd in commands:
         try:
            solver_tui(solver_session, cmd)
            time.sleep(0.3)
         except Exception:
            pass
   finally:
      try:
         solver_tui(solver_session, "/file/stop-transcript")
      except Exception:
         pass

   try:
      return extract_latest_force_value(transcript_path)
   except Exception:
      return None



def force_graphics_window_ready(solver_session):
   cmds = [
      "/display/set-window 1",
      "/display/open-window 1",
      "/display/set-window 1",
   ]

   for cmd in cmds:
      solver_tui(solver_session, cmd)

   time.sleep(0.5)



def apply_xoy_picture_graphics_options(solver_session):
   """
   保存 xoy 云图前设置图形选项：
   - Disable Graphics Grid Plane / Ground Plane
   - Enable Graphics Reflections

   Fluent 2024R1 GUI 里这个选项有时实际叫 ground-plane，
   而不是 grid-plane，所以这里把 grid-plane、ground-plane、
   floor-plane、reflections 的多种 TUI / Scheme 名称都尝试一遍。
   """
   print("   ▶ 设置 xoy 图片图形选项：Disable Graphics Grid Plane，Enable Graphics Reflections")

   force_graphics_window_ready(solver_session)

   # 1) Scheme/RP 变量兜底
   scheme_cmds = [
      # Grid / Ground plane OFF
      "(rpsetvar 'graphics/grid-plane? #f)",
      "(rpsetvar 'graphics/show-grid-plane? #f)",
      "(rpsetvar 'graphics/graphics-grid-plane? #f)",
      "(rpsetvar 'graphics/ground-plane? #f)",
      "(rpsetvar 'graphics/show-ground-plane? #f)",
      "(rpsetvar 'graphics/floor-plane? #f)",
      "(rpsetvar 'display/grid-plane? #f)",
      "(rpsetvar 'display/show-grid-plane? #f)",
      "(rpsetvar 'display/ground-plane? #f)",
      "(rpsetvar 'display/show-ground-plane? #f)",
      "(rpsetvar 'display/floor-plane? #f)",
      "(rpsetvar 'rendering/grid-plane? #f)",
      "(rpsetvar 'rendering/show-grid-plane? #f)",
      "(rpsetvar 'rendering/ground-plane? #f)",
      "(rpsetvar 'rendering/show-ground-plane? #f)",
      "(rpsetvar 'rendering/floor-plane? #f)",

      # Reflections ON
      "(rpsetvar 'graphics/reflections? #t)",
      "(rpsetvar 'graphics/graphics-reflections? #t)",
      "(rpsetvar 'graphics/enable-reflections? #t)",
      "(rpsetvar 'display/reflections? #t)",
      "(rpsetvar 'display/graphics-reflections? #t)",
      "(rpsetvar 'display/enable-reflections? #t)",
      "(rpsetvar 'rendering/reflections? #t)",
      "(rpsetvar 'rendering/graphics-reflections? #t)",
      "(rpsetvar 'rendering/enable-reflections? #t)",
   ]

   for s in scheme_cmds:
      try:
         solver_session.scheme_eval.eval(s)
      except Exception:
         pass

   # 2) PyFluent settings API 兜底
   possible_paths = [
      ("settings", "results", "graphics"),
      ("settings", "results", "graphics", "display"),
      ("settings", "results", "graphics", "rendering"),
      ("settings", "results", "graphics", "display_options"),
      ("settings", "results", "graphics", "rendering_options"),
      ("settings", "preferences", "graphics"),
      ("settings", "preferences", "graphics", "rendering"),
      ("settings", "preferences", "graphics", "display_options"),
   ]

   for path in possible_paths:
      try:
         obj = solver_session
         for attr in path:
            obj = getattr(obj, attr)

         for grid_key in [
            "grid_plane",
            "graphics_grid_plane",
            "show_grid_plane",
            "ground_plane",
            "show_ground_plane",
            "floor_plane",
            "show_floor_plane",
         ]:
            try:
               setattr(obj, grid_key, False)
               print(f"   已设置 {'.'.join(path)}.{grid_key}=False")
            except Exception:
               pass

         for refl_key in [
            "reflections",
            "graphics_reflections",
            "enable_reflections",
            "show_reflections",
         ]:
            try:
               setattr(obj, refl_key, True)
               print(f"   已设置 {'.'.join(path)}.{refl_key}=True")
            except Exception:
               pass
      except Exception:
         pass

   # 3) TUI 命令兜底：grid-plane + ground-plane 都尝试
   cmds = [
      # Disable Graphics Grid Plane / Ground Plane
      "/display/set/rendering-options/grid-plane? no",
      "/display/set/rendering-options/show-grid-plane? no",
      "/display/set/rendering-options/graphics-grid-plane? no",
      "/display/set/rendering-options/ground-plane? no",
      "/display/set/rendering-options/show-ground-plane? no",
      "/display/set/rendering-options/floor-plane? no",
      "/display/set/rendering-options/show-floor-plane? no",
      "/display/set/rendering-options/grid-plane no",
      "/display/set/rendering-options/ground-plane no",
      "/display/set/rendering-options/floor-plane no",
      "/display/set/display-options/grid-plane? no",
      "/display/set/display-options/show-grid-plane? no",
      "/display/set/display-options/graphics-grid-plane? no",
      "/display/set/display-options/ground-plane? no",
      "/display/set/display-options/show-ground-plane? no",
      "/display/set/display-options/floor-plane? no",
      "/display/set/grid-plane no",
      "/display/set/ground-plane no",
      "/display/set/floor-plane no",
      "/display/grid-plane no",
      "/display/ground-plane no",
      "/display/floor-plane no",

      # Enable Graphics Reflections
      "/display/set/rendering-options/reflections? yes",
      "/display/set/rendering-options/graphics-reflections? yes",
      "/display/set/rendering-options/enable-reflections? yes",
      "/display/set/rendering-options/reflections yes",
      "/display/set/rendering-options/graphics-reflections yes",
      "/display/set/rendering-options/enable-reflections yes",
      "/display/set/display-options/reflections? yes",
      "/display/set/display-options/graphics-reflections? yes",
      "/display/set/display-options/enable-reflections? yes",
      "/display/set/reflections yes",
      "/display/set/graphics-reflections yes",
      "/display/set/enable-reflections yes",
      "/display/reflections yes",
      "/display/graphics-reflections yes",

      "/display/update-scene",
      "/display/re-render",
   ]

   for cmd in cmds:
      solver_tui(solver_session, cmd)

   time.sleep(1.0)



def set_view_for_contour_image(solver_session, view_key):
   position = (0.0, 0.0, 1000.0)
   target = (0.0, 0.0, 0.0)
   up = (0.0, 1.0, 0.0)
   view_dir = (0.0, 0.0, -1.0)

   print("   ▶ 设置 xoy 视角: +Z front | +Y up | +X right")

   cmds = [
      "/display/set-window 1",
      f"/display/views/camera/position {position[0]} {position[1]} {position[2]}",
      f"/display/views/camera/target {target[0]} {target[1]} {target[2]}",
      f"/display/views/camera/up-vector {up[0]} {up[1]} {up[2]}",
      f"/display/views/set/view-direction {view_dir[0]} {view_dir[1]} {view_dir[2]}",
      f"/display/views/set/up-vector {up[0]} {up[1]} {up[2]}",
      "/display/views/auto-scale",
      "/display/update-scene",
      "/display/re-render",
   ]

   for cmd in cmds:
      solver_tui(solver_session, cmd)

   time.sleep(1.2)


def _try_get_contour_state(contour_obj):
   try:
      return contour_obj.get_state()
   except Exception:
      return None


def resolve_contour_field_name(solver_session, requested_field):
   """
   解析 Fluent 当前版本可用的云图变量名。
   Static Pressure 在 Fluent/PyFluent 中通常对应 pressure。
   """
   requested = str(requested_field).strip()
   if requested in ["pressure", "static-pressure", "static_pressure"]:
      candidates = ["pressure", "static-pressure", "static_pressure"]
   else:
      candidates = [requested]

   try:
      info = solver_session.fields.field_info.get_scalar_fields_info()
      if isinstance(info, dict):
         keys = list(info.keys())
         low_map = {str(k).lower(): k for k in keys}
         for c in candidates:
            if c in keys:
               return c
            if c.lower() in low_map:
               return low_map[c.lower()]
   except Exception:
      pass

   return candidates[0]


def _try_set_contour_field_only(contour_obj, contour_name, field_name):
   """
   只更新云图变量，不碰 surfaces。
   避免 Fluent 2024R1 报：
   api-set-var: the object is not active
   results/graphics/contour/.../surfaces
   """
   ok = False

   for st in [
      {"field": field_name},
      {"contours_of": field_name},
      {"contours-of": field_name},
   ]:
      try:
         contour_obj.set_state(st)
         print(f"   已通过 set_state 设置云图变量: {contour_name} | {st}")
         ok = True
         break
      except Exception:
         try:
            contour_obj.set_state(st, "replace")
            print(f"   已通过 set_state replace 设置云图变量: {contour_name} | {st}")
            ok = True
            break
         except Exception:
            pass

   for attr_name in ["field", "field_name", "field_variable", "contours_of"]:
      try:
         setattr(contour_obj, attr_name, field_name)
         print(f"   已设置 {contour_name}.{attr_name} = {field_name}")
         ok = True
      except Exception:
         pass

   return ok


def create_or_update_contour_for_picture(solver_session, contour_name, field_name, surface_name):
   """
   创建或更新云图对象。
   关键原则：
   1. 不删除已有云图对象；
   2. 已存在对象不再写 surfaces / surface_names，只更新 field；
   3. 新建对象时只尝试 surfaces_list，不使用 surfaces。
   """
   field_name = resolve_contour_field_name(solver_session, field_name)
   print(f"   准备云图对象: {contour_name} | field={field_name} | surface={surface_name}")

   try:
      contours = solver_session.settings.results.graphics.contour
      exists = contour_name in list(contours.keys())

      if not exists:
         created = False
         for init_state in [
            {"field": field_name, "surfaces_list": [surface_name], "filled": True, "node_values": True},
            {"field": field_name, "surfaces_list": [surface_name], "filled?": True, "node-values?": True},
            {"field": field_name, "surfaces_list": [surface_name]},
            {"contours_of": field_name, "surfaces_list": [surface_name], "filled": True},
            {"field": field_name},
            {},
         ]:
            try:
               contours[contour_name] = init_state
               print(f"   settings API 已新建云图对象: {contour_name} | {init_state}")
               created = True
               break
            except Exception:
               try:
                  contours.create(contour_name)
                  print(f"   settings API create 已新建云图对象: {contour_name}")
                  created = True
                  break
               except Exception:
                  pass
         if not created:
            print(f"   settings API 新建云图对象失败: {contour_name}")
      else:
         print(f"   云图对象已存在，不删除，且不重写 surfaces: {contour_name}")

      try:
         contour_obj = contours[contour_name]
         _try_set_contour_field_only(contour_obj, contour_name, field_name)
      except Exception as e:
         print(f"   取得或更新云图对象失败: {contour_name} | {e}")

      return True

   except Exception as e:
      print(f"   settings API 创建/更新云图对象失败: {contour_name} | {e}")

   # TUI 兜底，只在对象不存在时尝试创建。不删除已有对象。
   try:
      solver_tui(
         solver_session,
         f"/display/objects/create contour {contour_name} field {field_name} surfaces-list {surface_name} () quit",
      )
      return True
   except Exception:
      return False


def display_contour_for_picture(solver_session, contour_name, field_name, surface_name):
   """
   显示云图对象。优先显示对象，避免把残差窗口误保存成云图。
   """
   force_graphics_window_ready(solver_session)

   create_or_update_contour_for_picture(
      solver_session=solver_session,
      contour_name=contour_name,
      field_name=field_name,
      surface_name=surface_name,
   )

   displayed = False

   try:
      contour_obj = solver_session.settings.results.graphics.contour[contour_name]
      # 显示前只强制更新 field，不再写 surfaces。
      field_resolved = resolve_contour_field_name(solver_session, field_name)
      _try_set_contour_field_only(contour_obj, contour_name, field_resolved)

      try:
         contour_obj.display()
         displayed = True
         print(f"   已通过 settings API 显示云图对象: {contour_name}")
      except Exception:
         try:
            contour_obj.display(window_id=1)
            displayed = True
            print(f"   已通过 settings API 显示云图对象到窗口 1: {contour_name}")
         except Exception as e:
            print(f"   settings API 显示云图对象失败: {contour_name} | {e}")

   except Exception as e:
      print(f"   无法访问云图对象: {contour_name} | {e}")

   if not displayed:
      try:
         solver_tui(solver_session, f"/display/objects/display {contour_name}")
         displayed = True
         print(f"   已尝试通过 TUI 显示云图对象: {contour_name}")
      except Exception:
         displayed = False

   # 防止保存残差：显示后多刷新几次，给 GUI 充分时间切换。
   for cmd in [
      "/display/set-window 1",
      "/display/views/auto-scale",
      "/display/update-scene",
      "/display/re-render",
      "/display/update-scene",
      "/display/re-render",
   ]:
      solver_tui(solver_session, cmd)

   time.sleep(2.0)

   if not displayed:
      print(f"   云图对象未确认显示成功，跳过保存，避免保存上一张图: {contour_name}")

   return displayed


def try_save_picture_with_api(solver_session, image_path):
   image_path_fluent = normalize_path_for_fluent(image_path)

   try:
      solver_session.tui.display.set.picture.driver("png")
   except Exception:
      pass

   try:
      solver_session.tui.display.set.picture.x_resolution(PICTURE_X_RESOLUTION)
   except Exception:
      pass

   try:
      solver_session.tui.display.set.picture.y_resolution(PICTURE_Y_RESOLUTION)
   except Exception:
      pass

   try:
      solver_session.tui.display.save_picture(image_path_fluent)
      time.sleep(1.0)
      if os.path.exists(image_path):
         return True
   except Exception:
      pass

   try:
      solver_session.tui.display.hardcopy(image_path_fluent)
      time.sleep(1.0)
      if os.path.exists(image_path):
         return True
   except Exception:
      pass

   return False


def try_save_picture_with_tui(solver_session, image_path):
   image_path_fluent = normalize_path_for_fluent(image_path)

   journal_cmds = [
      "/display/set-window 1",
      "/display/set/picture/driver png",
      f"/display/set/picture/x-resolution {PICTURE_X_RESOLUTION}",
      f"/display/set/picture/y-resolution {PICTURE_Y_RESOLUTION}",
      "/display/set/picture/color-mode color",
      f'/display/save-picture "{image_path_fluent}"',
   ]

   try:
      write_and_read_journal(solver_session, journal_cmds, "save_picture.jou")
      time.sleep(1.0)
      if os.path.exists(image_path):
         return True
   except Exception as e:
      print(f"   journal 保存图片失败: {e}")

   for cmd in journal_cmds:
      solver_tui(solver_session, cmd)

   time.sleep(1.0)

   if os.path.exists(image_path):
      return True

   solver_tui(solver_session, f'/display/hardcopy "{image_path_fluent}"')
   time.sleep(1.0)

   return os.path.exists(image_path)


def prepare_picture_scene_before_save(solver_session, contour_name, field_name, surface_name, view_key):
   """
   为避免把残差窗口误存成速度云图，保存前先做一次不落盘的图形预热。
   第一组速度更容易出现 residual 截图，因此每次保存都统一预热。
   """
   force_graphics_window_ready(solver_session)

   pre_cmds = [
      "/display/set-window 1",
      "/plot/close-window",
      "/display/clear",
      "/display/update-scene",
      "/display/re-render",
   ]
   for cmd in pre_cmds:
      solver_tui(solver_session, cmd)

   time.sleep(0.8)

   displayed = False
   for _ in range(2):
      displayed = display_contour_for_picture(
         solver_session=solver_session,
         contour_name=contour_name,
         field_name=field_name,
         surface_name=surface_name,
      )
      set_view_for_contour_image(solver_session, view_key)
      apply_xoy_picture_graphics_options(solver_session)
      for cmd in [
         "/display/set-window 1",
         "/display/update-scene",
         "/display/re-render",
         "/display/update-scene",
         "/display/re-render",
      ]:
         solver_tui(solver_session, cmd)
      time.sleep(1.2)
      if displayed:
         break

   return displayed

def save_one_contour_picture(solver_session, contour_name, field_name, surface_name, file_name, view_key):
   image_path = os.path.abspath(os.path.join(WORK_DIR, file_name))

   if not image_path.lower().endswith(".png"):
      image_path += ".png"

   try:
      if os.path.exists(image_path):
         os.remove(image_path)
   except Exception:
      pass

   print(f"   ▶ 正在显示并保存云图图片: {contour_name} -> {image_path}")

   displayed = prepare_picture_scene_before_save(
      solver_session=solver_session,
      contour_name=contour_name,
      field_name=field_name,
      surface_name=surface_name,
      view_key=view_key,
   )

   if not displayed:
      print(f"   跳过保存图片，原因：云图未确认显示成功，避免误保存上一张图: {contour_name}")
      return False

   set_view_for_contour_image(solver_session, view_key)
   apply_xoy_picture_graphics_options(solver_session)
   for cmd in [
      "/display/set-window 1",
      "/display/update-scene",
      "/display/re-render",
      "/display/update-scene",
      "/display/re-render",
   ]:
      solver_tui(solver_session, cmd)

   time.sleep(1.5)

   ok = try_save_picture_with_api(solver_session, image_path)

   if not ok:
      ok = try_save_picture_with_tui(solver_session, image_path)

   if ok and os.path.exists(image_path):
      print(f"   已保存云图图片: {image_path}")
   else:
      print(f"   未检测到云图图片生成: {image_path}")
      print("     这说明当前 Fluent 版本没有响应 save-picture / hardcopy。")

   return ok

def save_contour_pictures(solver_session, velocity_tag=None):
   """
   保存 xoy 速度云图和 xoy 静压云图。
   为避免第一个速度工况把 residual 窗口误保存成速度云图，
   保存时先做一次 contour 显示预热，再正式落盘。
   """
   if not SAVE_CONTOUR_IMAGES:
      print("   跳过云图图片保存：SAVE_CONTOUR_IMAGES=False")
      return

   if not os.path.exists(WORK_DIR):
      os.makedirs(WORK_DIR)

   for c in CONTOURS_TO_CREATE:
      try:
         create_or_update_contour_for_picture(
            solver_session=solver_session,
            contour_name=c["name"],
            field_name=c["field"],
            surface_name=c["surface"],
         )
      except Exception as e:
         print(f"   云图对象预创建失败: {c['name']} | {e}")

   for spec in CONTOUR_IMAGE_SPECS:
      base_contour_name = spec["contour"]
      base_file = spec["file"]

      if velocity_tag:
         root, ext = os.path.splitext(base_file)
         file_name = f"{root}_{velocity_tag}{ext}"
         save_contour_name = f"{base_contour_name}_{velocity_tag}"
      else:
         file_name = base_file
         save_contour_name = base_contour_name

      contour_def = None
      for c in CONTOURS_TO_CREATE:
         if c["name"] == base_contour_name:
            contour_def = c
            break

      if contour_def is None:
         print(f"   未找到云图定义: {base_contour_name}")
         continue

      print(
         f"   云图保存任务: object={save_contour_name}, "
         f"field={contour_def['field']}, surface={contour_def['surface']}, file={file_name}"
      )

      save_one_contour_picture(
         solver_session=solver_session,
         contour_name=save_contour_name,
         field_name=contour_def["field"],
         surface_name=contour_def["surface"],
         file_name=file_name,
         view_key=spec["view"],
      )

   for c in CONTOURS_TO_CREATE:
      try:
         create_or_update_contour_for_picture(
            solver_session=solver_session,
            contour_name=c["name"],
            field_name=c["field"],
            surface_name=c["surface"],
         )
      except Exception as e:
         print(f"   云图对象确认失败: {c['name']} | {e}")

def _container_has_key(container, name):
   try:
      return name in container.keys()
   except Exception:
      try:
         return name in list(container)
      except Exception:
         return False


def _safe_pop_named(container, name):
   try:
      if _container_has_key(container, name):
         container.pop(name)
         return True
   except Exception:
      pass
   return False


def _try_set_state(obj, state):
   try:
      obj.set_state(state)
      return True
   except Exception:
      pass

   try:
      obj.set_state(state, "replace")
      return True
   except Exception:
      pass

   return False


def _try_set_attrs(obj, state):
   ok_any = False
   for k, v in state.items():
      candidates = [k, k.replace("-", "_"), k.replace("_", "-")]
      for kk in candidates:
         try:
            setattr(obj, kk, v)
            ok_any = True
            break
         except Exception:
            pass
   return ok_any


def resolve_force_report_zones_for_solver(solver_session, old_report_zones):
   """
   更稳地识别用于力报告的 AUV 壁面 zone。
   如果 SpaceClaim 传来的 report_zones 为空，则用当前 solver 里的 wall zones 兜底。
   外部计算域 wall 已经在前面转成 symmetry，所以此时剩余 wall 通常就是 AUV 表面。
   """
   zones = list(old_report_zones) if old_report_zones else []

   if len(zones) > 0:
      return zones

   try:
      current_wall_zones = get_bc_names(solver_session, "wall")
   except Exception:
      current_wall_zones = []

   exclude_keywords = ["inlet", "outlet", "symmetry", "fluid", "domain"]
   fallback = []
   for z in current_wall_zones:
      zl = z.lower()
      if any(k in zl for k in exclude_keywords):
         continue
      fallback.append(z)

   if len(fallback) > 0:
      print(f"   force report zones 为空，改用当前 solver wall zones 兜底: {fallback}")
      return fallback

   # 最后兜底：直接使用所有 wall zones
   if len(current_wall_zones) > 0:
      print(f"   force report zones 为空，最后兜底使用所有 wall zones: {current_wall_zones}")
      return current_wall_zones

   return []




def build_total_pressure_solution_export_path(velocity):
   """
   每个速度工况的 AUV 表面 total pressure 解数据导出文件。
   输出格式按 Fluent 手动导出的 ASCII 文件，不使用 .csv 后缀。
   """
   tag = speed_tag(velocity)
   return os.path.join(WORK_DIR, f"surface_total_pressure_cell_center_{tag}").replace(chr(92), "/")



def normalize_field_name_key(name):
   return str(name).lower().replace("_", "-").replace(" ", "-")


def list_available_scalar_field_names(solver_session):
   try:
      info = solver_session.fields.field_info.get_scalar_fields_info()
      if isinstance(info, dict):
         return list(info.keys())
   except Exception:
      pass
   return []


def resolve_scalar_field_name_by_candidates(solver_session, candidates, must_contain=None):
   """
   从 Fluent 当前可用标量变量中匹配字段名。
   """
   try:
      info = solver_session.fields.field_info.get_scalar_fields_info()
      if isinstance(info, dict):
         keys = list(info.keys())
         low_map = {str(k).lower(): k for k in keys}
         norm_map = {normalize_field_name_key(k): k for k in keys}

         for c in candidates:
            if c in keys:
               return c
            if str(c).lower() in low_map:
               return low_map[str(c).lower()]
            nc = normalize_field_name_key(c)
            if nc in norm_map:
               return norm_map[nc]

         if must_contain:
            for k in keys:
               nk = normalize_field_name_key(k)
               ok = True
               for token in must_contain:
                  if token not in nk:
                     ok = False
                     break
               if ok:
                  return k
   except Exception:
      pass

   return candidates[0]


def resolve_total_pressure_field_name(solver_session):
   """
   解析 Fluent/PyFluent 中 total pressure 的真实字段名。
   """
   return resolve_scalar_field_name_by_candidates(
      solver_session,
      candidates=[
         "total-pressure",
         "total_pressure",
         "total pressure",
         "Total Pressure",
         "pressure-total",
         "absolute-total-pressure",
      ],
      must_contain=["total", "pressure"],
   )


def resolve_coordinate_field_name(solver_session, axis):
   """
   解析 Fluent/PyFluent 中 x/y/z coordinate 的真实字段名。
   手动导出的表头是 x-coordinate、y-coordinate、z-coordinate。
   """
   axis = str(axis).lower().strip()
   return resolve_scalar_field_name_by_candidates(
      solver_session,
      candidates=[
         f"{axis}-coordinate",
         f"{axis}_coordinate",
         f"{axis} coordinate",
         f"{axis.upper()} Coordinate",
         f"coordinate-{axis}",
         f"coordinates-{axis}",
         f"{axis}-coordinates",
      ],
      must_contain=[axis, "coordinate"],
   )


def _flatten_numeric_sequence(obj, max_depth=10):
   """
   从 PyFluent field_data 返回对象中尽量提取数值序列。
   """
   if max_depth <= 0 or obj is None:
      return []

   for method_name in ["tolist", "to_list", "flatten"]:
      try:
         method = getattr(obj, method_name)
         out = method()
         vals = _flatten_numeric_sequence(out, max_depth=max_depth - 1)
         if vals:
            return vals
      except Exception:
         pass

   for attr_name in [
      "scalar_data", "data", "values", "field_data", "array", "as_array",
      "as_numpy_array", "to_numpy"
   ]:
      try:
         attr = getattr(obj, attr_name)
         if callable(attr):
            attr = attr()
         vals = _flatten_numeric_sequence(attr, max_depth=max_depth - 1)
         if vals:
            return vals
      except Exception:
         pass

   if isinstance(obj, dict):
      vals = []
      for v in obj.values():
         vals.extend(_flatten_numeric_sequence(v, max_depth=max_depth - 1))
      return vals

   if isinstance(obj, (list, tuple)):
      vals = []
      for item in obj:
         if isinstance(item, (int, float)):
            vals.append(float(item))
         else:
            vals.extend(_flatten_numeric_sequence(item, max_depth=max_depth - 1))
      return vals

   try:
      if isinstance(obj, (int, float)):
         return [float(obj)]
   except Exception:
      pass

   return []


def _extract_vector3_sequence(obj):
   """
   从 PyFluent surface data 中尽量提取三维坐标序列。
   """
   try:
      if isinstance(obj, dict):
         for v in obj.values():
            out = _extract_vector3_sequence(v)
            if out:
               return out
      if isinstance(obj, (list, tuple)):
         if len(obj) > 0 and isinstance(obj[0], (list, tuple)) and len(obj[0]) >= 3:
            out = []
            for p in obj:
               try:
                  out.append((float(p[0]), float(p[1]), float(p[2])))
               except Exception:
                  pass
            if out:
               return out
   except Exception:
      pass

   flat = _flatten_numeric_sequence(obj)
   if len(flat) >= 3:
      n = len(flat) // 3
      return [(flat[3*i], flat[3*i + 1], flat[3*i + 2]) for i in range(n)]

   return []


def _get_surface_info_map(solver_session):
   try:
      info = solver_session.fields.field_info.get_surfaces_info()
      if isinstance(info, dict):
         return info
   except Exception:
      pass
   return {}


def _get_surface_id_candidates(solver_session, zone_name):
   """
   为 field_data API 准备 surface 名称/id 候选。
   """
   candidates = [zone_name]
   info = _get_surface_info_map(solver_session)

   if isinstance(info, dict):
      zinfo = info.get(zone_name)
      if isinstance(zinfo, dict):
         for key in ["surface_id", "id", "zone_id", "thread_id"]:
            if key in zinfo:
               candidates.append(zinfo[key])

      for k, v in info.items():
         try:
            if str(k).lower() == str(zone_name).lower():
               candidates.append(k)
            if isinstance(v, dict):
               for nf in ["surface_name", "name", "zone_name", "thread_name"]:
                  if nf in v and str(v[nf]).lower() == str(zone_name).lower():
                     candidates.append(k)
                     for id_key in ["surface_id", "id", "zone_id", "thread_id"]:
                        if id_key in v:
                           candidates.append(v[id_key])
         except Exception:
            pass

   out = []
   for c in candidates:
      if c not in out:
         out.append(c)
   return out


def resolve_total_pressure_export_zones(solver_session, report_zones):
   """
   更稳地识别 AUV 表面 zones。
   """
   zones = resolve_force_report_zones_for_solver(solver_session, report_zones)
   zones = list(zones) if zones else []

   try:
      wall_zones = get_bc_names(solver_session, "wall")
   except Exception:
      wall_zones = []

   exclude_keywords = ["inlet", "outlet", "symmetry", "fluid", "domain"]
   known_part_keywords = [
      "auv", "body", "shaft", "prop", "propeller", "rudder", "fin",
      "wing", "appendage", "fairing", "hull", "sail", "strut"
   ]

   for z in wall_zones:
      zl = str(z).lower()
      if any(k in zl for k in exclude_keywords):
         continue
      if z not in zones:
         zones.append(z)

   try:
      surf_info = _get_surface_info_map(solver_session)
      for name in surf_info.keys():
         nl = str(name).lower()
         if any(k in nl for k in exclude_keywords):
            continue
         if any(k in nl for k in known_part_keywords):
            if name not in zones:
               zones.append(name)
   except Exception:
      pass

   return zones


def get_scalar_field_data_for_zone(solver_session, field_name, zone_name):
   """
   读取某个 AUV 表面 zone 上的标量数据。
   Cell Center / face center 对 PyFluent 通常对应 node_value=False。
   """
   fd = solver_session.fields.field_data
   surface_candidates = _get_surface_id_candidates(solver_session, zone_name)

   call_attempts = []
   for surf in surface_candidates:
      call_attempts.extend([
         lambda surf=surf: fd.get_scalar_field_data(field_name=field_name, surfaces=[surf], node_value=False),
         lambda surf=surf: fd.get_scalar_field_data(field_name=field_name, surfaces=[surf], node_value="false"),
         lambda surf=surf: fd.get_scalar_field_data(field_name=field_name, surfaces=[surf]),
         lambda surf=surf: fd.get_scalar_field_data(field_name, [surf]),
         lambda surf=surf: fd.get_scalar_field_data(field_name, surf),
      ])

   last_err = None
   for call in call_attempts:
      try:
         raw = call()
         vals = _flatten_numeric_sequence(raw)
         if vals:
            return vals
      except Exception as e:
         last_err = e

   if last_err is not None:
      print(f"   标量 field_data 读取失败: field={field_name}, zone={zone_name} | {last_err}")
   else:
      print(f"   标量 field_data 读取失败: field={field_name}, zone={zone_name}")
   return []


def get_coordinate_fields_for_zone(solver_session, zone_name):
   """
   优先通过 x-coordinate/y-coordinate/z-coordinate 三个标量字段读取单元中心坐标。
   这比旧版从 SurfaceDataType 里找 centroid 更接近 Fluent 手动导出的 Solution Data。
   """
   x_field = resolve_coordinate_field_name(solver_session, "x")
   y_field = resolve_coordinate_field_name(solver_session, "y")
   z_field = resolve_coordinate_field_name(solver_session, "z")

   xs = get_scalar_field_data_for_zone(solver_session, x_field, zone_name)
   ys = get_scalar_field_data_for_zone(solver_session, y_field, zone_name)
   zs = get_scalar_field_data_for_zone(solver_session, z_field, zone_name)

   n = min(len(xs), len(ys), len(zs))
   if n > 0:
      return [(xs[i], ys[i], zs[i]) for i in range(n)]

   return []


def get_face_centroids_for_zone(solver_session, zone_name):
   """
   坐标读取兜底：
   1. 优先读 x/y/z-coordinate 标量字段；
   2. 再尝试 SurfaceDataType 的 face centroid。
   """
   coords = get_coordinate_fields_for_zone(solver_session, zone_name)
   if coords:
      return coords

   try:
      fd = solver_session.fields.field_data
   except Exception:
      return []

   surface_candidates = _get_surface_id_candidates(solver_session, zone_name)

   data_type_candidates = []

   try:
      from ansys.fluent.core.services.field_data import SurfaceDataType
      for attr in [
         "FacesCentroid", "FaceCentroid", "FacesCentroids",
         "FacesCentroidData", "Centroid", "Vertices",
         "FACES_CENTROID", "FACE_CENTROID", "FACES_CENTROIDS", "VERTICES"
      ]:
         try:
            data_type_candidates.append([getattr(SurfaceDataType, attr)])
         except Exception:
            pass
   except Exception:
      pass

   data_type_candidates.extend([
      ["faces-centroid"],
      ["face-centroid"],
      ["faces-centroids"],
      ["centroid"],
      ["vertices"],
   ])

   for surf in surface_candidates:
      for data_types in data_type_candidates:
         for call in [
            lambda surf=surf, data_types=data_types: fd.get_surface_data(data_types=data_types, surfaces=[surf]),
            lambda surf=surf, data_types=data_types: fd.get_surface_data(surfaces=[surf], data_types=data_types),
            lambda surf=surf, data_types=data_types: fd.get_surface_data(data_types, [surf]),
         ]:
            try:
               raw = call()
               coords = _extract_vector3_sequence(raw)
               if coords:
                  return coords
            except Exception:
               pass

   return []


def format_manual_solution_data_number(value):
   """
   按 Fluent 手动导出的 ASCII 风格输出科学计数法。
   例：-3.265637755E-01 或  2.849999964E-01
   """
   try:
      return f"{float(value): .9E}"
   except Exception:
      return ""


def write_manual_style_total_pressure_file(export_path, rows):
   """
   写成和手动 File -> Export -> Solution Data 类似的格式。
   表头和列顺序：
      cellnumber,    x-coordinate,    y-coordinate,    z-coordinate,  total-pressure
   """
   with open(export_path, "w", encoding="utf-8") as f:
      f.write("cellnumber,    x-coordinate,    y-coordinate,    z-coordinate,  total-pressure\n")
      for row in rows:
         cellnumber, x, y, z, p = row
         f.write(
            f"{int(cellnumber):10d},"
            f"{format_manual_solution_data_number(x)},"
            f"{format_manual_solution_data_number(y)},"
            f"{format_manual_solution_data_number(z)},"
            f"{format_manual_solution_data_number(p)}\n"
         )


def try_export_total_pressure_by_tui(solver_session, export_path, export_zones, field_name):
   """
   尝试使用 Fluent 原生 File -> Export -> Solution Data/ASCII 导出。
   输出文件不带 .csv 后缀，和手动保存文件一致。
   """
   if not export_zones:
      return False

   zone_expr = " ".join(export_zones)
   fluent_path = normalize_path_for_fluent(export_path)

   commands_variants = [
      [
         "/file/export/ascii",
         fluent_path,
         zone_expr,
         "()",
         "no",
         field_name,
         "()",
      ],
      [
         f"/file/export/ascii {fluent_path} {zone_expr} () no {field_name} ()",
      ],
      [
         "/file/export/solution-data",
         fluent_path,
         "ascii",
         "cell-center",
         "comma",
         zone_expr,
         "()",
         field_name,
         "()",
      ],
      [
         f"/file/export/solution-data {fluent_path} ascii cell-center comma {zone_expr} () {field_name} ()",
      ],
   ]

   for i, cmds in enumerate(commands_variants, start=1):
      try:
         before_size = os.path.getsize(export_path) if os.path.exists(export_path) else 0
      except Exception:
         before_size = 0

      try:
         write_and_read_journal(solver_session, cmds, f"export_total_pressure_solution_data_{i}.jou")
         time.sleep(0.8)
      except Exception as e:
         print(f"   TUI 导出 total pressure 尝试 {i} 失败: {e}")

      try:
         after_size = os.path.getsize(export_path) if os.path.exists(export_path) else 0
      except Exception:
         after_size = 0

      if after_size > before_size and after_size > 100:
         print(f"   TUI 导出 total pressure 成功: {export_path}")
         return True

   return False


def export_total_pressure_solution_data(solver_session, report_zones, velocity):
   """
   导出当前速度工况 AUV 表面 total pressure 解数据。

   对应手动操作：
      File -> Export -> Solution Data
      File Type: ASCII
      Location: Cell Center
      Delimiter: Comma
      Surfaces: auv_body、shaft、propeller 等所有 AUV 表面
      Quantities: Total Pressure

   输出文件格式按手动保存文件：
      cellnumber,    x-coordinate,    y-coordinate,    z-coordinate,  total-pressure
   """
   export_zones = resolve_total_pressure_export_zones(solver_session, report_zones)
   export_path = build_total_pressure_solution_export_path(velocity).replace("/", os.sep)
   field_name = resolve_total_pressure_field_name(solver_session)
   available_fields = list_available_scalar_field_names(solver_session)

   # 清理旧版 total pressure 导出日志，不再保留 surface_total_pressure_export_log_*.out。
   try:
      for _fn in os.listdir(WORK_DIR):
         if str(_fn).lower().startswith("surface_total_pressure_export_log_") and str(_fn).lower().endswith(".out"):
            try:
               os.remove(os.path.join(WORK_DIR, _fn))
            except Exception:
               pass
   except Exception:
      pass

   print("   ▶ 导出 AUV 表面 total pressure Solution Data")
   print(f"   ▶ 速度: {velocity} m/s")
   print(f"   ▶ Field Variable: {field_name}")
   print(f"   ▶ Location: Cell Center")
   print(f"   ▶ Delimiter: Comma")
   print(f"   ▶ Surfaces: {export_zones}")
   print(f"   ▶ 输出文件: {export_path}")

   folder = os.path.dirname(export_path)
   if folder and not os.path.exists(folder):
      os.makedirs(folder)

   write_manual_style_total_pressure_file(export_path, [])

   if not export_zones:
      print("   未找到 AUV 表面 zone，已生成空文件和日志。")
      return export_path

   all_rows = []
   cellnumber = 1
   missing_coordinate_zones = []

   for zone in export_zones:
      values = get_scalar_field_data_for_zone(solver_session, field_name, zone)
      centroids = get_face_centroids_for_zone(solver_session, zone)

      if not values:
         print(f"   zone={zone} 未读取到 total pressure 数据。")
         continue

      if len(centroids) < len(values):
         missing_coordinate_zones.append(zone)
         print(f"   zone={zone} 坐标数量不足: coords={len(centroids)}, values={len(values)}")

      for i, val in enumerate(values):
         x = y = z = ""
         if i < len(centroids):
            try:
               x, y, z = centroids[i]
            except Exception:
               x = y = z = ""
         all_rows.append((cellnumber, x, y, z, val))
         cellnumber += 1

      print(f"   zone={zone} 导出 total pressure 数据点数: {len(values)}")

   if len(all_rows) > 0:
      write_manual_style_total_pressure_file(export_path, all_rows)
      print(f"   total pressure solution data 已保存: {export_path}")
      if missing_coordinate_zones:
         print(f"   注意：以下 zone 的坐标数量不足: {missing_coordinate_zones}")
         print(f"   当前可用标量字段数量: {len(available_fields)}")
      return export_path

   print("   PyFluent field_data 没有读到有效 total pressure，开始尝试 Fluent TUI 原生导出。")
   ok_tui = try_export_total_pressure_by_tui(
      solver_session=solver_session,
      export_path=export_path,
      export_zones=export_zones,
      field_name=field_name,
   )

   try:
      size_now = os.path.getsize(export_path)
   except Exception:
      size_now = 0

   if ok_tui:
      print(f"   total pressure solution data 已由 Fluent TUI 导出: {export_path}")
   else:
      print(f"   total pressure solution data 未读到有效数据，但已保留手动格式表头: {export_path}")
      print(f"   文件大小: {size_now}")
      print(f"   当前识别字段: {field_name}")
      print(f"   当前识别表面: {export_zones}")
      print(f"   当前可用标量字段数量: {len(available_fields)}")

   return export_path


def create_report_definition_settings_api(solver_session, name, vector, report_zones):
   """
   通过 PyFluent settings API 创建 drag report definition。
   对 x/y/z 三个方向都用 drag 类型，只改变 force_vector。
   """
   vx, vy, vz = vector
   state_candidates = [
      {
         "thread_names": report_zones,
         "force_vector": [vx, vy, vz],
         "scaled": False,
      },
      {
         "thread-names": report_zones,
         "force-vector": [vx, vy, vz],
         "scaled?": False,
      },
      {
         "zones": report_zones,
         "force_vector": [vx, vy, vz],
         "scaled": False,
      },
   ]

   try:
      report_definitions = solver_session.settings.solution.report_definitions
   except Exception as e:
      print(f"   无法访问 settings.solution.report_definitions: {e}")
      return False

   # 优先 drag 容器。drag + force_vector 可以表示任意方向力。
   for container_name in ["drag", "force", "lift", "surface", "surface_report", "surface_reports"]:
      try:
         container = getattr(report_definitions, container_name)
      except Exception:
         continue

      try:
         _safe_pop_named(container, name)
      except Exception:
         pass

      try:
         container[name] = {}
      except Exception:
         try:
            container.create(name)
         except Exception:
            continue

      try:
         obj = container[name]
      except Exception:
         continue

      ok = False
      for st in state_candidates:
         if _try_set_state(obj, st):
            ok = True
            break
         if _try_set_attrs(obj, st):
            ok = True
            break

      if ok:
         print(f"   settings API 已创建 report definition: {name} | container={container_name}")
         return True

   return False


def create_report_file_settings_api(solver_session, name, report_file):
   """
   通过 PyFluent settings API 创建 report file。
   """
   state_candidates = [
      {"report_defs": [name], "file_name": report_file, "frequency": 1},
      {"report-defs": [name], "file-name": report_file, "frequency": 1},
      {"report_defs": [name], "file_name": report_file},
      {"report-defs": [name], "file-name": report_file},
   ]

   try:
      report_files_container = solver_session.settings.solution.monitor.report_files
   except Exception as e:
      print(f"   无法访问 settings.solution.monitor.report_files: {e}")
      return False

   try:
      _safe_pop_named(report_files_container, name)
   except Exception:
      pass

   try:
      report_files_container[name] = {}
   except Exception:
      try:
         report_files_container.create(name)
      except Exception as e:
         print(f"   settings API 创建 report file 容器失败: {name} | {e}")
         return False

   try:
      obj = report_files_container[name]
   except Exception:
      return False

   for st in state_candidates:
      if _try_set_state(obj, st):
         print(f"   settings API 已创建 report file: {name} -> {report_file}")
         return True
      if _try_set_attrs(obj, st):
         print(f"   settings API 已设置 report file: {name} -> {report_file}")
         return True

   return False


def create_force_reports_tui_journal(solver_session, report_zones, report_files, report_configs):
   """
   TUI journal 兜底创建三个方向力报告。
   注意这里三个方向都使用 drag report definition，只修改 force-vector。
   """
   if len(report_zones) == 0:
      return []

   zone_expr = " ".join(report_zones)
   cmds = []

   # 删除旧 report/report-file。失败不影响继续。
   for cfg in report_configs:
      name = cfg["name"]
      cmds.extend([
         f"/solve/report-files/delete {name}",
         f"/solve/report-definitions/delete {name}",
      ])

   for cfg in report_configs:
      name = cfg["name"]
      vx, vy, vz = cfg["vector"]
      report_file = report_files[name]

      # 同时给两种常用顺序，增强兼容性。
      cmds.append(
         f'/solve/report-definitions/add {name} drag thread-names {zone_expr} () force-vector {vx} {vy} {vz} scaled? no quit'
      )
      cmds.append(
         f'/solve/report-files/add {name} report-defs {name} () file-name "{report_file}" frequency 1 quit'
      )

   try:
      jou = write_and_read_journal(solver_session, cmds, "setup_force_xyz_reports.jou")
      print(f"   已执行三方向力报告 journal: {jou}")
   except Exception as e:
      print(f"   三方向力报告 journal 执行失败: {e}")

   # 执行后检查 settings 里是否存在。
   created = []
   for cfg in report_configs:
      name = cfg["name"]
      try:
         rd = solver_session.settings.solution.report_definitions
         exists = False
         for c_name in ["drag", "force", "lift"]:
            try:
               c = getattr(rd, c_name)
               if _container_has_key(c, name):
                  exists = True
                  break
            except Exception:
               pass
         if exists:
            created.append(name)
      except Exception:
         pass

   return created


def create_force_reports(solver_session, report_zones, report_files):
   """
   创建三个方向的力报告：
   - force_drag：x 方向阻力
   - force_sway_y：y 方向侧向力
   - force_heave_z：z 方向垂向力

   这一版做了三层保险：
   1. 如果 report_zones 为空，自动用当前 solver 的 wall zones 兜底；
   2. 先用 PyFluent settings API 创建 report definition 和 report file；
   3. 再用 TUI journal 兜底创建一次。
   """
   report_zones = resolve_force_report_zones_for_solver(solver_session, report_zones)

   if len(report_zones) == 0:
      print("   没有找到 AUV 表面 wall zone，三个方向的力报告暂不创建。")
      return []

   # 删除旧文件，避免误判上一轮旧结果。
   for p in report_files.values():
      try:
         local_p = str(p).replace("/", os.sep)
         if os.path.exists(local_p):
            os.remove(local_p)
      except Exception:
         pass

   report_configs = [
      {"name": REPORT_NAME, "vector": (1, 0, 0), "cn": "阻力_X"},
      {"name": REPORT_SWAY_Y_NAME, "vector": (0, 1, 0), "cn": "侧向力_Y"},
      {"name": REPORT_HEAVE_Z_NAME, "vector": (0, 0, 1), "cn": "垂向力_Z"},
   ]

   created = []

   for cfg in report_configs:
      name = cfg["name"]
      print(f"   ▶ 创建 {cfg['cn']} 报告: {name} | vector={cfg['vector']} | zones={report_zones}")

      ok_def = create_report_definition_settings_api(
         solver_session=solver_session,
         name=name,
         vector=cfg["vector"],
         report_zones=report_zones,
      )

      ok_file = False
      if ok_def:
         ok_file = create_report_file_settings_api(
            solver_session=solver_session,
            name=name,
            report_file=report_files[name],
         )

      if ok_def and ok_file:
         created.append(name)
      else:
         print(f"   settings API 未能完整创建 {name}，后续使用 TUI journal 兜底。")

   # 无论 settings API 是否成功，再走一遍 TUI journal 兜底，确保 Fluent GUI 中能看到报告。
   tui_created = create_force_reports_tui_journal(
      solver_session=solver_session,
      report_zones=report_zones,
      report_files=report_files,
      report_configs=report_configs,
   )

   for name in tui_created:
      if name not in created:
         created.append(name)

   print(f"   ▶ 三方向力报告创建结果: {created}")
   return created




# ========================================================
# 6.0 多速度批量计算与增强后处理函数覆盖区
# ========================================================

RESULT_MANIFEST_PATH = os.path.join(WORK_DIR, "result_manifest.json")


def _iso_now():
   return datetime.now().astimezone().isoformat(timespec="seconds")


def _path_if_exists(path):
   if not path:
      return None
   p = os.path.abspath(str(path))
   return p if os.path.exists(p) else p


def build_initial_result_manifest():
   return {
      "schema_version": "1.0",
      "status": "running",
      "started_at": _iso_now(),
      "completed_at": None,
      "model": {
         "path": os.path.abspath(str(INPUT_MODEL)),
         "name": MODEL_NAME,
      },
      "work_dir": os.path.abspath(str(WORK_DIR)),
      "request": {
         "velocities_m_s": [float(v) for v in VELOCITY_LIST],
         "processor_count": int(PROCESSOR_COUNT),
         "iterations_per_velocity": int(ITERATIONS),
         "save_contour_images": bool(SAVE_CONTOUR_IMAGES),
      },
      "software": {
         "spaceclaim_exe": SPACECLAIM_EXE,
         "fluent_exe": FLUENT_EXE_PATH,
         "fluent_release_hint": "2024 R1 / v241",
      },
      "cases": [],
      "artifacts": {},
      "errors": [],
   }


def write_result_manifest(manifest):
   try:
      Path(RESULT_MANIFEST_PATH).write_text(
         json.dumps(manifest, ensure_ascii=False, indent=2),
         encoding="utf-8",
      )
      print(f"   result_manifest 已保存: {RESULT_MANIFEST_PATH}")
   except Exception as e:
      print(f"   result_manifest 写入失败: {e}")
   return RESULT_MANIFEST_PATH


def finalize_result_manifest(manifest, summary_excel=None):
   requested = [float(v) for v in VELOCITY_LIST]
   success_velocities = [
      float(c.get("velocity_m_s")) for c in manifest.get("cases", [])
      if c.get("status") == "success"
   ]
   missing = [v for v in requested if v not in success_velocities]

   if not missing and len(success_velocities) == len(requested):
      manifest["status"] = "success"
   elif success_velocities:
      manifest["status"] = "partial_failure"
   else:
      manifest["status"] = "failed"

   manifest["completed_at"] = _iso_now()
   manifest["missing_velocities_m_s"] = missing

   artifacts = manifest.setdefault("artifacts", {})
   if summary_excel:
      artifacts["summary_excel"] = os.path.abspath(str(summary_excel))

   chart_files = [
      "force_x_vs_velocity.png",
      "force_y_vs_velocity.png",
      "force_z_vs_velocity.png",
   ]
   artifacts["force_velocity_charts"] = [
      os.path.abspath(os.path.join(WORK_DIR, f))
      for f in chart_files
      if os.path.exists(os.path.join(WORK_DIR, f))
   ]
   artifacts["adaptive_mesh_summary"] = (
      os.path.abspath(ADAPTIVE_SUMMARY_PATH)
      if os.path.exists(ADAPTIVE_SUMMARY_PATH) else None
   )
   write_result_manifest(manifest)
   return manifest


def speed_tag(velocity):
   """
   文件名速度标签。
   2.0 -> v2ms
   2.5 -> v2p5ms
   """
   s = ("%g" % float(velocity)).replace("-", "m").replace(".", "p")
   return f"v{s}ms"


def build_report_files_for_velocity(velocity):
   tag = speed_tag(velocity)
   return {
      REPORT_NAME: os.path.join(WORK_DIR, f"{REPORT_NAME}_{tag}{REPORT_FILE_EXT}").replace(chr(92), "/"),
      REPORT_SWAY_Y_NAME: os.path.join(WORK_DIR, f"{REPORT_SWAY_Y_NAME}_{tag}{REPORT_FILE_EXT}").replace(chr(92), "/"),
      REPORT_HEAVE_Z_NAME: os.path.join(WORK_DIR, f"{REPORT_HEAVE_Z_NAME}_{tag}{REPORT_FILE_EXT}").replace(chr(92), "/"),

      # Results -> Reports -> Forces 手动操作对应的两个分量：
      # Pressure = 压差阻力，Viscous = 摩擦阻力，方向向量为 (1, 0, 0)。
      REPORT_DRAG_PRESSURE_NAME: os.path.join(WORK_DIR, f"{REPORT_DRAG_PRESSURE_NAME}_{tag}{REPORT_FILE_EXT}").replace(chr(92), "/"),
      REPORT_DRAG_FRICTION_NAME: os.path.join(WORK_DIR, f"{REPORT_DRAG_FRICTION_NAME}_{tag}{REPORT_FILE_EXT}").replace(chr(92), "/"),
   }



def create_surface_pressure_report_definition_settings_api(solver_session, name, report_kind, report_zones):
   """
   创建 Report Definitions -> New -> Surface Report。
   report_kind:
      vertex_average
      vertex_maximum
   Field Variable: pressure/static pressure
   Surfaces: AUV 表面 zones，例如 auv_body、shaft、propeller 等。
   """
   report_zones = resolve_force_report_zones_for_solver(solver_session, report_zones)
   if len(report_zones) == 0:
      return False

   if report_kind == "vertex_average":
      type_values = ["surface-vertexavg", "surface-vertexavg", "surface-vertexavg", "surface-vertexavg?"]
   else:
      type_values = ["surface-vertexmax", "surface-vertexmax", "surface-vertexmax", "surface-vertexmax?"]

   field_values = ["pressure", "static-pressure", "static_pressure"]

   try:
      report_definitions = solver_session.settings.solution.report_definitions
   except Exception as e:
      print(f"   无法访问 surface report definitions: {e}")
      return False

   container_candidates = ["surface", "surface_report", "surface_reports"]

   state_candidates = []
   for tp in type_values:
      for fld in field_values:
         state_candidates.extend([
            {"report_type": tp, "field": fld, "surface_names": report_zones},
            {"report-type": tp, "field": fld, "surface-names": report_zones},
            {"surface_report_type": tp, "field_variable": fld, "surfaces": report_zones},
            {"surface-report-type": tp, "field-variable": fld, "surfaces": report_zones},
            {"report_type": tp, "field_variable": fld, "thread_names": report_zones},
            {"report-type": tp, "field-variable": fld, "thread-names": report_zones},
         ])

   for container_name in container_candidates:
      try:
         container = getattr(report_definitions, container_name)
      except Exception:
         continue

      try:
         _safe_pop_named(container, name)
      except Exception:
         pass

      try:
         container[name] = {}
      except Exception:
         try:
            container.create(name)
         except Exception:
            continue

      try:
         obj = container[name]
      except Exception:
         continue

      for st in state_candidates:
         if _try_set_state(obj, st) or _try_set_attrs(obj, st):
            print(f"   settings API 已创建表面静压报告: {name} | {report_kind} | zones={report_zones}")
            return True

   return False


def create_surface_pressure_reports(solver_session, report_zones, report_files):
   """
   创建两个表面静压报告：
   1. surface_pressure_vertex_average：surface-vertexavg
   2. surface_pressure_vertex_maximum：surface-vertexmax
   """
   report_zones = resolve_force_report_zones_for_solver(solver_session, report_zones)

   if len(report_zones) == 0:
      print("   没有找到 AUV 表面 zone，表面静压报告暂不创建。")
      return []

   created = []

   configs = [
      (REPORT_SURFACE_PRESSURE_AVG_NAME, "vertex_average", "表面静压 surface-vertexavg"),
      (REPORT_SURFACE_PRESSURE_MAX_NAME, "vertex_maximum", "表面静压 surface-vertexmax"),
   ]

   for name, kind, cn in configs:
      print(f"   ▶ 创建 {cn} 报告: {name} | field=pressure | zones={report_zones}")

      try:
         local_p = str(report_files[name]).replace("/", os.sep)
         if os.path.exists(local_p):
            os.remove(local_p)
      except Exception:
         pass

      ok_def = create_surface_pressure_report_definition_settings_api(
         solver_session=solver_session,
         name=name,
         report_kind=kind,
         report_zones=report_zones,
      )

      ok_file = False
      if ok_def:
         ok_file = create_report_file_settings_api(
            solver_session=solver_session,
            name=name,
            report_file=report_files[name],
         )

      if ok_def and ok_file:
         created.append(name)
      else:
         print(f"   {name} 创建未完全成功。可在 Fluent GUI 中按 Surface Report 检查。")

   return created


def write_force_summary_excel(all_force_rows):
   """
   多速度水动力结果合并到一个 Excel。
   包含：
      速度
      三方向合力 Force_X/Y/Z
      压差阻力 Pressure Drag_X
      摩擦阻力 Friction Drag_X
   """
   xlsx_path = os.path.join(WORK_DIR, "auv_force_summary_all_speeds.xlsx")
   rows = [[
      "Velocity (m/s)",
      "Force_X_Total (N)",
      "Force_Y_Total (N)",
      "Force_Z_Total (N)",
      "Pressure_Drag_X (N)",
      "Friction_Drag_X (N)",
   ]]

   for row in all_force_rows:
      rows.append([
         row.get("velocity", ""),
         row.get("drag_x", ""),
         row.get("sway_y", ""),
         row.get("heave_z", ""),
         row.get("drag_pressure", ""),
         row.get("drag_friction", ""),
      ])

   saved_path = None

   try:
      saved_path = write_simple_xlsx(xlsx_path, rows, sheet_name="force_summary")
      print(f"   多速度水动力 Excel 已保存: {saved_path}")
   except Exception as e:
      print(f"   xlsx 写入失败: {e}")
      saved_path = xlsx_path

   try:
      write_force_vs_velocity_charts(all_force_rows)
   except Exception as e:
      print(f"   力-速度关系 PNG 图生成失败: {e}")

   return saved_path



def _to_float_or_none(value):
   try:
      if value is None:
         return None
      s = str(value).strip()
      if s == "" or s.lower() in ["none", "not found", "nan"]:
         return None
      return float(s)
   except Exception:
      return None



def ensure_matplotlib_available():
   """
   确保当前 Python 环境具备 matplotlib。
   新工作站的 pyfluent_v241 环境若未安装，则自动尝试安装；
   安装失败时抛出明确错误，而不是静默跳过三张力-速度关系图。
   """
   try:
      import matplotlib  # noqa: F401
      return True
   except Exception:
      pass

   print("   当前 Python 环境未检测到 matplotlib，正在自动安装...")
   try:
      subprocess.check_call([
         sys.executable, "-m", "pip", "install", "matplotlib"
      ])
      import importlib
      importlib.invalidate_caches()
      import matplotlib  # noqa: F401
      print("   matplotlib 安装成功。")
      return True
   except Exception as e:
      raise RuntimeError(
         "matplotlib 不可用，无法生成 PNG 图片。"
         "请在当前 pyfluent_v241 环境执行：python -m pip install matplotlib"
         f" | 原因: {e}"
      )


def write_force_png_chart(file_path, xs, ys, x_label, y_label, title):
   """
   用 matplotlib 生成 PNG 图，并在每个数据点旁标注对应的力数值。
   最终工作目录只保存 PNG 图片。

   标注规则：
      y >= 0：数值放在点的右上方；
      y < 0 ：数值放在点的右下方；
   同时增加坐标轴边距，避免文字被图片边缘裁剪。
   """
   ensure_matplotlib_available()
   import matplotlib
   matplotlib.use("Agg")
   import matplotlib.pyplot as plt

   pairs = sorted(zip(xs, ys), key=lambda item: item[0])
   xs = [p[0] for p in pairs]
   ys = [p[1] for p in pairs]

   fig, ax = plt.subplots(figsize=(8, 5), dpi=150)

   ax.plot(xs, ys, marker="o")

   for x, y in zip(xs, ys):
      try:
         label = f"{float(y):.3f}"
      except Exception:
         label = str(y)

      if y >= 0:
         offset = (7, 7)
         va = "bottom"
      else:
         offset = (7, -7)
         va = "top"

      ax.annotate(
         label,
         xy=(x, y),
         xytext=offset,
         textcoords="offset points",
         ha="left",
         va=va,
         fontsize=9,
      )

   ax.set_xlabel(x_label)
   ax.set_ylabel(y_label)
   ax.set_title(title)
   ax.grid(True)

   # 横坐标直接显示实际速度值，方便对应 Excel。
   try:
      ax.set_xticks(xs)
   except Exception:
      pass

   # 给点旁边的数值留出空间，避免被裁剪。
   try:
      ax.margins(x=0.12, y=0.18)
   except Exception:
      pass

   fig.tight_layout()
   fig.savefig(file_path)
   plt.close(fig)

   print(f"   PNG 图已保存，已标注每个数据点数值: {file_path}")
   return file_path

def write_force_vs_velocity_charts(all_force_rows):
   """
   根据 Excel 同源数据生成三张力-速度关系 PNG 图：
      force_x_vs_velocity.png
      force_y_vs_velocity.png
      force_z_vs_velocity.png

   每张图的每个点旁边都会标注该点的力值。
   若只有一个速度工况，不作图。
   """
   parsed_rows = []
   for row in all_force_rows:
      v = _to_float_or_none(row.get("velocity"))
      fx = _to_float_or_none(row.get("drag_x"))
      fy = _to_float_or_none(row.get("sway_y"))
      fz = _to_float_or_none(row.get("heave_z"))

      if v is None:
         continue

      parsed_rows.append((v, fx, fy, fz))

   parsed_rows = sorted(parsed_rows, key=lambda x: x[0])

   unique_speeds = sorted(set([r[0] for r in parsed_rows]))
   if len(unique_speeds) <= 1:
      print("   速度工况数量 <= 1，跳过力-速度关系 PNG 图生成。")
      return []

   chart_specs = [
      ("force_x_vs_velocity.png", "Force_X (N)", "X Direction Force vs Velocity", 1),
      ("force_y_vs_velocity.png", "Force_Y (N)", "Y Direction Force vs Velocity", 2),
      ("force_z_vs_velocity.png", "Force_Z (N)", "Z Direction Force vs Velocity", 3),
   ]

   outputs = []

   for file_name, y_label, title, idx in chart_specs:
      xs = []
      ys = []
      for row in parsed_rows:
         v = row[0]
         y = row[idx]
         if y is None:
            continue
         xs.append(v)
         ys.append(y)

      if len(xs) <= 1:
         print(f"   {file_name} 可用数据点 <= 1，跳过该图。")
         continue

      png_path = os.path.join(WORK_DIR, file_name)
      write_force_png_chart(
         file_path=png_path,
         xs=xs,
         ys=ys,
         x_label="Velocity (m/s)",
         y_label=y_label,
         title=title,
      )
      outputs.append(png_path)

   return outputs




def _patch_graphics_state_dict(obj):
   """
   递归修改 settings state 中和 grid/ground/floor/reflection 有关的键。
   """
   changed = False

   def norm_key(k):
      return str(k).lower().replace("-", "").replace("_", "").replace(" ", "").replace("?", "")

   if isinstance(obj, dict):
      out = {}
      for k, v in obj.items():
         nk = norm_key(k)
         is_plane_key = (
            ("grid" in nk and "plane" in nk)
            or ("ground" in nk and "plane" in nk)
            or ("floor" in nk and "plane" in nk)
         )
         is_reflection_key = "reflection" in nk

         if is_plane_key:
            out[k] = False
            changed = True
         elif is_reflection_key:
            out[k] = True
            changed = True
         else:
            patched_v, ch = _patch_graphics_state_dict(v)
            out[k] = patched_v
            changed = changed or ch
      return out, changed

   if isinstance(obj, list):
      new_list = []
      for item in obj:
         patched_item, ch = _patch_graphics_state_dict(item)
         new_list.append(patched_item)
         changed = changed or ch
      return new_list, changed

   return obj, False


def _try_patch_settings_object_state(obj, label):
   try:
      state = obj.get_state()
   except Exception:
      return False

   patched, changed = _patch_graphics_state_dict(state)
   if not changed:
      return False

   try:
      obj.set_state(patched)
      print(f"   已通过 settings state 修改图形选项: {label}")
      return True
   except Exception:
      try:
         obj.set_state(patched, "replace")
         print(f"   已通过 settings state replace 修改图形选项: {label}")
         return True
      except Exception:
         return False


def apply_xoy_picture_graphics_options(solver_session):
   """
   保存 xoy 云图前设置：
   1. Disable Graphics Grid Plane
   2. Enable Graphics Reflections

   防呆说明：
   Fluent 不同版本中该 GUI 选项可能实际叫 grid-plane、ground-plane 或 floor-plane。
   本函数使用 settings state 递归 patch + Scheme/RP 变量 + TUI 多命令三重兜底。
   如果 Fluent 版本仍未响应，终端会打印哪些 settings state 被成功修改。
   """
   print("   ▶ 设置 xoy 图片图形选项：Disable Graphics Grid Plane，Enable Graphics Reflections")
   force_graphics_window_ready(solver_session)

   settings_roots = [
      ("settings.results.graphics", lambda s: s.settings.results.graphics),
      ("settings.results.graphics.views", lambda s: s.settings.results.graphics.views),
      ("settings.results.graphics.picture", lambda s: s.settings.results.graphics.picture),
      ("settings.results.graphics.contour", lambda s: s.settings.results.graphics.contour),
      ("settings.preferences", lambda s: s.settings.preferences),
      ("settings.preferences.graphics", lambda s: s.settings.preferences.graphics),
   ]
   for label, getter in settings_roots:
      try:
         _try_patch_settings_object_state(getter(solver_session), label)
      except Exception:
         pass

   scheme_cmds = [
      "(rpsetvar 'graphics/grid-plane? #f)",
      "(rpsetvar 'graphics/show-grid-plane? #f)",
      "(rpsetvar 'graphics/graphics-grid-plane? #f)",
      "(rpsetvar 'graphics/ground-plane? #f)",
      "(rpsetvar 'graphics/show-ground-plane? #f)",
      "(rpsetvar 'graphics/floor-plane? #f)",
      "(rpsetvar 'display/grid-plane? #f)",
      "(rpsetvar 'display/show-grid-plane? #f)",
      "(rpsetvar 'display/ground-plane? #f)",
      "(rpsetvar 'display/show-ground-plane? #f)",
      "(rpsetvar 'rendering/grid-plane? #f)",
      "(rpsetvar 'rendering/ground-plane? #f)",
      "(rpsetvar 'rendering/floor-plane? #f)",
      "(rpsetvar 'graphics/reflections? #t)",
      "(rpsetvar 'graphics/graphics-reflections? #t)",
      "(rpsetvar 'graphics/enable-reflections? #t)",
      "(rpsetvar 'display/reflections? #t)",
      "(rpsetvar 'display/graphics-reflections? #t)",
      "(rpsetvar 'rendering/reflections? #t)",
      "(rpsetvar 'rendering/enable-reflections? #t)",
   ]
   for s in scheme_cmds:
      try:
         solver_session.scheme_eval.eval(s)
      except Exception:
         pass

   cmds = [
      "/display/set/rendering-options/grid-plane? no",
      "/display/set/rendering-options/show-grid-plane? no",
      "/display/set/rendering-options/graphics-grid-plane? no",
      "/display/set/rendering-options/ground-plane? no",
      "/display/set/rendering-options/show-ground-plane? no",
      "/display/set/rendering-options/floor-plane? no",
      "/display/set/display-options/grid-plane? no",
      "/display/set/display-options/ground-plane? no",
      "/display/set/display-options/floor-plane? no",
      "/display/set/grid-plane no",
      "/display/set/ground-plane no",
      "/display/set/floor-plane no",
      "/display/grid-plane no",
      "/display/ground-plane no",
      "/display/floor-plane no",
      "/display/set/rendering-options/reflections? yes",
      "/display/set/rendering-options/graphics-reflections? yes",
      "/display/set/rendering-options/enable-reflections? yes",
      "/display/set/rendering-options/reflections yes",
      "/display/set/rendering-options/graphics-reflections yes",
      "/display/set/display-options/reflections? yes",
      "/display/set/display-options/graphics-reflections? yes",
      "/display/set/reflections yes",
      "/display/set/graphics-reflections yes",
      "/display/reflections yes",
      "/display/graphics-reflections yes",
      "/display/update-scene",
      "/display/re-render",
   ]
   for cmd in cmds:
      solver_tui(solver_session, cmd)
   time.sleep(1.0)


def save_contour_pictures(solver_session, velocity_tag=None):
   """
   保存 xoy 速度云图和 xoy 静压云图。
   多速度计算时，图片名自动带速度标签，例如：
   xoy_v2ms.png
   xoy_static_pressure_v2ms.png
   Fluent 中保留 xoy_vel、xoz_vel、xoy_static_pressure 三个云图对象。
   """
   if not SAVE_CONTOUR_IMAGES:
      print("   跳过云图图片保存：SAVE_CONTOUR_IMAGES=False")
      return

   if not os.path.exists(WORK_DIR):
      os.makedirs(WORK_DIR)

   for spec in CONTOUR_IMAGE_SPECS:
      contour_name = spec["contour"]
      base_file = spec["file"]

      if velocity_tag:
         root, ext = os.path.splitext(base_file)
         file_name = f"{root}_{velocity_tag}{ext}"
      else:
         file_name = base_file

      contour_def = None
      for c in CONTOURS_TO_CREATE:
         if c["name"] == contour_name:
            contour_def = c
            break
      if contour_def is None:
         print(f"   未找到云图定义: {contour_name}")
         continue

      save_one_contour_picture(
         solver_session=solver_session,
         contour_name=contour_name,
         field_name=contour_def["field"],
         surface_name=contour_def["surface"],
         file_name=file_name,
         view_key=spec["view"],
      )

   for c in CONTOURS_TO_CREATE:
      try:
         create_or_update_contour_for_picture(
            solver_session=solver_session,
            contour_name=c["name"],
            field_name=c["field"],
            surface_name=c["surface"],
         )
      except Exception as e:
         print(f"   云图对象确认失败: {c['name']} | {e}")


def _container_has_key(container, name):
   try:
      return name in container.keys()
   except Exception:
      try:
         return name in list(container)
      except Exception:
         return False


def _safe_pop_named(container, name):
   try:
      if _container_has_key(container, name):
         container.pop(name)
         return True
   except Exception:
      pass
   return False


def _try_set_state(obj, state):
   try:
      obj.set_state(state)
      return True
   except Exception:
      pass
   try:
      obj.set_state(state, "replace")
      return True
   except Exception:
      pass
   return False


def _try_set_attrs(obj, state):
   ok_any = False
   for k, v in state.items():
      for kk in [k, k.replace("-", "_"), k.replace("_", "-")]:
         try:
            setattr(obj, kk, v)
            ok_any = True
            break
         except Exception:
            pass
   return ok_any


def resolve_force_report_zones_for_solver(solver_session, old_report_zones):
   zones = list(old_report_zones) if old_report_zones else []
   if zones:
      return zones
   try:
      current_wall_zones = get_bc_names(solver_session, "wall")
   except Exception:
      current_wall_zones = []
   exclude_keywords = ["inlet", "outlet", "symmetry", "fluid", "domain"]
   fallback = [z for z in current_wall_zones if not any(k in z.lower() for k in exclude_keywords)]
   if fallback:
      print(f"   force report zones 为空，改用当前 solver wall zones 兜底: {fallback}")
      return fallback
   return current_wall_zones


def create_report_definition_settings_api(solver_session, name, vector, report_zones):
   vx, vy, vz = vector
   roots = []
   for root_name in ["solution", "settings.solution"]:
      try:
         root = solver_session
         for attr in root_name.split("."):
            root = getattr(root, attr)
         roots.append((root_name, root))
      except Exception:
         pass

   state_candidates = [
      {"thread_names": report_zones, "force_vector": [vx, vy, vz], "scaled": False},
      {"thread-names": report_zones, "force-vector": [vx, vy, vz], "scaled?": False},
      {"zone_names": report_zones, "force_vector": [vx, vy, vz], "scaled": False},
      {"zone-names": report_zones, "force-vector": [vx, vy, vz], "scaled?": False},
      {"zones": report_zones, "force_vector": [vx, vy, vz], "scaled": False},
   ]

   for root_name, root in roots:
      try:
         report_definitions = root.report_definitions
      except Exception:
         continue
      for container_name in ["drag", "force", "lift", "surface", "surface_report", "surface_reports"]:
         try:
            container = getattr(report_definitions, container_name)
         except Exception:
            continue
         _safe_pop_named(container, name)
         try:
            container[name] = {}
         except Exception:
            try:
               container.create(name)
            except Exception:
               continue
         try:
            obj = container[name]
         except Exception:
            continue
         ok = False
         for st in state_candidates:
            if _try_set_state(obj, st) or _try_set_attrs(obj, st):
               ok = True
               break
         for attr_name in ["thread_names", "zone_names", "zones"]:
            try:
               setattr(obj, attr_name, report_zones)
               ok = True
            except Exception:
               pass
         try:
            setattr(obj, "force_vector", [vx, vy, vz])
            ok = True
         except Exception:
            pass
         try:
            setattr(obj, "scaled", False)
         except Exception:
            pass
         if ok:
            print(f"   settings API 已创建 report definition: {name} | {root_name}.{container_name}")
            return True
   return False


def create_report_file_settings_api(solver_session, name, report_file):
   roots = []
   for root_name in ["solution", "settings.solution"]:
      try:
         root = solver_session
         for attr in root_name.split("."):
            root = getattr(root, attr)
         roots.append((root_name, root))
      except Exception:
         pass

   state_candidates = [
      {"report_defs": [name], "file_name": report_file, "frequency": 1},
      {"report-defs": [name], "file-name": report_file, "frequency": 1},
   ]

   for root_name, root in roots:
      try:
         report_files_container = root.monitor.report_files
      except Exception:
         continue
      _safe_pop_named(report_files_container, name)
      try:
         report_files_container[name] = {}
      except Exception:
         try:
            report_files_container.create(name)
         except Exception:
            continue
      try:
         obj = report_files_container[name]
      except Exception:
         continue
      ok = False
      for st in state_candidates:
         if _try_set_state(obj, st) or _try_set_attrs(obj, st):
            ok = True
            break
      try:
         setattr(obj, "report_defs", [name])
         ok = True
      except Exception:
         pass
      try:
         setattr(obj, "file_name", report_file)
         ok = True
      except Exception:
         pass
      try:
         setattr(obj, "frequency", 1)
      except Exception:
         pass
      if ok:
         print(f"   settings API 已创建 report file: {name} -> {report_file}")
         return True
   return False


def create_force_reports_tui_journal(solver_session, report_zones, report_files, report_configs):
   if not report_zones:
      return []
   zone_expr = " ".join(report_zones)
   cmds = []
   for cfg in report_configs:
      name = cfg["name"]
      cmds.extend([f"/solve/report-files/delete {name}", f"/solve/report-definitions/delete {name}"])
   for cfg in report_configs:
      name = cfg["name"]
      vx, vy, vz = cfg["vector"]
      report_file = report_files[name]
      cmds.append(f'/solve/report-definitions/add {name} drag thread-names {zone_expr} () force-vector {vx} {vy} {vz} scaled? no quit')
      cmds.append(f'/solve/report-files/add {name} report-defs {name} () file-name "{report_file}" frequency 1 quit')
   try:
      jou = write_and_read_journal(solver_session, cmds, "setup_force_xyz_reports.jou")
      print(f"   已执行三方向力报告 journal: {jou}")
   except Exception as e:
      print(f"   三方向力报告 journal 执行失败: {e}")
   return [cfg["name"] for cfg in report_configs]


def create_force_reports(solver_session, report_zones, report_files):
   report_zones = resolve_force_report_zones_for_solver(solver_session, report_zones)
   if not report_zones:
      print("   没有找到 AUV 表面 wall zone，三个方向的力报告暂不创建。")
      return []
   for p in report_files.values():
      try:
         local_p = str(p).replace("/", os.sep)
         if os.path.exists(local_p):
            os.remove(local_p)
      except Exception:
         pass
   report_configs = [
      {"name": REPORT_NAME, "vector": (1, 0, 0), "cn": "阻力_X"},
      {"name": REPORT_SWAY_Y_NAME, "vector": (0, 1, 0), "cn": "侧向力_Y"},
      {"name": REPORT_HEAVE_Z_NAME, "vector": (0, 0, 1), "cn": "垂向力_Z"},
   ]
   created = []
   for cfg in report_configs:
      name = cfg["name"]
      print(f"   ▶ 创建 {cfg['cn']} 报告: {name} | vector={cfg['vector']} | zones={report_zones}")
      ok_def = create_report_definition_settings_api(solver_session, name, cfg["vector"], report_zones)
      ok_file = create_report_file_settings_api(solver_session, name, report_files[name]) if ok_def else False
      if ok_def and ok_file:
         created.append(name)
      else:
         print(f"   settings API 未能完整创建 {name}，后续使用 TUI journal 兜底。")
   tui_created = create_force_reports_tui_journal(solver_session, report_zones, report_files, report_configs)
   for name in tui_created:
      if name not in created:
         created.append(name)
   print(f"   ▶ 三方向力报告创建结果: {created}")
   return created



def _unique_keep_order(items):
   out = []
   for x in items:
      if x is None:
         continue
      s = str(x).strip()
      if not s:
         continue
      if s not in out:
         out.append(s)
   return out


def collect_inlet_zones_for_velocity(solver_session, inlet_zones):
   """
   更稳地识别入口 zone。
   先用传入的 inlet_zones；
   再从当前 Fluent 的 velocity-inlet / pressure-outlet / wall / symmetry 里按 inlet 名称兜底查找。
   """
   candidates = []

   if inlet_zones:
      candidates.extend(list(inlet_zones))

   all_bc = []
   for bc_type in ["velocity_inlet", "pressure_outlet", "wall", "symmetry"]:
      try:
         all_bc.extend(get_bc_names(solver_session, bc_type))
      except Exception:
         pass

   try:
      named_inlets = find_zones_by_labels(all_bc, ["inlet"])
      candidates.extend(named_inlets)
   except Exception:
      pass

   try:
      current_velocity_inlets = get_bc_names(solver_session, "velocity_inlet")
      # 优先选择名字里有 inlet 的 velocity-inlet；没有就全部加入兜底。
      named = [z for z in current_velocity_inlets if "inlet" in str(z).lower()]
      candidates.extend(named if named else current_velocity_inlets)
   except Exception:
      pass

   candidates = _unique_keep_order(candidates)

   print(f"   ▶ 当前用于设置速度的 inlet zones = {candidates}")
   return candidates


def _patch_velocity_state_object(obj, velocity):
   """
   递归 patch velocity-inlet 的 state：
   重点处理 vmag / velocity-magnitude / momentum.velocity_magnitude 等字段。
   """
   changed = False

   def norm_key(k):
      return str(k).lower().replace("-", "").replace("_", "").replace(" ", "").replace("?", "")

   def is_velocity_magnitude_key(k):
      nk = norm_key(k)
      if nk in ["vmag", "velocitymagnitude", "velocitymagnitudes", "speed"]:
         return True
      if "velocity" in nk and "magnitude" in nk:
         return True
      if "vmag" in nk:
         return True
      return False

   def is_velocity_method_key(k):
      nk = norm_key(k)
      return "velocityspecificationmethod" in nk or nk == "velocitymethod"

   def is_direction_method_key(k):
      nk = norm_key(k)
      return "directionspecificationmethod" in nk or nk == "directionmethod"

   def patch_value(v, parent_key=""):
      nonlocal changed

      if isinstance(v, dict):
         out = {}
         parent_is_vmag = is_velocity_magnitude_key(parent_key)

         for k, sub in v.items():
            nk = norm_key(k)

            if parent_is_vmag and nk in ["value", "constant", "constantvalue", "magnitude"]:
               out[k] = float(velocity)
               changed = True
            elif is_velocity_magnitude_key(k):
               if isinstance(sub, dict):
                  sub_out = dict(sub)
                  wrote = False
                  for kk in list(sub_out.keys()):
                     nkk = norm_key(kk)
                     if nkk in ["value", "constant", "constantvalue", "magnitude"]:
                        sub_out[kk] = float(velocity)
                        wrote = True
                  if not wrote:
                     sub_out["value"] = float(velocity)
                  out[k] = sub_out
               else:
                  out[k] = float(velocity)
               changed = True
            elif is_velocity_method_key(k):
               # 让 Fluent 使用速度大小方式，而不是保留某些默认 0 值方式。
               out[k] = "Magnitude and Direction"
               changed = True
            elif is_direction_method_key(k):
               # AUV 外流场入口一般使用法向入口，保持和 GUI 常规设置一致。
               out[k] = "Normal to Boundary"
               changed = True
            else:
               out[k] = patch_value(sub, k)

         return out

      if isinstance(v, list):
         return [patch_value(x, parent_key) for x in v]

      if isinstance(v, tuple):
         return tuple(patch_value(x, parent_key) for x in v)

      return v

   patched = patch_value(obj)
   return patched, changed


def _try_set_velocity_state_on_bc(bc_obj, zone_name, velocity, label):
   """
   对一个 velocity-inlet BC 对象进行 state patch。
   """
   ok = False

   try:
      state = bc_obj.get_state()
      patched, changed = _patch_velocity_state_object(state, velocity)
      if changed:
         try:
            bc_obj.set_state(patched)
            print(f"   {label} state 设置入口速度成功: {zone_name} = {velocity} m/s")
            ok = True
         except Exception:
            try:
               bc_obj.set_state(patched, "replace")
               print(f"   {label} state replace 设置入口速度成功: {zone_name} = {velocity} m/s")
               ok = True
            except Exception as e:
               print(f"   {label} state 设置失败: {zone_name} | {e}")
   except Exception:
      pass

   # 直接属性兜底。不同 PyFluent 版本字段名不一样，所以多路径尝试。
   attr_paths = [
      ["momentum", "velocity_magnitude", "value"],
      ["momentum", "velocity_magnitude", "constant"],
      ["momentum", "velocity_magnitude"],
      ["momentum", "vmag", "value"],
      ["momentum", "vmag"],
      ["velocity_magnitude", "value"],
      ["velocity_magnitude"],
      ["vmag", "value"],
      ["vmag"],
   ]

   for path in attr_paths:
      try:
         obj = bc_obj
         for attr in path[:-1]:
            obj = getattr(obj, attr)
         setattr(obj, path[-1], float(velocity))
         print(f"   {label} 属性设置入口速度成功: {zone_name}.{'.'.join(path)} = {velocity}")
         ok = True
      except Exception:
         pass

   return ok


def set_inlet_velocity_api(solver_session, inlet_zones, velocity):
   """
   使用 PyFluent API/settings 尝试设置入口速度。
   """
   ok_any = False

   for z in inlet_zones:
      # setup.boundary_conditions
      for root_label, root_getter in [
         ("setup", lambda s: s.setup.boundary_conditions),
         ("settings.setup", lambda s: s.settings.setup.boundary_conditions),
      ]:
         try:
            root = root_getter(solver_session)
         except Exception:
            continue

         for container_name in ["velocity_inlet", "velocity-inlet"]:
            try:
               container = getattr(root, container_name)
            except Exception:
               continue

            try:
               bc_obj = container[z]
            except Exception:
               try:
                  bc_obj = container[str(z)]
               except Exception:
                  continue

            if _try_set_velocity_state_on_bc(bc_obj, z, velocity, root_label + "." + container_name):
               ok_any = True

   return ok_any



def set_inlet_velocity_journal(solver_session, inlet_zones, velocity):
   """
   使用 Fluent journal/TUI 设置入口速度。

   重要修正：
   只保留 Fluent 2024R1 已验证可识别的 vmag 写法。
   不再使用 velocity-magnitude / velocity-magnitude constant 这类命令，
   因为 Fluent 报错：
      Error: invalid command
      Error Object: "velocity-magnitude"
   说明该版本的 velocity-inlet TUI 菜单里没有这个命令名。
   """
   if not inlet_zones:
      print("   没有入口 zone，无法通过 journal 设置速度。")
      return False

   cmds = []
   for z in inlet_zones:
      # 先确保 zone 类型是 velocity-inlet
      cmds.append(f'/define/boundary-conditions/modify-zones/zone-type {z} velocity-inlet')

      # 保留 v12/v15 里更接近 Fluent 2024R1 的写法
      cmds.append(f'/define/boundary-conditions/set/velocity-inlet {z} () vmag no {float(velocity)} quit')

   try:
      jou = write_and_read_journal(solver_session, cmds, f"set_inlet_velocity_{speed_tag(velocity)}.jou")
      print(f"   已通过 journal 设置入口速度: U = {velocity} m/s | {jou}")
      return True
   except Exception as e:
      print(f"   journal 设置入口速度失败: {e}")

   # 单条 TUI 兜底：即使 journal 中断，也逐条执行，不再包含非法 velocity-magnitude 命令。
   ok_any = False
   for cmd in cmds:
      try:
         if solver_tui(solver_session, cmd):
            ok_any = True
      except Exception:
         pass

   if ok_any:
      print(f"   已通过单条 TUI 兜底设置入口速度: U = {velocity} m/s")

   return ok_any


def print_velocity_inlet_state_for_debug(solver_session, inlet_zones, velocity):
   """
   打印入口速度相关 state，便于确认是不是仍为 0。
   """
   for z in inlet_zones:
      for root_label, root_getter in [
         ("setup", lambda s: s.setup.boundary_conditions),
         ("settings.setup", lambda s: s.settings.setup.boundary_conditions),
      ]:
         try:
            root = root_getter(solver_session)
            container = root.velocity_inlet
            bc_obj = container[z]
            state = bc_obj.get_state()
            print(f"   {root_label}.velocity_inlet[{z}] state after setting U={velocity}:")
            print(state)
            return
         except Exception:
            pass


def set_inlet_velocity_for_case(solver_session, inlet_zones, velocity):
   """
   多速度循环中的入口速度设置函数。

   关键修正：
   - v15 虽然进入了多速度循环，但入口速度没有稳定写入 Fluent BC；
   - 这一版同时使用 journal/TUI 和 PyFluent settings API；
   - 设置后会打印 velocity-inlet state，方便确认 vmag / velocity-magnitude 是否已经不是 0。
   """
   inlet_zones_current = collect_inlet_zones_for_velocity(solver_session, inlet_zones)

   if not inlet_zones_current:
      print("   没有识别到入口 zone，当前速度没有设置成功。")
      return False

   print(f"   ▶ 开始设置当前入口速度: U = {velocity} m/s")

   # 先用 journal，等价于原来 v12 中能正常工作的方式。
   ok_journal = set_inlet_velocity_journal(solver_session, inlet_zones_current, velocity)

   # 再用 API/settings patch，防止 TUI 命令没有真正写入 state。
   ok_api = set_inlet_velocity_api(solver_session, inlet_zones_current, velocity)

   # 再跑一次 journal，确保 API 修改后 Fluent 内部 BC 面板同步。
   ok_journal_2 = set_inlet_velocity_journal(solver_session, inlet_zones_current, velocity)

   print_velocity_inlet_state_for_debug(solver_session, inlet_zones_current, velocity)

   ok = bool(ok_journal or ok_api or ok_journal_2)
   if ok:
      print(f"   当前速度工况入口速度已写入: U = {velocity} m/s")
   else:
      print(f"   当前速度工况入口速度写入失败: U = {velocity} m/s")

   return ok




def solver_server_is_alive(solver_session):
   """
   检查 Fluent Solver 是否仍能响应。
   若 Solver 已经因为内存/并行错误断开，则后续云图、导出、报告都应停止，避免连续报错。
   """
   try:
      solver_session.scheme_eval.eval("(+ 1 1)")
      return True
   except Exception:
      return False



def _patch_residual_state(obj, criteria):
   """
   递归修补 residual monitor state 中六个方程的 absolute criteria。
   """
   eq_names = ["continuity", "x-velocity", "y-velocity", "z-velocity", "k", "omega"]
   eq_norms = [x.replace("-", "").replace("_", "").lower() for x in eq_names]

   def norm_key(k):
      return str(k).lower().replace(" ", "").replace("_", "").replace("-", "").replace("?", "")

   if isinstance(obj, dict):
      out = {}
      for key, value in obj.items():
         nk = norm_key(key)

         if nk in eq_norms and isinstance(value, dict):
            sub = dict(value)
            # 常见字段名全部写一遍；Fluent 会忽略不存在字段或由 set_state 失败兜底。
            for ckey in [
               "absolute_criteria",
               "absolute-criteria",
               "absolute criteria",
               "AbsoluteCriteria",
               "criteria",
               "convergence_criteria",
               "convergence-criteria",
               "Convergence Criteria",
            ]:
               sub[ckey] = float(criteria)
            sub["check_convergence"] = True
            sub["check-convergence"] = True
            out[key] = _patch_residual_state(sub, criteria)
         elif "absolute" in nk and ("criteria" in nk or "criterion" in nk):
            out[key] = float(criteria)
         elif "convergence" in nk and ("criteria" in nk or "criterion" in nk):
            out[key] = float(criteria)
         else:
            out[key] = _patch_residual_state(value, criteria)

      return out

   if isinstance(obj, list):
      return [_patch_residual_state(x, criteria) for x in obj]

   return obj


def set_residual_convergence_criteria(solver_session):
   """
   设置 求解 -> 计算监控 -> 残差：
      continuity
      x-velocity
      y-velocity
      z-velocity
      k
      omega
   六个变量的绝对收敛标准统一为 RESIDUAL_ABS_CRITERIA。

   只保留已验证成功的交互式 journal 写法，避免出现：
      ERROR: Please answer y[es] or n[o].
   """
   criteria = float(RESIDUAL_ABS_CRITERIA)

   print("   ▶ 设置残差收敛标准")
   print(f"     continuity / x-velocity / y-velocity / z-velocity / k / omega = {criteria:.1e}")

   cmds = [
      "/solve/monitors/residual/convergence-criteria",
      str(criteria),
      str(criteria),
      str(criteria),
      str(criteria),
      str(criteria),
      str(criteria),
   ]

   try:
      jou = write_and_read_journal(solver_session, cmds, "set_residual_criteria.jou")
      print(f"   残差收敛标准已通过交互式 journal 设置: {jou}")
      return True
   except Exception as e:
      print(f"   残差收敛标准设置失败: {e}")

   return False



def initialize_and_run_case(solver_session, inlet_zones, iterations):
   try:
      velocity_inlet_zones_now = get_bc_names(solver_session, "velocity_inlet")
      inlet_for_init = find_zones_by_labels(velocity_inlet_zones_now, ["inlet"])
      actual_inlet_zone = inlet_for_init[0] if inlet_for_init else (velocity_inlet_zones_now[0] if velocity_inlet_zones_now else "inlet")
      solver_session.tui.solve.initialize.compute_defaults.velocity_inlet(actual_inlet_zone)
      solver_session.tui.solve.initialize.initialize_flow()
      print(f"   已从入口 {actual_inlet_zone} 初始化。")
   except Exception as e:
      print(f"   标准初始化失败，改用 hybrid initialize: {e}")
      try:
         solver_session.solution.initialization.hybrid_initialize()
      except Exception:
         try:
            solver_session.tui.solve.initialize.hyb_initialization()
         except Exception as e2:
            print(f"   hybrid 初始化也失败: {e2}")
            return False

   try:
      solver_session.solution.run_calculation.iterate(iter_count=iterations)
   except Exception as e1:
      print(f"   settings API 迭代失败: {e1}")
      try:
         solver_session.tui.solve.iterate(iterations)
      except Exception as e2:
         print(f"   TUI 迭代失败: {e2}")
         return False

   if not solver_server_is_alive(solver_session):
      print("   Fluent Solver 已无响应，停止当前速度后处理，避免保存残差图或继续报错。")
      return False

   return True




# ========================================================
# 6.1 v14 修正：报告值即时计算与多速度顺序修正
# ========================================================

def _parse_numeric_from_any(obj):
   """从 PyFluent compute 返回值或字符串中提取最后一个数值。"""
   import re
   if obj is None:
      return None
   if isinstance(obj, (int, float)):
      return float(obj)
   if isinstance(obj, dict):
      nums = []
      for v in obj.values():
         x = _parse_numeric_from_any(v)
         if x is not None:
            nums.append(x)
      return nums[-1] if nums else None
   if isinstance(obj, (list, tuple)):
      nums = []
      for v in obj:
         x = _parse_numeric_from_any(v)
         if x is not None:
            nums.append(x)
      return nums[-1] if nums else None
   s = str(obj)
   vals = [float(x) for x in re.findall(r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?", s)]
   return vals[-1] if vals else None


def parse_last_numeric_from_text_file(file_path):
   import re
   if not file_path or not os.path.exists(file_path):
      return None
   try:
      with open(file_path, "r", encoding="utf-8", errors="ignore") as f:
         content = f.read()
   except Exception:
      return None

   nums = re.findall(r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?", content)
   if not nums:
      return None
   try:
      return float(nums[-1])
   except Exception:
      return None


def run_tui_report_to_file(solver_session, commands, output_file, transcript_name):
   local_output = str(output_file).replace("/", os.sep)
   if os.path.exists(local_output):
      try:
         os.remove(local_output)
      except Exception:
         pass

   transcript_path = os.path.join(WORK_DIR, transcript_name)
   local_transcript = transcript_path.replace("/", os.sep)
   try:
      if os.path.exists(local_transcript):
         os.remove(local_transcript)
   except Exception:
      pass

   start_cmd = f'/file/start-transcript "{normalize_path_for_fluent(local_transcript)}"'
   stop_cmd = "/file/stop-transcript"

   try:
      solver_tui(solver_session, start_cmd)
      time.sleep(0.3)
      for cmd in commands:
         solver_tui(solver_session, cmd)
         time.sleep(0.3)
   finally:
      solver_tui(solver_session, stop_cmd)
      time.sleep(0.3)

   value = parse_last_numeric_from_text_file(local_transcript)

   with open(local_output, "w", encoding="utf-8") as f:
      f.write("# Auto-generated Fluent scalar report\n")
      f.write(f"value = {value}\n")
      f.write("commands =\n")
      for cmd in commands:
         f.write(cmd + "\n")

   return value



def parse_total_force_vector_from_forces_text(text):
   """
   从 Fluent Results -> Reports -> Forces 的完整矢量表中解析 Net 行。

   Net 行格式示例：
      Net (Px Py Pz) (Vx Vy Vz) (Tx Ty Tz) ...

   返回：
      pressure_vector
      viscous_vector
      total_vector

   这里的 total_vector 才作为 Excel 中 Force_X/Y/Z 的权威来源。
   """
   result = {
      "pressure_vector": None,
      "viscous_vector": None,
      "total_vector": None,
   }

   if not text:
      return result

   pattern = re.compile(
      r"^\s*Net\s+"
      r"\(([^()]*)\)\s+"
      r"\(([^()]*)\)\s+"
      r"\(([^()]*)\)",
      flags=re.M
   )

   matches = pattern.findall(str(text))
   if not matches:
      return result

   def parse_triplet(s):
      nums = re.findall(
         r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?",
         str(s)
      )
      if len(nums) < 3:
         return None
      try:
         return (float(nums[0]), float(nums[1]), float(nums[2]))
      except Exception:
         return None

   # 完整 Forces 矢量表通常只会命中一次；若有多次，取最后一个有效矢量 Net。
   for ptxt, vtxt, ttxt in reversed(matches):
      pv = parse_triplet(ptxt)
      vv = parse_triplet(vtxt)
      tv = parse_triplet(ttxt)
      if pv is not None and vv is not None and tv is not None:
         result["pressure_vector"] = pv
         result["viscous_vector"] = vv
         result["total_vector"] = tv
         return result

   return result


def compute_drag_pressure_friction_from_forces_panel(solver_session, report_zones, report_files, velocity_tag=None):
   """
   自动执行用户手动验证过的操作：
      Results -> Reports -> Forces
      Options = Forces
      Direction Vector = (1, 0, 0)
      Wall Zones = 所有 AUV 表面 zones

   从输出表中读取 Net 行：
      Pressure = 压差阻力
      Viscous  = 摩擦阻力
      Total    = 两者合力

   输出文件：
      force_drag_pressure_v*.out
      force_drag_friction_v*.out
      results_report_forces_x_v*.out    # 原始 Forces 表，便于检查
   """
   results = {
      "drag_pressure": None,
      "drag_friction": None,
      "drag_total_from_forces_panel": None,
      "force_pressure_vector": None,
      "force_viscous_vector": None,
      "force_total_vector": None,
      "forces_raw_file": None,
   }

   report_zones = resolve_force_report_zones_for_solver(solver_session, report_zones)

   if len(report_zones) == 0:
      print("   没有找到 AUV wall zones，压差阻力/摩擦阻力暂不计算。")
      write_scalar_report_out(
         file_path=report_files.get(REPORT_DRAG_PRESSURE_NAME),
         title=REPORT_DRAG_PRESSURE_NAME,
         value=None,
         extra_lines=["source = Results-Report-Forces", "reason = no wall zones"],
      )
      write_scalar_report_out(
         file_path=report_files.get(REPORT_DRAG_FRICTION_NAME),
         title=REPORT_DRAG_FRICTION_NAME,
         value=None,
         extra_lines=["source = Results-Report-Forces", "reason = no wall zones"],
      )
      return results

   if velocity_tag:
      transcript_name = f"results_report_forces_x_{velocity_tag}.trn"
   else:
      transcript_name = "results_report_forces_x.trn"

   print("   ▶ 计算压差阻力和摩擦阻力：Results -> Reports -> Forces")
   print("     Options = Forces")
   print("     Direction Vector = (1, 0, 0)")
   print(f"     Wall Zones = {report_zones}")

   wall_result = run_wall_forces_transcript(
      solver_session=solver_session,
      report_zones=report_zones,
      transcript_name=transcript_name,
   )

   results["drag_pressure"] = wall_result.get("pressure")
   results["drag_friction"] = wall_result.get("viscous")
   results["drag_total_from_forces_panel"] = wall_result.get("total")

   raw_text = wall_result.get("raw", "")
   command_text = wall_result.get("command", "not parsed")

   vector_result = parse_total_force_vector_from_forces_text(raw_text)
   results["force_pressure_vector"] = vector_result.get("pressure_vector")
   results["force_viscous_vector"] = vector_result.get("viscous_vector")
   results["force_total_vector"] = vector_result.get("total_vector")

   if results["force_total_vector"] is not None:
      fx, fy, fz = results["force_total_vector"]
      print(
         "   三方向总力直接取自 Forces 完整矢量 Net 行: "
         f"Fx={fx}, Fy={fy}, Fz={fz}"
      )

   raw_out_file = os.path.join(WORK_DIR, transcript_name).replace(".trn", ".out").replace(chr(92), "/")
   results["forces_raw_file"] = raw_out_file

   write_scalar_report_out(
      file_path=report_files.get(REPORT_DRAG_PRESSURE_NAME),
      title=REPORT_DRAG_PRESSURE_NAME,
      value=results["drag_pressure"],
      extra_lines=[
         "source = Results -> Reports -> Forces",
         "manual_gui_equivalent = Results|Reports|Forces, Options=Forces, Direction Vector=(1 0 0), Wall Zones=all AUV surfaces",
         "component = Pressure column on Net row",
         "physical_meaning = pressure drag / form drag in X direction",
         "direction_vector = 1 0 0",
         f"wall_zones = {report_zones}",
         f"successful_variant = {command_text}",
         f"raw_forces_output_file = {raw_out_file}",
      ],
   )

   write_scalar_report_out(
      file_path=report_files.get(REPORT_DRAG_FRICTION_NAME),
      title=REPORT_DRAG_FRICTION_NAME,
      value=results["drag_friction"],
      extra_lines=[
         "source = Results -> Reports -> Forces",
         "manual_gui_equivalent = Results|Reports|Forces, Options=Forces, Direction Vector=(1 0 0), Wall Zones=all AUV surfaces",
         "component = Viscous column on Net row",
         "physical_meaning = viscous/friction drag in X direction",
         "direction_vector = 1 0 0",
         f"wall_zones = {report_zones}",
         f"successful_variant = {command_text}",
         f"raw_forces_output_file = {raw_out_file}",
      ],
   )

   print(f"   压差阻力 Pressure Drag_X = {results['drag_pressure']}")
   print(f"   摩擦阻力 Viscous/Friction Drag_X = {results['drag_friction']}")
   print(f"   Forces 原始输出文件 = {raw_out_file}")

   return results


def compute_drag_components_and_pressure_reports(solver_session, report_zones, report_files):
   """
   输出压差阻力、摩擦阻力和 xoy 静压。

   Fluent 可以计算压差阻力和摩擦阻力：
      Report > Forces > Wall Forces
   自动化时用 /report/forces/wall-forces 抓取 pressure / viscous / total 的 x 方向分量。
   """
   results = {
      "drag_pressure": None,
      "drag_friction": None,
      "xoy_static_pressure": None,
   }

   report_zones = resolve_force_report_zones_for_solver(solver_session, report_zones)

   wall_result = run_wall_forces_transcript(
      solver_session=solver_session,
      report_zones=report_zones,
      transcript_name="tmp_wall_forces_x.trn",
   )

   results["drag_pressure"] = wall_result.get("pressure")
   results["drag_friction"] = wall_result.get("viscous")
   raw_text = wall_result.get("raw", "")

   write_scalar_report_out(
      file_path=report_files[REPORT_DRAG_PRESSURE_NAME],
      title=REPORT_DRAG_PRESSURE_NAME,
      value=results["drag_pressure"],
      extra_lines=[
         "source = report_forces_wall_forces",
         "component = pressure_force_x",
         f"zones = {report_zones}",
         f"command = {wall_result.get('command', 'not found')}",
         "note = If value is not found, inspect tmp_wall_forces_x.trn for the current Fluent TUI format.",
         raw_text,
      ],
   )

   write_scalar_report_out(
      file_path=report_files[REPORT_DRAG_FRICTION_NAME],
      title=REPORT_DRAG_FRICTION_NAME,
      value=results["drag_friction"],
      extra_lines=[
         "source = report_forces_wall_forces",
         "component = viscous_force_x",
         f"zones = {report_zones}",
         f"command = {wall_result.get('command', 'not found')}",
         "note = If value is not found, inspect tmp_wall_forces_x.trn for the current Fluent TUI format.",
         raw_text,
      ],
   )

   try:
      results["xoy_static_pressure"] = compute_xoy_static_pressure_report(
         solver_session=solver_session,
         output_file=report_files[REPORT_XOY_STATIC_PRESSURE_NAME],
      )
   except Exception:
      results["xoy_static_pressure"] = None

   return results




# ========================================================
# 5.9 最终覆盖区：云图显示、力结果输出与 Excel 防呆
# ========================================================

def write_scalar_report_out(file_path, title, value, extra_lines=None):
   """
   写出统一格式的 .out 标量结果文件。
   即使数值没取得，也写出 not found，避免留下旧文件或 value=None 的误判结果。
   """
   local_path = str(file_path).replace("/", os.sep)
   folder = os.path.dirname(local_path)
   if folder and not os.path.exists(folder):
      os.makedirs(folder)

   if value is None:
      value_text = "not found"
   else:
      value_text = str(value)

   with open(local_path, "w", encoding="utf-8") as f:
      f.write("# AUV Fluent scalar report\n")
      f.write(f"name = {title}\n")
      f.write(f"value = {value_text}\n")
      if extra_lines:
         for line in extra_lines:
            f.write(str(line).rstrip() + "\n")

   return local_path


def normalize_report_number(value):
   if value is None:
      return None
   if isinstance(value, str):
      if value.strip().lower() in ["not found", "none", ""]:
         return None
   try:
      return float(value)
   except Exception:
      return None


def resolve_contour_field_name(solver_session, requested_field):
   """
   Static Pressure 在 Fluent 结果变量里通常是 pressure。
   速度云图保持 velocity-magnitude。
   """
   requested = str(requested_field).strip()

   if requested in ["pressure", "static-pressure", "static_pressure"]:
      candidates = ["pressure", "static-pressure", "static_pressure"]
   else:
      candidates = [requested]

   try:
      info = solver_session.fields.field_info.get_scalar_fields_info()
      if isinstance(info, dict):
         keys = list(info.keys())
         low_map = {str(k).lower(): k for k in keys}
         for item in candidates:
            if item in keys:
               return item
            if item.lower() in low_map:
               return low_map[item.lower()]
   except Exception:
      pass

   return candidates[0]


def safe_set_contour_state_no_delete(solver_session, contour_name, field_name, surface_name):
   """
   不删除云图对象，只创建缺失对象并更新 field。

   注意：不再对已有对象写 surfaces / surfaces_list / surface_names。
   Fluent 2024R1 中这些字段可能处于 inactive 状态，写入会出现：
      api-set-var: the object is not active
      results/graphics/contour/.../surfaces
   """
   field_name = resolve_contour_field_name(solver_session, field_name)
   print(f"   云图对象设置: {contour_name} | field={field_name} | surface={surface_name}")

   try:
      contours = solver_session.settings.results.graphics.contour
   except Exception as e:
      print(f"   无法访问 graphics contour 容器: {e}")
      return False

   try:
      keys = list(contours.keys())
   except Exception:
      keys = []

   if contour_name not in keys:
      # 只在对象不存在时创建；这里用 TUI 创建并给 surface。
      # 已有对象绝不重写 surface，避免 inactive surfaces 报错。
      created = False
      field_for_tui = field_name
      for cmd in [
         f"/display/objects/create contour {contour_name} field {field_for_tui} surfaces-list {surface_name} () quit",
         f"/display/objects/create contour {contour_name} field {field_for_tui} surfaces {surface_name} () quit",
      ]:
         try:
            solver_tui(solver_session, cmd)
            created = True
            print(f"   已通过 TUI 创建云图对象: {contour_name}")
            break
         except Exception:
            pass

      if not created:
         try:
            contours[contour_name] = {"field": field_name}
            created = True
            print(f"   已通过 settings API 创建云图对象: {contour_name}")
         except Exception:
            try:
               contours.create(contour_name)
               created = True
               print(f"   已通过 settings create 创建云图对象: {contour_name}")
            except Exception:
               pass

      if not created:
         print(f"   云图对象创建失败: {contour_name}")
         return False
   else:
      print(f"   云图对象已存在，不删除、不重写 surface: {contour_name}")

   try:
      obj = contours[contour_name]
   except Exception as e:
      print(f"   无法取得云图对象: {contour_name} | {e}")
      return False

   ok = False

   # 只更新 field，不包含任何 surface 字段。
   for st in [
      {"field": field_name},
      {"contours_of": field_name},
      {"contours-of": field_name},
   ]:
      try:
         obj.set_state(st)
         print(f"   已 set_state 云图变量: {contour_name} | {st}")
         ok = True
         break
      except Exception:
         try:
            obj.set_state(st, "replace")
            print(f"   已 set_state replace 云图变量: {contour_name} | {st}")
            ok = True
            break
         except Exception:
            pass

   for attr_name in ["field", "field_name", "field_variable", "contours_of"]:
      try:
         setattr(obj, attr_name, field_name)
         print(f"   已设置 {contour_name}.{attr_name} = {field_name}")
         ok = True
         break
      except Exception:
         pass

   return ok


def display_contour_for_picture(solver_session, contour_name, field_name, surface_name):
   """
   只显示已创建/更新的云图对象，不删除、不重建。
   不使用直接 /display/contours field surface 作为成功判断，避免保存上一张残差图。
   """
   force_graphics_window_ready(solver_session)

   state_ok = safe_set_contour_state_no_delete(
      solver_session=solver_session,
      contour_name=contour_name,
      field_name=field_name,
      surface_name=surface_name,
   )

   displayed = False

   try:
      obj = solver_session.settings.results.graphics.contour[contour_name]
      try:
         obj.display()
         displayed = True
         print(f"   已显示云图对象: {contour_name}")
      except Exception:
         try:
            obj.display(window_id=1)
            displayed = True
            print(f"   已显示云图对象到窗口 1: {contour_name}")
         except Exception as e:
            print(f"   settings API 显示失败: {contour_name} | {e}")
   except Exception as e:
      print(f"   取得云图对象失败: {contour_name} | {e}")

   if not displayed:
      # TUI 只显示对象，不直接显示 field，避免 field 命令无效后保存残差。
      for cmd in [
         f"/display/objects/display {contour_name}",
         f"/display/objects/display {contour_name} quit",
      ]:
         try:
            solver_tui(solver_session, cmd)
            displayed = True
            print(f"   已尝试 TUI 显示云图对象: {contour_name}")
            break
         except Exception:
            pass

   for cmd in [
      "/display/set-window 1",
      "/display/views/auto-scale",
      "/display/update-scene",
      "/display/re-render",
   ]:
      solver_tui(solver_session, cmd)

   # 第一个速度时 Fluent GUI 常停留在残差窗口，这里多等并二次刷新。
   time.sleep(2.0)
   for cmd in [
      "/display/update-scene",
      "/display/re-render",
   ]:
      solver_tui(solver_session, cmd)
   time.sleep(1.0)

   return displayed and state_ok


def save_one_contour_picture(solver_session, contour_name, field_name, surface_name, file_name, view_key):
   image_path = os.path.abspath(os.path.join(WORK_DIR, file_name))

   if not image_path.lower().endswith(".png"):
      image_path += ".png"

   try:
      if os.path.exists(image_path):
         os.remove(image_path)
   except Exception:
      pass

   print(f"   正在显示并保存云图图片: {contour_name} -> {image_path}")

   displayed = display_contour_for_picture(
      solver_session=solver_session,
      contour_name=contour_name,
      field_name=field_name,
      surface_name=surface_name,
   )

   if not displayed:
      print(f"   跳过保存图片：云图对象没有确认显示成功，避免保存残差或上一张图: {contour_name}")
      return False

   set_view_for_contour_image(solver_session, view_key)
   apply_xoy_picture_graphics_options(solver_session)

   # 视角与图形选项设置后再次显示同一个对象，防止窗口回到 residual plot。
   displayed = display_contour_for_picture(
      solver_session=solver_session,
      contour_name=contour_name,
      field_name=field_name,
      surface_name=surface_name,
   )

   if not displayed:
      print(f"   二次显示失败，跳过保存图片: {contour_name}")
      return False

   set_view_for_contour_image(solver_session, view_key)

   for cmd in [
      "/display/set-window 1",
      "/display/update-scene",
      "/display/re-render",
   ]:
      solver_tui(solver_session, cmd)

   time.sleep(2.0)

   ok = try_save_picture_with_api(solver_session, image_path)

   if not ok:
      ok = try_save_picture_with_tui(solver_session, image_path)

   if ok and os.path.exists(image_path):
      print(f"   已保存云图图片: {image_path}")
   else:
      print(f"   未检测到云图图片生成: {image_path}")

   return ok


def _get_scalar_field_data_for_contour(solver_session, field_name, surface_name):
   """
   云图专用标量读取：
   先尝试 Cell/Face Center，再尝试 Node Value。
   不调用 Fluent Graphics，不执行 /display 命令。
   """
   # 先复用现有通用读取函数
   vals = get_scalar_field_data_for_zone(solver_session, field_name, surface_name)
   if vals:
      return vals

   try:
      fd = solver_session.fields.field_data
      surface_candidates = _get_surface_id_candidates(solver_session, surface_name)
   except Exception:
      return []

   for surf in surface_candidates:
      attempts = [
         lambda surf=surf: fd.get_scalar_field_data(
            field_name=field_name, surfaces=[surf], node_value=True
         ),
         lambda surf=surf: fd.get_scalar_field_data(
            field_name=field_name, surfaces=[surf]
         ),
      ]
      for call in attempts:
         try:
            raw = call()
            vals = _flatten_numeric_sequence(raw)
            if vals:
               return vals
         except Exception:
            pass
   return []


def save_contour_pictures(solver_session, velocity_tag=None):
   """
   新工作站稳定 + 美观增强版云图输出。

   设计思路：
   1) 仍然不调用 Fluent GUI /display /camera /update-scene，避免 Graphics 崩溃；
   2) 直接通过 PyFluent field_data 读取已求解完成的截面数据；
   3) 用 matplotlib 统一渲染 xoy / xoz 两个平面的速度与静压云图；
   4) 相比上一版，增加：
      - xoz 速度云图
      - xoz 静压云图
      - 更平滑的色带
      - 统一白底、较清晰的色标和标题
      - 压力场在正负并存时使用对称色标，更直观
   """
   if not SAVE_CONTOUR_IMAGES:
      print("   跳过云图图片保存：SAVE_CONTOUR_IMAGES=False")
      return []

   ensure_matplotlib_available()

   import numpy as np
   import matplotlib
   matplotlib.use("Agg")
   import matplotlib.pyplot as plt
   import matplotlib.tri as mtri

   outputs = []

   def _pretty_title(field_name, surface_name, velocity_tag=None):
      if "velocity" in normalize_field_name_key(field_name):
         base = f"Velocity Magnitude on {surface_name.upper()} Plane"
      else:
         base = f"Static Pressure on {surface_name.upper()} Plane"
      if velocity_tag:
         base += f" ({velocity_tag})"
      return base

   def _pretty_cbar_label(field_name):
      if "velocity" in normalize_field_name_key(field_name):
         return "Velocity Magnitude (m/s)"
      return "Static Pressure (Pa)"

   for spec in CONTOUR_IMAGE_SPECS:
      contour_name = spec["contour"]
      base_file = spec["file"]

      contour_def = None
      for c in CONTOURS_TO_CREATE:
         if c["name"] == contour_name:
            contour_def = c
            break

      if contour_def is None:
         print(f"   未找到云图定义: {contour_name}")
         continue

      field_name = contour_def["field"]
      surface_name = contour_def["surface"]

      if velocity_tag:
         root, ext = os.path.splitext(base_file)
         file_name = f"{root}_{velocity_tag}{ext}"
      else:
         file_name = base_file

      out_path = os.path.join(WORK_DIR, file_name)

      print(
         f"   Python field_data 云图: {contour_name} | "
         f"field={field_name} | surface={surface_name}"
      )

      try:
         coords = get_coordinate_fields_for_zone(solver_session, surface_name)
         values = _get_scalar_field_data_for_contour(
            solver_session, field_name, surface_name
         )

         n = min(len(coords), len(values))
         if n < 3:
            raise RuntimeError(
               f"有效云图数据点不足: coords={len(coords)}, values={len(values)}"
            )

         coords = coords[:n]
         values = values[:n]

         xyz = np.asarray(coords, dtype=float)
         vv = np.asarray(values, dtype=float)

         surf_key = str(surface_name).lower()
         if surf_key == "xoy":
            a = xyz[:, 0]
            b = xyz[:, 1]
            xlabel = "X (m)"
            ylabel = "Y (m)"
         elif surf_key == "xoz":
            a = xyz[:, 0]
            b = xyz[:, 2]
            xlabel = "X (m)"
            ylabel = "Z (m)"
         else:
            a = xyz[:, 0]
            b = xyz[:, 1]
            xlabel = "X (m)"
            ylabel = "Y (m)"

         mask = np.isfinite(a) & np.isfinite(b) & np.isfinite(vv)
         a = a[mask]
         b = b[mask]
         vv = vv[mask]

         if len(vv) < 3:
            raise RuntimeError("去除 NaN/Inf 后有效数据点不足。")

         # 去掉重复平面坐标，避免三角剖分异常。
         rounded = np.column_stack((np.round(a, 10), np.round(b, 10)))
         _, unique_idx = np.unique(rounded, axis=0, return_index=True)
         unique_idx = np.sort(unique_idx)
         a = a[unique_idx]
         b = b[unique_idx]
         vv = vv[unique_idx]

         fig, ax = plt.subplots(
            figsize=(PICTURE_X_RESOLUTION / 180.0,
                     PICTURE_Y_RESOLUTION / 180.0),
            dpi=180,
         )

         ax.set_facecolor("white")
         fig.patch.set_facecolor("white")

         pad_x = 0.03 * max(1e-12, float(np.max(a) - np.min(a)))
         pad_y = 0.03 * max(1e-12, float(np.max(b) - np.min(b)))
         ax.set_xlim(float(np.min(a) - pad_x), float(np.max(a) + pad_x))
         ax.set_ylim(float(np.min(b) - pad_y), float(np.max(b) + pad_y))

         plotted = False
         cmap = "turbo" if "velocity" in normalize_field_name_key(field_name) else "coolwarm"
         try:
            tri = mtri.Triangulation(a, b)
            if "velocity" in normalize_field_name_key(field_name):
               vmin = float(np.nanmin(vv))
               vmax = float(np.nanmax(vv))
               if abs(vmax - vmin) < 1e-15:
                  vmax = vmin + 1e-12
               levels = np.linspace(vmin, vmax, 120)
            else:
               vmin = float(np.nanmin(vv))
               vmax = float(np.nanmax(vv))
               if vmin < 0.0 and vmax > 0.0:
                  vmax_abs = max(abs(vmin), abs(vmax))
                  levels = np.linspace(-vmax_abs, vmax_abs, 120)
               else:
                  if abs(vmax - vmin) < 1e-15:
                     vmax = vmin + 1e-12
                  levels = np.linspace(vmin, vmax, 120)

            contour = ax.tricontourf(tri, vv, levels=levels, cmap=cmap, extend="both")
            try:
               ax.tricontour(tri, vv, levels=12, colors="k", linewidths=0.18, alpha=0.25)
            except Exception:
               pass
            cbar = fig.colorbar(contour, ax=ax, pad=0.02, fraction=0.04)
            plotted = True
         except Exception as e:
            print(f"   tricontourf 失败，改用 scatter 兜底: {e}")

         if not plotted:
            sc = ax.scatter(a, b, c=vv, s=3, cmap=cmap)
            cbar = fig.colorbar(sc, ax=ax, pad=0.02, fraction=0.04)

         cbar.set_label(_pretty_cbar_label(field_name), fontsize=10)
         cbar.ax.tick_params(labelsize=9)
         ax.set_title(_pretty_title(field_name, surface_name, velocity_tag), fontsize=12, pad=10)
         ax.set_xlabel(xlabel, fontsize=10)
         ax.set_ylabel(ylabel, fontsize=10)
         ax.tick_params(labelsize=9)
         ax.set_aspect("equal", adjustable="box")
         ax.grid(False)

         # 让版式更像正式后处理图片。
         for spine in ax.spines.values():
            spine.set_linewidth(0.8)
         fig.tight_layout()
         fig.savefig(out_path, dpi=180, bbox_inches="tight", facecolor="white")
         plt.close(fig)

         if os.path.exists(out_path) and os.path.getsize(out_path) > 0:
            print(f"   云图 PNG 已保存: {out_path}")
            outputs.append(out_path)
         else:
            print(f"   云图 PNG 未生成: {out_path}")

      except Exception as e:
         print(f"   云图生成失败: {contour_name} | {e}")

   return outputs


def extract_force_value_robust(solver_session, report_name, report_file):
   """
   计算 report definition 当前值，并手动写出 .out。
   不再依赖 Fluent 自动 report-file 落盘。
   """
   val = compute_report_value_now(solver_session, report_name)

   if val is None:
      val = extract_latest_force_value(report_file)

   write_scalar_report_out(
      file_path=report_file,
      title=report_name,
      value=val,
      extra_lines=["source = report_definition_compute"],
   )

   return val



# ========================================================
# 5.95 最终覆盖区：表面静压 Surface Report
# ========================================================

def create_surface_pressure_reports_tui_journal(solver_session, report_zones, report_files):
   """
   创建 Fluent GUI 中可见的 Surface Report Definitions。

   对应：
      Report Definitions -> New -> Surface Report
      Vertex Average / Vertex Maximum
      Field Variable: Pressure - Static Pressure
      Surfaces: auv_body / shaft / propeller / ...

   Fluent 2024 R1 的实际 report type：
      surface-vertexavg
      surface-vertexmax
   """
   report_zones = resolve_force_report_zones_for_solver(solver_session, report_zones)
   if len(report_zones) == 0:
      print("   没有找到 AUV 表面 zone，表面静压报告暂不创建。")
      return []

   zone_expr = " ".join(report_zones)

   configs = [
      (REPORT_SURFACE_PRESSURE_AVG_NAME, "surface-vertexavg", "表面静压 Vertex Average"),
      (REPORT_SURFACE_PRESSURE_MAX_NAME, "surface-vertexmax", "表面静压 Vertex Maximum"),
   ]

   cmds = []

   for name, report_type, cn in configs:
      report_file = report_files[name]
      cmds.extend([
         f"/solve/report-files/delete {name}",
         f"/solve/report-definitions/delete {name}",
         f'/solve/report-definitions/add {name} {report_type} field pressure surface-names {zone_expr} () quit',
         f'/solve/report-files/add {name} report-defs {name} () file-name "{report_file}" frequency 1 quit',
      ])

   try:
      jou = write_and_read_journal(solver_session, cmds, "setup_surface_pressure_reports.jou")
      print(f"   已执行表面静压报告 journal: {jou}")
   except Exception as e:
      print(f"   表面静压报告 journal 执行失败: {e}")

   created = [REPORT_SURFACE_PRESSURE_AVG_NAME, REPORT_SURFACE_PRESSURE_MAX_NAME]
   print(f"   表面静压报告创建结果: {created}")
   return created


def create_surface_pressure_reports(solver_session, report_zones, report_files):
   """
   只用 TUI journal 创建表面压力 Report Definitions，避免 settings API object is not active。
   """
   return create_surface_pressure_reports_tui_journal(
      solver_session=solver_session,
      report_zones=report_zones,
      report_files=report_files,
   )


def compute_surface_pressure_by_tui(solver_session, report_zones, report_kind, output_file):
   """
   计算表面平均/最大静压并写出文件。
   优先使用 GUI 中的 Report Definition，再用 surface-integrals 兜底。
   """
   report_zones = resolve_force_report_zones_for_solver(solver_session, report_zones)
   if len(report_zones) == 0:
      return None

   zone_expr = " ".join(report_zones)

   if report_kind == "avg":
      report_name = REPORT_SURFACE_PRESSURE_AVG_NAME
      title = REPORT_SURFACE_PRESSURE_AVG_NAME
      fallback_commands = [
         f"/report/surface-integrals/area-weighted-avg pressure {zone_expr} ()",
         f"/report/surface-integrals/area-weighted-average pressure {zone_expr} ()",
         f"/report/surface-integrals/facet-average pressure {zone_expr} ()",
      ]
   else:
      report_name = REPORT_SURFACE_PRESSURE_MAX_NAME
      title = REPORT_SURFACE_PRESSURE_MAX_NAME
      fallback_commands = [
         f"/report/surface-integrals/maximum pressure {zone_expr} ()",
         f"/report/surface-integrals/facet-maximum pressure {zone_expr} ()",
         f"/report/surface-integrals/vertex-maximum pressure {zone_expr} ()",
      ]

   val = None
   tried = []

   for cmd in [
      f"/solve/report-definitions/compute {report_name}",
      f"/solve/report-definitions/compute {report_name} quit",
      f"/solve/report-files/write {report_name}",
      "/solve/report-files/write-all",
   ]:
      tried.append(cmd)
      try:
         solver_tui(solver_session, cmd)
         time.sleep(0.3)
      except Exception:
         pass

   try:
      val = extract_latest_force_value(output_file)
   except Exception:
      val = None

   if val is None:
      for i, cmd in enumerate(fallback_commands, start=1):
         tried.append(cmd)
         try:
            val = run_tui_report_to_file(
               solver_session=solver_session,
               commands=[cmd],
               output_file=output_file,
               transcript_name=f"tmp_{title}_{i}.trn",
            )
            if val is not None:
               break
         except Exception:
            pass

   write_scalar_report_out(
      file_path=output_file,
      title=title,
      value=val,
      extra_lines=[
         "source = report_definitions_surface_report",
         "field = pressure",
         f"zones = {report_zones}",
      ] + tried,
   )

   return val


def parse_wall_forces_transcript(text, report_zones=None):
   """
   解析 Fluent Results -> Reports -> Forces 的输出表。

   目标表格形式：
      Forces - Direction Vector (1 0 0)

                              Forces [N]                         Coefficients
      Zone                      Pressure        Viscous         Total
      auv_body                  482.11585       85.625867       567.74172
      Net                       482.11585       85.625867       567.74172

   取 Net 行的前三个数：
      Pressure = 压差阻力
      Viscous  = 摩擦阻力
      Total    = 总阻力
   如果没有 Net 行，则累加所选 AUV wall zones 的前三列。
   """
   result = {"pressure": None, "viscous": None, "total": None}

   if text is None:
      return result

   lines = str(text).splitlines()
   float_re = re.compile(r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?")

   # 1. 优先解析 Net 行。
   for line in lines:
      parts = line.strip().split()
      if len(parts) >= 4 and parts[0].lower() == "net":
         nums = [float(x) for x in float_re.findall(line)]
         if len(nums) >= 3:
            result["pressure"] = nums[0]
            result["viscous"] = nums[1]
            result["total"] = nums[2]
            return result

   # 2. 解析具体 zone 行。若多个 zone，则累加。
   selected = []
   if report_zones:
      selected = [str(z).lower() for z in report_zones]

   sum_pressure = 0.0
   sum_viscous = 0.0
   sum_total = 0.0
   count = 0

   for line in lines:
      parts = line.strip().split()
      if len(parts) < 4:
         continue

      zone_name = parts[0].lower()

      if selected:
         matched = False
         for z in selected:
            if zone_name == z or z in zone_name or zone_name in z:
               matched = True
               break
         if not matched:
            continue
      else:
         # 没有指定 zone 时，跳过表头和分隔线。
         if zone_name in ["zone", "forces", "coefficients", "direction", "vector"]:
            continue
         if set(zone_name) == set("-"):
            continue

      nums = [float(x) for x in float_re.findall(line)]
      if len(nums) >= 3:
         sum_pressure += nums[0]
         sum_viscous += nums[1]
         sum_total += nums[2]
         count += 1

   if count > 0:
      result["pressure"] = sum_pressure
      result["viscous"] = sum_viscous
      result["total"] = sum_total
      return result

   # 3. 极端兜底：解析包含 pressure / viscous / total 关键词的行。
   for line in lines:
      low = line.lower()
      nums = [float(x) for x in float_re.findall(line)]
      if not nums:
         continue
      val = nums[0] if len(nums) >= 3 else nums[-1]
      if "pressure" in low and result["pressure"] is None:
         result["pressure"] = val
      elif ("viscous" in low or "friction" in low or "shear" in low) and result["viscous"] is None:
         result["viscous"] = val
      elif "total" in low and result["total"] is None:
         result["total"] = val

   return result


def parse_force_triplets_from_line(line):
   """
   解析 Fluent Force Report 的一行。

   支持：
   1. Direction Vector 标量表：
      Net 243.42879 7.4332302 250.86202 ...
   2. 全矢量表：
      Net (243.42879 -28.24 1.75) (7.433 0.53 -0.12) (250.86 ...)
      取每个向量的 X 分量。
   """
   float_re = re.compile(r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?")

   groups = re.findall(r"\(([^)]*)\)", str(line))
   if len(groups) >= 3:
      parsed_groups = []
      for g in groups[:3]:
         nums = [float(x) for x in float_re.findall(g)]
         if len(nums) >= 1:
            parsed_groups.append(nums)

      if len(parsed_groups) >= 3:
         return {
            "pressure": parsed_groups[0][0],
            "viscous": parsed_groups[1][0],
            "total": parsed_groups[2][0],
         }

   nums = [float(x) for x in float_re.findall(str(line))]
   if len(nums) >= 3:
      return {
         "pressure": nums[0],
         "viscous": nums[1],
         "total": nums[2],
      }

   return {"pressure": None, "viscous": None, "total": None}


def _looks_like_forces_table_header(text):
   low = str(text).lower()
   return ("forces [n]" in low and "pressure" in low and "viscous" in low and "total" in low)


def _extract_real_forces_blocks(text):
   """
   只提取真实 Fluent Forces 输出块。
   不再从 mesh summary、zone list、residual 等全文数字中猜测。
   """
   lines = str(text).splitlines()
   blocks = []

   for i, line in enumerate(lines):
      low = line.lower().strip()

      if "forces - direction vector" in low:
         header = "\n".join(lines[i:i+20])
         if not _looks_like_forces_table_header(header):
            continue

         block = []
         for j in range(i, min(len(lines), i + 180)):
            block.append(lines[j])
            if j > i and lines[j].strip().lower().startswith("net"):
               break

         blocks.append(("direction", block))

      elif low == "forces":
         header = "\n".join(lines[i:i+12])
         if not _looks_like_forces_table_header(header):
            continue

         block = []
         for j in range(i, min(len(lines), i + 220)):
            block.append(lines[j])
            if j > i and lines[j].strip().lower().startswith("net"):
               break

         blocks.append(("vector", block))

   return blocks


def _parse_net_from_forces_block(block):
   if not block:
      return {"pressure": None, "viscous": None, "total": None}

   for line in block:
      parts = line.strip().split()
      if len(parts) >= 4 and parts[0].lower() == "net":
         parsed = parse_force_triplets_from_line(line)
         if parsed.get("pressure") is not None:
            return parsed

   return {"pressure": None, "viscous": None, "total": None}


def _parse_zone_sum_from_direction_block(block, report_zones=None):
   """
   只有真实 Direction Vector 表没有 Net 行时才累加 zone。
   """
   result = {"pressure": None, "viscous": None, "total": None}

   if not block:
      return result

   selected = [str(z).lower() for z in report_zones] if report_zones else []

   in_table = False
   sums = {"pressure": 0.0, "viscous": 0.0, "total": 0.0}
   count = 0

   for line in block:
      low = line.lower()

      if "zone" in low and "pressure" in low and "viscous" in low and "total" in low:
         in_table = True
         continue

      if not in_table:
         continue

      parts = line.strip().split()
      if len(parts) < 4:
         continue

      zone_name = parts[0].lower()

      if zone_name in ["zone", "forces", "coefficients", "net"]:
         continue
      if all(ch == "-" for ch in zone_name):
         continue

      if selected and not any(zone_name == z or z in zone_name or zone_name in z for z in selected):
         continue

      parsed = parse_force_triplets_from_line(line)

      if parsed.get("pressure") is not None:
         sums["pressure"] += parsed["pressure"]
         sums["viscous"] += parsed["viscous"]
         sums["total"] += parsed["total"]
         count += 1

   if count > 0:
      return sums

   return result


def parse_wall_forces_transcript(text, report_zones=None):
   """
   严格解析 Results -> Reports -> Forces 输出。

   只接受真实 Forces 表：
      Forces - Direction Vector (1 0 0)
      Net    Pressure    Viscous    Total

   若没有真实 Forces 表，返回 None，绝不输出 10229 这种假数。
   """
   result = {"pressure": None, "viscous": None, "total": None}

   if text is None:
      return result

   blocks = _extract_real_forces_blocks(text)
   if not blocks:
      return result

   direction_blocks = [b for kind, b in blocks if kind == "direction"]

   for block in reversed(direction_blocks):
      parsed = _parse_net_from_forces_block(block)
      if parsed.get("pressure") is not None:
         return parsed

      parsed = _parse_zone_sum_from_direction_block(block, report_zones=report_zones)
      if parsed.get("pressure") is not None:
         return parsed

   vector_blocks = [b for kind, b in blocks if kind == "vector"]

   for block in reversed(vector_blocks):
      parsed = _parse_net_from_forces_block(block)
      if parsed.get("pressure") is not None:
         return parsed

   return result


def _list_indices_scheme(indices):
   if indices is None or len(indices) == 0:
      return "'()"
   return "'( " + " ".join([str(int(i)) for i in indices]) + ")"


def _normalize_zone_for_match(name):
   return str(name).strip().lower()


def _is_forbidden_force_zone_name(name):
   """
   这些不是 AUV 模型表面，不能进入 Results -> Reports -> Forces 的 Wall Zones。
   尤其是 fluid:1：它在 Force Reports 面板里会出现，但不是模型受力表面。
   """
   low = _normalize_zone_for_match(name)

   if low in ["wall", "outer_wall", "outer-wall", "inlet", "outlet", "fluid", "interior--fluid"]:
      return True

   if low.startswith("fluid:"):
      return True

   if "interior" in low:
      return True

   if "inlet" in low or "outlet" in low:
      return True

   return False


def _force_panel_wall_zone_order(solver_session):
   """
   Force Reports 面板的 Wall Zones 列表不是 get_bc_names("wall") 的原始顺序，
   而是按名称排序后显示。

   你这次的日志证明：
      选择 0..17 会选到 fluid:1，漏掉 sonar。
   原因是 GUI 排序后大致为：
      antenna, ..., fluid:1, payload, propeller, sonar
   所以 sonar 实际是后面的 index，不能按原始 wall_zones 顺序映射。
   """
   wall_zones = get_bc_names(solver_session, "wall")

   # GUI 面板的显示顺序按字母顺序；大小写统一用 lower。
   ordered = sorted(wall_zones, key=lambda x: str(x).lower())

   return ordered


def _resolve_gui_wall_zone_indices(solver_session, report_zones):
   """
   把 XYZ 三个方向力报告已经使用的 AUV 表面 report_zones
   精确映射到 Results -> Reports -> Forces 面板的 Wall Zones 索引。

   原则：
      1. 只选模型表面；
      2. 索引按 Force Reports GUI 面板排序后的列表计算；
      3. 不选 fluid:1、wall、inlet、outlet、interior 等非模型面；
      4. 优先 exact match，不再用过宽的包含匹配。
   """
   gui_wall_zones = _force_panel_wall_zone_order(solver_session)

   desired = []
   for z in (report_zones or []):
      zs = str(z).strip()
      if not zs:
         continue
      if _is_forbidden_force_zone_name(zs):
         continue
      desired.append(zs)

   desired_norm = [_normalize_zone_for_match(z) for z in desired]
   desired_set = set(desired_norm)

   selected = []
   selected_names = []
   missing = []

   # 第一层：精确匹配。这里应该覆盖你的正常情况。
   for i, z in enumerate(gui_wall_zones):
      zn = _normalize_zone_for_match(z)

      if _is_forbidden_force_zone_name(z):
         continue

      if zn in desired_set:
         selected.append(i)
         selected_names.append(z)

   # 记录没有精确匹配到的模型面。
   selected_norm = set([_normalize_zone_for_match(z) for z in selected_names])
   for z in desired:
      if _normalize_zone_for_match(z) not in selected_norm:
         missing.append(z)

   # 第二层：只对缺失项做非常保守的 fallback，仍然排除 fluid 等非模型面。
   # 注意：不再使用 "zone_name in target" 这种过宽规则，避免 fluid:1 被误选。
   if missing:
      for miss in missing[:]:
         mn = _normalize_zone_for_match(miss)

         for i, z in enumerate(gui_wall_zones):
            if i in selected:
               continue
            if _is_forbidden_force_zone_name(z):
               continue

            zn = _normalize_zone_for_match(z)

            # 只允许同名后缀/前缀很明确的情况。
            if zn == mn:
               selected.append(i)
               selected_names.append(z)
               missing.remove(miss)
               break

   # 如果 report_zones 本身为空，才允许兜底选择所有 AUV-like wall zones。
   if len(selected) == 0 and not desired:
      for i, z in enumerate(gui_wall_zones):
         if _is_forbidden_force_zone_name(z):
            continue
         selected.append(i)
         selected_names.append(z)

   selected = sorted(set(selected))

   print(f"   GUI Force Reports wall zones sorted order = {gui_wall_zones}")
   print(f"   XYZ 力报告使用的模型表面 = {desired}")
   print(f"   GUI Force Reports selected zone names = {selected_names}")
   print(f"   GUI Force Reports selected indices = {selected}")

   if missing:
      print(f"   警告：以下 XYZ 力报告表面没有在 Force Reports GUI wall list 中精确找到: {missing}")

   # 强制检查：不允许选到 fluid:1 等非模型面。
   bad_selected = []
   for i in selected:
      try:
         name = gui_wall_zones[i]
         if _is_forbidden_force_zone_name(name):
            bad_selected.append(name)
      except Exception:
         pass

   if bad_selected:
      raise RuntimeError("Force Reports 选中了非模型表面，已停止选择: " + str(bad_selected))

   return selected



def _read_file_if_exists(path):
   try:
      local = str(path).replace("/", os.sep)
      if os.path.exists(local):
         with open(local, "r", encoding="utf-8", errors="ignore") as f:
            return f.read()
   except Exception:
      pass
   return ""


def _read_recent_force_report_files(start_time, base_name):
   """
   读取 GUI Write 可能保存出来的 .frp 文件。
   Fluent 文件对话框有时会自动补扩展名，也可能把相对路径保存到 WORK_DIR。
   """
   raw = ""
   candidates = []

   try:
      for fn in os.listdir(WORK_DIR):
         low = fn.lower()
         if low.endswith(".frp") or "force" in low or "forces" in low or "results_report_forces" in low:
            p = os.path.join(WORK_DIR, fn)
            try:
               mt = os.path.getmtime(p)
            except Exception:
               continue

            if mt >= start_time - 2.0:
               candidates.append((mt, p))
   except Exception:
      pass

   for suffix in ["", ".frp", ".out", ".txt"]:
      p = os.path.join(WORK_DIR, base_name + suffix)
      try:
         if os.path.exists(p):
            candidates.append((os.path.getmtime(p), p))
      except Exception:
         pass

   seen = set()
   candidates = sorted(candidates, reverse=True)

   for mt, p in candidates:
      if p in seen:
         continue
      seen.add(p)

      text = _read_file_if_exists(p)
      if text:
         raw += f"\n\n===== recent force report file: {p} =====\n"
         raw += text

   return raw


def read_latest_fluent_transcript_text():
   """
   兜底读取 WORK_DIR 中最新 fluent-*.trn。
   注意：解析函数只认真实 Forces 表，不会再从普通日志中误取数字。
   """
   try:
      candidates = []

      for fn in os.listdir(WORK_DIR):
         low = fn.lower()
         if low.startswith("fluent-") and low.endswith(".trn"):
            p = os.path.join(WORK_DIR, fn)
            try:
               candidates.append((os.path.getmtime(p), p))
            except Exception:
               pass

      if not candidates:
         return ""

      candidates.sort(reverse=True)
      latest = candidates[0][1]

      with open(latest, "r", encoding="utf-8", errors="ignore") as f:
         text = f.read()

      print(f"   已读取最新 Fluent 全局 transcript 作为 Forces 解析兜底: {latest}")
      return text

   except Exception as e:
      print(f"   读取最新 Fluent transcript 失败: {e}")
      return ""



# ========================================================
# v65 最终覆盖：Results -> Reports -> Forces 表解析
# ========================================================
def _v65_float_list_from_line(line):
   float_re = re.compile(r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?")
   return [float(x) for x in float_re.findall(str(line))]


def _v65_parse_net_line(line):
   """
   目标 Net 行：
      Net 243.42879 7.4332302 250.86202 397.43475 12.135886 409.57064

   第 1 个数 = 压差阻力 Pressure Drag_X
   第 2 个数 = 摩擦阻力 Friction Drag_X
   第 3 个数 = X 方向总阻力 Total Drag_X
   """
   s = str(line).strip()

   if not s.lower().startswith("net"):
      return {"pressure": None, "viscous": None, "total": None}

   groups = re.findall(r"\(([^)]*)\)", s)

   if len(groups) >= 3:
      parsed_groups = []
      for g in groups[:3]:
         nums = _v65_float_list_from_line(g)
         if len(nums) >= 1:
            parsed_groups.append(nums)

      if len(parsed_groups) >= 3:
         return {
            "pressure": parsed_groups[0][0],
            "viscous": parsed_groups[1][0],
            "total": parsed_groups[2][0],
         }

   nums = _v65_float_list_from_line(s)

   if len(nums) >= 3:
      return {
         "pressure": nums[0],
         "viscous": nums[1],
         "total": nums[2],
      }

   return {"pressure": None, "viscous": None, "total": None}


def _v65_has_pressure_viscous_total(lines):
   text = "\n".join([str(x).lower() for x in lines])
   return ("pressure" in text) and ("viscous" in text) and ("total" in text)


def _v65_parse_direction_vector_table(text):
   """
   优先解析 Forces - Direction Vector (1 0 0) 表。
   """
   lines = str(text).splitlines()
   direction_indices = []

   for i, line in enumerate(lines):
      low = str(line).lower()
      if "forces" in low and "direction" in low and "vector" in low:
         direction_indices.append(i)

   for idx in reversed(direction_indices):
      block = lines[idx:min(len(lines), idx + 240)]

      # 必须确认附近有 Pressure / Viscous / Total，防止误解析其它 Net。
      if not _v65_has_pressure_viscous_total(block[:80]):
         continue

      for line in block:
         if str(line).strip().lower().startswith("net"):
            parsed = _v65_parse_net_line(line)
            if parsed.get("pressure") is not None:
               return parsed

   return {"pressure": None, "viscous": None, "total": None}


def _v65_parse_vector_forces_table(text):
   """
   兜底解析全矢量 Forces 表。取 Net 行三个向量的 X 分量。
   """
   lines = str(text).splitlines()
   force_indices = []

   for i, line in enumerate(lines):
      if str(line).strip().lower() == "forces":
         force_indices.append(i)

   for idx in reversed(force_indices):
      block = lines[idx:min(len(lines), idx + 280)]

      if not _v65_has_pressure_viscous_total(block[:100]):
         continue

      for line in block:
         if str(line).strip().lower().startswith("net"):
            parsed = _v65_parse_net_line(line)
            if parsed.get("pressure") is not None:
               return parsed

   return {"pressure": None, "viscous": None, "total": None}


def parse_wall_forces_transcript(text, report_zones=None):
   """
   v65 解析规则：
   1. 只解析 Results -> Reports -> Forces 真实输出表；
   2. 优先取 Forces - Direction Vector (1 0 0) 的 Net 行；
   3. 直接取 Net 行前三个数：
        Pressure = 第 1 个数
        Viscous  = 第 2 个数
        Total    = 第 3 个数
   """
   result = {"pressure": None, "viscous": None, "total": None}

   if text is None:
      return result

   for parser in [_v65_parse_direction_vector_table, _v65_parse_vector_forces_table]:
      try:
         parsed = parser(text)
         if parsed.get("pressure") is not None:
            return parsed
      except Exception as e:
         print(f"   v65 Forces 解析分支失败: {e}")

   return result


def _v65_read_file_tail(path, max_chars=2500000):
   try:
      local = str(path).replace("/", os.sep)
      if not os.path.exists(local):
         return ""
      with open(local, "r", encoding="utf-8", errors="ignore") as f:
         data = f.read()
      if len(data) > max_chars:
         return data[-max_chars:]
      return data
   except Exception:
      return ""


def _v65_read_recent_global_fluent_transcripts(start_time=None, max_files=5):
   """
   读取 WORK_DIR 中最近的 fluent-*.trn。
   你上传的 trn 已证明 GUI OK 后 Forces 表会写入这个全局 transcript。
   """
   raw = ""

   try:
      candidates = []
      for fn in os.listdir(WORK_DIR):
         low = fn.lower()
         if low.startswith("fluent-") and low.endswith(".trn"):
            p = os.path.join(WORK_DIR, fn)
            try:
               mt = os.path.getmtime(p)
            except Exception:
               continue

            if start_time is None or mt >= start_time - 10.0:
               candidates.append((mt, p))

      candidates.sort(reverse=True)

      for mt, p in candidates[:max_files]:
         txt = _v65_read_file_tail(p)
         if txt:
            raw += f"\n\n===== v65 global fluent transcript: {p} =====\n"
            raw += txt
            print(f"   v65 已读取全局 Fluent transcript: {p}")

   except Exception as e:
      raw += f"\n[v65 read global transcript failed] {e}\n"
      print(f"   v65 读取全局 transcript 失败: {e}")

   return raw


def _v65_read_recent_force_files(start_time=None, base_name=None, max_files=20):
   """
   读取 GUI Write 可能生成的 .frp/.out 文件。
   """
   raw = ""

   try:
      candidates = []
      for fn in os.listdir(WORK_DIR):
         low = fn.lower()
         if not (low.endswith(".frp") or low.endswith(".out") or "force" in low or "forces" in low):
            continue

         p = os.path.join(WORK_DIR, fn)
         try:
            mt = os.path.getmtime(p)
         except Exception:
            continue

         if start_time is None or mt >= start_time - 10.0:
            candidates.append((mt, p))

      if base_name:
         for suffix in ["", ".frp", ".out", ".txt", ".trn"]:
            p = os.path.join(WORK_DIR, base_name + suffix)
            try:
               if os.path.exists(p):
                  candidates.append((os.path.getmtime(p), p))
            except Exception:
               pass

      seen = set()
      candidates.sort(reverse=True)

      for mt, p in candidates[:max_files]:
         if p in seen:
            continue
         seen.add(p)

         txt = _v65_read_file_tail(p)
         if txt:
            raw += f"\n\n===== v65 recent force file: {p} =====\n"
            raw += txt

   except Exception as e:
      raw += f"\n[v65 read force files failed] {e}\n"

   return raw



def safe_close_force_reports_panel(solver_session):
   """
   新工作站稳定版：不再主动执行 Force Reports 面板关闭命令。

   原因：不同 Fluent GUI 版本的按钮 widget 名称不一致，
   强行执行 cx-gui-do 关闭命令时，终端会出现大量：
      cannot find widget / cx-close-dialog / cx-hide-dialog
   这些报错虽然不影响 Forces 结果，但会干扰日志阅读。

   当前策略：Results -> Reports -> Forces 的 OK 已经执行完成，
   数据也已经成功写入 transcript / out 文件，因此这里直接跳过自动关闭面板。
   """
   print("   已跳过自动关闭 Force Reports 面板（避免 widget 报错，不影响结果）。")
   return True


def run_forces_panel_by_gui_macro(solver_session, report_zones, transcript_path):
   """
   v66：用 journal 一次性执行 Results -> Reports -> Forces 的 GUI 命令。

   v65 的问题：
      fluent-*.trn 里只出现了第一条
      (cx-gui-do cx-set-list-tree-selections ... "Results|Reports|Forces")
      后面没有 Wall Zones 选择，也没有 OK，因此源头上没有真正生成 Forces 表。

   v66 的改法：
      1. 把完整 GUI 操作写入一个 .jou；
      2. 用 solver_session.tui.file.read_journal 一次性执行；
      3. 日志里应该同时出现：
         - Results|Reports|Forces
         - Force Reports*Table2*List1(Wall Zones)
         - Force Reports*PanelButtons*PushButton1(OK)
      4. OK 后从全局 fluent-*.trn / 局部 transcript / .frp 中解析 Net 行；
      5. 保存/输出完成后，在 journal 外部安全尝试关闭 Force Reports 面板。
   """
   local_transcript = str(transcript_path).replace("/", os.sep)
   base_no_ext = os.path.splitext(os.path.basename(local_transcript))[0]
   gui_journal_path = os.path.join(WORK_DIR, base_no_ext + "_gui_forces.jou").replace(chr(92), "/")

   try:
      if os.path.exists(local_transcript):
         os.remove(local_transcript)
   except Exception:
      pass

   indices = _resolve_gui_wall_zone_indices(solver_session, report_zones)
   indices_expr = _list_indices_scheme(indices)

   raw = ""
   start_time = time.time()

   print("   ▶ v66 用 GUI journal 执行 Results -> Reports -> Forces")
   print(f"   ▶ GUI journal = {gui_journal_path}")
   print(f"   ▶ Wall zone indices = {indices_expr}")
   print("   ▶ 注意：这些 indices 已按 Force Reports GUI 排序后的 Wall Zones 列表计算，排除了 fluid:1 等非模型面。")
   print("   ▶ 目标命令必须执行到 Force Reports*PanelButtons*PushButton1(OK)，随后在 journal 外安全尝试关闭 Force Reports 面板")

   # 这组命令直接参照你手动成功的 .trn。
   # 不逐条 scheme_eval，避免 v65 只执行第一句就停住。
   gui_cmds = [
      '(cx-gui-do cx-set-list-tree-selections "NavigationPane*Frame2*Table1*List_Tree2" (list "Results|Reports|Forces"))',
      '(cx-gui-do cx-set-list-tree-selections "NavigationPane*Frame2*Table1*List_Tree2" (list "Results|Reports|Forces"))',
      '(cx-gui-do cx-activate-item "NavigationPane*Frame2*Table1*List_Tree2")',
      '(cx-gui-do cx-set-list-tree-selections "NavigationPane*Frame2*Table1*List_Tree2" (list "Results|Reports|Forces"))',
      f'(cx-gui-do cx-set-list-selections "Force Reports*Table2*List1(Wall Zones)" {indices_expr})',
      '(cx-gui-do cx-activate-item "Force Reports*Table2*List1(Wall Zones)")',
      f'(cx-gui-do cx-set-list-selections "Force Reports*Table2*List1(Wall Zones)" {indices_expr})',
      '(cx-gui-do cx-activate-item "Force Reports*Table2*List1(Wall Zones)")',
      '(cx-gui-do cx-activate-item "Force Reports*PanelButtons*PushButton1(OK)")',
      # OK 成功后，面板通常仍在；尝试写一个 frp，失败也不影响从 transcript 读表。
      '(cx-gui-do cx-activate-item "Force Reports*PanelButtons*PushButton4(Write)")',
      f'(cx-gui-do cx-set-file-dialog-entries "Select File" \'( "{base_no_ext}") "Force Report Files (*.frp)")',
   ]

   try:
      with open(gui_journal_path, "w", encoding="utf-8") as f:
         for cmd in gui_cmds:
            f.write(cmd + "\n")
   except Exception as e:
      raw += f"\n[v66 write gui journal failed] {e}\n"
      print(f"   v66 GUI journal 写入失败: {e}")

   try:
      solver_tui(solver_session, f'/file/start-transcript "{normalize_path_for_fluent(local_transcript)}"')
      time.sleep(0.5)
   except Exception as e:
      raw += f"\n[v66 start transcript failed] {e}\n"

   try:
      solver_session.tui.file.read_journal(gui_journal_path)
      raw += f"\n[v66 gui journal executed] {gui_journal_path}\n"
      time.sleep(4.0)
   except Exception as e:
      raw += f"\n[v66 gui journal read failed] {e}\n"
      print(f"   v66 GUI journal 执行失败: {e}")

      # 如果 read_journal 失败，再用 ti-menu-load-string 一次性送入完整 GUI 命令串。
      try:
         block = "\n".join(gui_cmds)
         escaped = block.replace(chr(92), "/").replace('"', '\\"')
         solver_session.scheme_eval.eval(f'(ti-menu-load-string "{escaped}\n")')
         raw += "\n[v66 gui ti-menu-load-string fallback executed]\n"
         time.sleep(4.0)
      except Exception as e2:
         raw += f"\n[v66 gui ti-menu-load-string fallback failed] {e2}\n"
         print(f"   v66 GUI ti-menu-load-string 兜底失败: {e2}")

   finally:
      try:
         solver_tui(solver_session, "/file/stop-transcript")
         time.sleep(0.7)
      except Exception:
         pass

   # journal 已完成；单独尝试关闭 Force Reports 面板。
   # 这一步即使失败也不会影响 Forces 表解析。
   safe_close_force_reports_panel(solver_session)

   # 读取局部 transcript
   raw += "\n\n===== v66 local forces transcript =====\n"
   raw += _v65_read_file_tail(local_transcript)

   # 读取全局 fluent-*.trn：这是手动 GUI 成功时 Forces 表所在的位置。
   raw += "\n\n===== v66 global fluent transcript fallback =====\n"
   raw += _v65_read_recent_global_fluent_transcripts(start_time=start_time, max_files=6)

   # 读取可能生成的 .frp / .out
   raw += "\n\n===== v66 recent force files =====\n"
   raw += _v65_read_recent_force_files(start_time=start_time, base_name=base_no_ext, max_files=30)

   if "Force Reports*PanelButtons*PushButton1(OK)" in raw:
      print("   v66 确认：日志中已出现 Force Reports OK 命令。")
   else:
      print("   v66 警告：捕获文本中没有发现 Force Reports OK 命令，说明 GUI journal 仍未跑到 OK。")

   if "Forces - Direction Vector" in raw or "\nForces" in raw:
      print("   v66 确认：捕获文本中存在 Forces 输出表。")
   else:
      print("   v66 警告：捕获文本中没有 Forces 输出表。")

   parsed = parse_wall_forces_transcript(raw, report_zones=report_zones)
   if parsed.get("pressure") is not None:
      print(
         f"   v66 已解析 Net 行: "
         f"Pressure={parsed.get('pressure')}, Viscous={parsed.get('viscous')}, Total={parsed.get('total')}"
      )

   return raw



def run_forces_panel_by_ti_menu_string(solver_session, report_zones, frp_name, transcript_path):
   """
   用 ti-menu-load-string 模拟手动操作 Results -> Reports -> Forces。

   之前用 file.read_journal 时，Fluent 在 filename (*.frp) 提示处没有吃到下一行输入，
   于是反复报 Empty filename。这里改成一次性把交互输入送给 Fluent 的 TUI 解释器。
   """
   zone_expr = " ".join(report_zones)
   local_transcript = str(transcript_path).replace("/", os.sep)

   # 这里的第一行进入 Forces 面板；第二行就是 filename (*.frp) 的回答。
   # 不加引号，不给绝对路径，只给当前工作目录下的相对文件名。
   block = "\n".join([
      "/report/forces/wall-forces",
      frp_name,
      zone_expr,
      "()",
      "1",
      "0",
      "0",
      "",
   ])

   try:
      if os.path.exists(local_transcript):
         os.remove(local_transcript)
   except Exception:
      pass

   print("   ▶ 执行 Results-Report-Forces，使用 ti-menu-load-string，一次性送入交互输入")
   print(f"   ▶ Forces frp filename = {frp_name}")
   print(f"   ▶ Forces wall zones = {report_zones}")
   print("   ▶ Direction Vector = (1, 0, 0)")
   print("   ------------------------------------------------------------")
   for line in block.splitlines():
      if line.strip():
         print(f"   TUI> {line}")
   print("   ------------------------------------------------------------")

   raw = ""
   try:
      solver_tui(solver_session, f'/file/start-transcript "{normalize_path_for_fluent(local_transcript)}"')
      time.sleep(0.2)

      escaped = block.replace(chr(92), "/").replace('"', '\\"')
      solver_session.scheme_eval.eval(f'(ti-menu-load-string "{escaped}")')
      time.sleep(1.2)
   except Exception as e:
      raw += f"\n[ti-menu-load-string failed] {e}\n"
      print(f"   Results-Report-Forces ti-menu-load-string 失败: {e}")
   finally:
      try:
         solver_tui(solver_session, "/file/stop-transcript")
         time.sleep(0.3)
      except Exception:
         pass

   try:
      if os.path.exists(local_transcript):
         with open(local_transcript, "r", encoding="utf-8", errors="ignore") as f:
            raw += f.read()
   except Exception:
      pass

   return raw



# ========================================================
# v64 最终覆盖：压差阻力/摩擦阻力 Forces 表解析
# ========================================================
def _v64_float_list_from_line(line):
   float_re = re.compile(r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?")
   return [float(x) for x in float_re.findall(str(line))]


def _v64_parse_net_line(line):
   """
   解析 Net 行。
   目标形式：
      Net 243.42879 7.4332302 250.86202 397.43475 12.135886 409.57064
   需要第 1、2 个数：
      pressure = 243.42879
      viscous  = 7.4332302

   也支持全矢量形式：
      Net (243.42879 -28.24 1.75) (7.433 0.53 -0.12) (250.86 ...)
   取每个括号组的第一个数。
   """
   s = str(line).strip()
   if not s.lower().startswith("net"):
      return {"pressure": None, "viscous": None, "total": None}

   groups = re.findall(r"\(([^)]*)\)", s)
   if len(groups) >= 3:
      parsed_groups = []
      for g in groups[:3]:
         nums = _v64_float_list_from_line(g)
         if len(nums) >= 1:
            parsed_groups.append(nums)
      if len(parsed_groups) >= 3:
         return {
            "pressure": parsed_groups[0][0],
            "viscous": parsed_groups[1][0],
            "total": parsed_groups[2][0],
         }

   nums = _v64_float_list_from_line(s)
   if len(nums) >= 3:
      return {
         "pressure": nums[0],
         "viscous": nums[1],
         "total": nums[2],
      }

   return {"pressure": None, "viscous": None, "total": None}


def _v64_line_is_net(line):
   return str(line).strip().lower().startswith("net")


def _v64_block_contains_force_header(lines):
   text = "\n".join([str(x).lower() for x in lines])
   if "pressure" not in text:
      return False
   if "viscous" not in text:
      return False
   if "total" not in text:
      return False
   return True


def _v64_parse_last_direction_vector_net(text):
   """
   优先解析最后一个 Forces - Direction Vector 表。
   不再要求严格匹配 Forces [N]，因为有的 transcript/编码会让 header 轻微变化。
   只要该块附近有 Pressure/Viscous/Total 表头，并且后面有 Net 行，就取 Net 行前三个数。
   """
   lines = str(text).splitlines()

   direction_indices = []
   for i, line in enumerate(lines):
      if "forces" in line.lower() and "direction" in line.lower() and "vector" in line.lower():
         direction_indices.append(i)

   for idx in reversed(direction_indices):
      block = lines[idx:min(len(lines), idx + 220)]

      if not _v64_block_contains_force_header(block[:40]):
         # 即使表头没被捕捉完整，也继续找 Net；但必须至少看到 Pressure/Viscous/Total 之一。
         loose_text = "\n".join([str(x).lower() for x in block[:80]])
         if "pressure" not in loose_text or "viscous" not in loose_text:
            continue

      for line in block:
         if _v64_line_is_net(line):
            parsed = _v64_parse_net_line(line)
            if parsed.get("pressure") is not None:
               return parsed

   return {"pressure": None, "viscous": None, "total": None}


def _v64_parse_last_forces_vector_net(text):
   """
   兜底解析全矢量 Forces 表：
      Forces
      Zone Pressure Viscous Total
      ...
      Net (px py pz) (vx vy vz) (tx ty tz)
   取 x 分量。
   """
   lines = str(text).splitlines()
   force_indices = []

   for i, line in enumerate(lines):
      if str(line).strip().lower() == "forces":
         force_indices.append(i)

   for idx in reversed(force_indices):
      block = lines[idx:min(len(lines), idx + 260)]
      if not _v64_block_contains_force_header(block[:50]):
         continue

      for line in block:
         if _v64_line_is_net(line):
            parsed = _v64_parse_net_line(line)
            if parsed.get("pressure") is not None:
               return parsed

   return {"pressure": None, "viscous": None, "total": None}


def _v64_parse_last_pressure_viscous_net_anywhere(text):
   """
   最后兜底：在全文中找处于 Force 表头之后的 Net 行。
   注意必须要求前面最多 80 行内出现 Pressure/Viscous/Total，避免解析 mesh summary 的 Net。
   """
   lines = str(text).splitlines()

   for i in range(len(lines) - 1, -1, -1):
      line = lines[i]
      if not _v64_line_is_net(line):
         continue

      context = lines[max(0, i - 80):i + 1]
      if not _v64_block_contains_force_header(context):
         continue

      parsed = _v64_parse_net_line(line)
      if parsed.get("pressure") is not None:
         return parsed

   return {"pressure": None, "viscous": None, "total": None}


def parse_wall_forces_transcript(text, report_zones=None):
   """
   v64 修正：专门提取 Forces 表 Net 行。

   用户确认需要：
      Net 243.42879 7.4332302 250.86202 397.43475 12.135886 409.57064

   提取规则：
      第 1 个数 = 压差阻力 Pressure Drag_X
      第 2 个数 = 摩擦阻力 Friction Drag_X
      第 3 个数 = X 向总阻力 Total Drag_X

   解析优先级：
      1. 最后一个 Forces - Direction Vector 表的 Net 行；
      2. 最后一个全矢量 Forces 表的 Net 行；
      3. 处于 Pressure/Viscous/Total 表头之后的最后一个 Net 行。
   """
   result = {"pressure": None, "viscous": None, "total": None}

   if text is None:
      return result

   for parser in [
      _v64_parse_last_direction_vector_net,
      _v64_parse_last_forces_vector_net,
      _v64_parse_last_pressure_viscous_net_anywhere,
   ]:
      try:
         parsed = parser(text)
         if parsed.get("pressure") is not None:
            return parsed
      except Exception as e:
         print(f"   v64 Forces 解析分支失败: {e}")

   return result



# ========================================================
# v66 最终覆盖：只从真实 Forces 表 Net 行取压差/摩擦阻力
# ========================================================
def _v66_nums(line):
   float_re = re.compile(r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?")
   return [float(x) for x in float_re.findall(str(line))]


def _v66_parse_net_line(line):
   s = str(line).strip()
   if not s.lower().startswith("net"):
      return {"pressure": None, "viscous": None, "total": None}

   groups = re.findall(r"\(([^)]*)\)", s)
   if len(groups) >= 3:
      g0 = _v66_nums(groups[0])
      g1 = _v66_nums(groups[1])
      g2 = _v66_nums(groups[2])
      if len(g0) >= 1 and len(g1) >= 1 and len(g2) >= 1:
         return {"pressure": g0[0], "viscous": g1[0], "total": g2[0]}

   nums = _v66_nums(s)
   if len(nums) >= 3:
      return {"pressure": nums[0], "viscous": nums[1], "total": nums[2]}

   return {"pressure": None, "viscous": None, "total": None}


def _v66_context_has_force_header(lines):
   text = "\n".join([str(x).lower() for x in lines])
   return ("pressure" in text and "viscous" in text and "total" in text)


def parse_wall_forces_transcript(text, report_zones=None):
   """
   v66 解析规则：
   只解析 Results -> Reports -> Forces 输出表中的 Net 行。
   对于这行：
      Net 243.42879 7.4332302 250.86202 397.43475 12.135886 409.57064
   直接取：
      第 1 个数 = 压差阻力
      第 2 个数 = 摩擦阻力
      第 3 个数 = 总阻力
   """
   result = {"pressure": None, "viscous": None, "total": None}

   if text is None:
      return result

   lines = str(text).splitlines()

   # 1. 优先最后一个 Forces - Direction Vector 块
   direction_indices = []
   for i, line in enumerate(lines):
      low = str(line).lower()
      if "forces" in low and "direction" in low and "vector" in low:
         direction_indices.append(i)

   for idx in reversed(direction_indices):
      block = lines[idx:min(len(lines), idx + 260)]
      if not _v66_context_has_force_header(block[:100]):
         continue
      for line in block:
         if str(line).strip().lower().startswith("net"):
            parsed = _v66_parse_net_line(line)
            if parsed.get("pressure") is not None:
               return parsed

   # 2. 兜底全矢量 Forces 块
   forces_indices = []
   for i, line in enumerate(lines):
      if str(line).strip().lower() == "forces":
         forces_indices.append(i)

   for idx in reversed(forces_indices):
      block = lines[idx:min(len(lines), idx + 300)]
      if not _v66_context_has_force_header(block[:120]):
         continue
      for line in block:
         if str(line).strip().lower().startswith("net"):
            parsed = _v66_parse_net_line(line)
            if parsed.get("pressure") is not None:
               return parsed

   return result


def run_wall_forces_transcript(solver_session, report_zones, transcript_name):
   """
   v65：调用 Results -> Reports -> Forces，并从 Net 行读取：
      Pressure = 第 1 个数
      Viscous  = 第 2 个数
      Total    = 第 3 个数

   不影响其它输出；失败时保存原始文本便于检查。
   """
   if not report_zones:
      return {"pressure": None, "viscous": None, "total": None, "raw": "", "command": "no zones", "zones": report_zones}

   report_zones = resolve_force_report_zones_for_solver(solver_session, report_zones)
   base_transcript_path = os.path.join(WORK_DIR, transcript_name).replace("/", os.sep)
   raw_out_path = base_transcript_path.replace(".trn", ".out")

   raw = ""
   command_used = "not parsed"

   try:
      raw = run_forces_panel_by_gui_macro(
         solver_session=solver_session,
         report_zones=report_zones,
         transcript_path=base_transcript_path,
      )
   except Exception as e:
      raw += f"\n[v65 GUI macro exception] {e}\n"
      print(f"   v65 GUI macro Forces 异常: {e}")

   parsed = parse_wall_forces_transcript(raw, report_zones=report_zones)

   if parsed.get("pressure") is not None:
      command_used = "v65-gui-results-reports-forces-net-row"
   else:
      # 保留原 TUI 兜底，但不让它影响其它流程。
      print("   v65 GUI 捕获未解析到 Net 行，尝试 TUI 兜底。")
      try:
         frp_name = os.path.basename(base_transcript_path.replace(".trn", ".frp"))
         tui_raw = run_forces_panel_by_ti_menu_string(
            solver_session=solver_session,
            report_zones=report_zones,
            frp_name=frp_name,
            transcript_path=base_transcript_path,
         )
         raw += "\n\n===== v65 TUI fallback =====\n" + tui_raw
         parsed = parse_wall_forces_transcript(raw, report_zones=report_zones)
         if parsed.get("pressure") is not None:
            command_used = "v65-tui-fallback-forces-net-row"
      except Exception as e:
         raw += f"\n[v65 TUI fallback exception] {e}\n"
         print(f"   v65 TUI Forces 兜底异常: {e}")

   try:
      with open(raw_out_path, "w", encoding="utf-8", errors="ignore") as f:
         f.write(raw)
   except Exception as e:
      print(f"   v65 Forces 原始输出保存失败: {e}")

   if parsed.get("pressure") is not None:
      parsed["raw"] = raw
      parsed["command"] = command_used
      parsed["zones"] = report_zones
      print(
         f"   Results-Report-Forces 成功: "
         f"Pressure={parsed.get('pressure')}, Viscous={parsed.get('viscous')}, Total={parsed.get('total')}"
      )
      print(f"   Forces 原始输出已保存: {raw_out_path}")
      return parsed

   print("   Results-Report-Forces 已执行，但未在捕获文本中解析到 Net 行。")
   print(f"   Forces 原始输出已保存，请检查: {raw_out_path}")

   return {
      "pressure": None,
      "viscous": None,
      "total": None,
      "raw": raw,
      "command": "v65 no real Net row captured",
      "zones": report_zones,
   }



def compute_xoy_static_pressure_report(solver_session, output_file):
   """
   输出 xoy 截面的面积加权平均静压。
   """
   commands = [
      "/report/surface-integrals/area-weighted-avg pressure xoy ()",
      "/report/surface-integrals/area-weighted-average pressure xoy ()",
      "/report/surface-integrals/area-weighted-avg static-pressure xoy ()",
      "/report/surface-integrals/area-weighted-average static-pressure xoy ()",
   ]

   best_val = None
   tried = []

   for i, cmd in enumerate(commands, start=1):
      tried.append(cmd)
      try:
         val = run_tui_report_to_file(
            solver_session,
            [cmd],
            output_file,
            f"tmp_xoy_static_pressure_{i}.trn",
         )
         if val is not None:
            best_val = val
            break
      except Exception:
         pass

   write_scalar_report_out(
      file_path=output_file,
      title=REPORT_XOY_STATIC_PRESSURE_NAME,
      value=best_val,
      extra_lines=["source = surface_integral_area_weighted_avg"] + tried,
   )

   return best_val


def compute_drag_components_and_pressure_reports(solver_session, report_zones, report_files):
   """
   用 Results -> Reports -> Forces 获取压差阻力和摩擦阻力。

   GUI 对应：
      Results
      Reports
      Forces
      Options = Forces
      Direction Vector = (1, 0, 0)
      Wall Zones = 所有 AUV 表面 zones

   输出：
      force_drag_pressure_v*.out  压差阻力
      force_drag_friction_v*.out  摩擦阻力
      tmp_results_report_forces_x.out  原始 Forces 输出
   """
   results = {
      "drag_pressure": None,
      "drag_friction": None,
      "drag_total_from_forces_panel": None,
      "xoy_static_pressure": None,
   }

   report_zones = resolve_force_report_zones_for_solver(solver_session, report_zones)

   print(f"   ▶ Results-Report-Forces wall zones = {report_zones}")
   print("   ▶ Results-Report-Forces Options = Forces")
   print("   ▶ Results-Report-Forces Direction Vector = (1, 0, 0)")

   wall_result = run_wall_forces_transcript(
      solver_session=solver_session,
      report_zones=report_zones,
      transcript_name="tmp_results_report_forces_x.trn",
   )

   results["drag_pressure"] = wall_result.get("pressure")
   results["drag_friction"] = wall_result.get("viscous")
   results["drag_total_from_forces_panel"] = wall_result.get("total")

   raw_text = wall_result.get("raw", "")

   write_scalar_report_out(
      file_path=report_files[REPORT_DRAG_PRESSURE_NAME],
      title=REPORT_DRAG_PRESSURE_NAME,
      value=results["drag_pressure"],
      extra_lines=[
         "source = Results-Report-Forces",
         "options = Forces",
         "component = Pressure column, X direction",
         "direction_vector = 1 0 0",
         f"wall_zones = {report_zones}",
         f"successful_variant = {wall_result.get('command', 'not parsed')}",
         raw_text,
      ],
   )

   write_scalar_report_out(
      file_path=report_files[REPORT_DRAG_FRICTION_NAME],
      title=REPORT_DRAG_FRICTION_NAME,
      value=results["drag_friction"],
      extra_lines=[
         "source = Results-Report-Forces",
         "options = Forces",
         "component = Viscous column, X direction",
         "direction_vector = 1 0 0",
         f"wall_zones = {report_zones}",
         f"successful_variant = {wall_result.get('command', 'not parsed')}",
         raw_text,
      ],
   )

   try:
      results["xoy_static_pressure"] = compute_xoy_static_pressure_report(
         solver_session=solver_session,
         output_file=report_files[REPORT_XOY_STATIC_PRESSURE_NAME],
      )
   except Exception:
      results["xoy_static_pressure"] = None

   return results




# ========================================================
# 6. 主流程
# ========================================================

meshing_session = None
solver_session = None

# ========================================================
# 5.10 最终修正版：表面静压报告、xlsx、报告读取
# ========================================================

def extract_latest_force_value(file_path):
   """从 Fluent report/out 文件中读取最后一个有效数值。"""
   import os
   import re
   if not file_path:
      return None
   local_path = str(file_path).replace("/", os.sep)
   if not os.path.exists(local_path):
      return None
   try:
      with open(local_path, "r", encoding="utf-8", errors="ignore") as f:
         content = f.read()
   except Exception:
      return None
   nums = re.findall(r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?", content)
   if not nums:
      return None
   try:
      return float(nums[-1])
   except Exception:
      return None


def _xlsx_col_name(idx):
   name = ""
   while idx > 0:
      idx, rem = divmod(idx - 1, 26)
      name = chr(65 + rem) + name
   return name


def _xlsx_escape(value):
   import html
   if value is None:
      return ""
   return html.escape(str(value), quote=True)


def write_simple_xlsx(xlsx_path, rows, sheet_name="summary"):
   """不依赖 openpyxl 的最小 xlsx 写入函数。"""
   import os
   import zipfile
   if not xlsx_path.lower().endswith(".xlsx"):
      xlsx_path += ".xlsx"
   folder = os.path.dirname(xlsx_path)
   if folder and not os.path.exists(folder):
      os.makedirs(folder)

   sheet_xml = [
      '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>',
      '<worksheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main">',
      '<sheetData>'
   ]
   for r_idx, row in enumerate(rows, start=1):
      sheet_xml.append(f'<row r="{r_idx}">')
      for c_idx, value in enumerate(row, start=1):
         cell_ref = f"{_xlsx_col_name(c_idx)}{r_idx}"
         if isinstance(value, (int, float)) and not isinstance(value, bool):
            sheet_xml.append(f'<c r="{cell_ref}"><v>{value}</v></c>')
         else:
            sheet_xml.append(f'<c r="{cell_ref}" t="inlineStr"><is><t>{_xlsx_escape(value)}</t></is></c>')
      sheet_xml.append('</row>')
   sheet_xml.extend(['</sheetData>', '</worksheet>'])

   content_types = """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
<Default Extension="xml" ContentType="application/xml"/>
<Override PartName="/xl/workbook.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml"/>
<Override PartName="/xl/worksheets/sheet1.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/>
</Types>"""
   rels = """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="xl/workbook.xml"/>
</Relationships>"""
   workbook = f"""<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<workbook xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main" xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships">
<sheets><sheet name="{_xlsx_escape(sheet_name)}" sheetId="1" r:id="rId1"/></sheets>
</workbook>"""
   workbook_rels = """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet" Target="worksheets/sheet1.xml"/>
</Relationships>"""
   with zipfile.ZipFile(xlsx_path, "w", zipfile.ZIP_DEFLATED) as z:
      z.writestr("[Content_Types].xml", content_types)
      z.writestr("_rels/.rels", rels)
      z.writestr("xl/workbook.xml", workbook)
      z.writestr("xl/_rels/workbook.xml.rels", workbook_rels)
      z.writestr("xl/worksheets/sheet1.xml", "\n".join(sheet_xml))
   return xlsx_path


def create_surface_pressure_reports(solver_session, report_zones, report_files):
   """
   最终稳定版：只用 Fluent 允许的 report-type：
      surface-vertexavg
      surface-vertexmax
   不再使用 vertex-avg / vertex-max，避免 Fluent 直接断开连接。
   """
   report_zones = resolve_force_report_zones_for_solver(solver_session, report_zones)
   if len(report_zones) == 0:
      print("   没有找到 AUV 表面 zone，表面静压报告暂不创建。")
      return []

   zone_expr = " ".join(report_zones)
   configs = [
      (REPORT_SURFACE_PRESSURE_AVG_NAME, "surface-vertexavg", report_files[REPORT_SURFACE_PRESSURE_AVG_NAME]),
      (REPORT_SURFACE_PRESSURE_MAX_NAME, "surface-vertexmax", report_files[REPORT_SURFACE_PRESSURE_MAX_NAME]),
   ]

   created = []
   for name, report_type, report_file in configs:
      print(f"   ▶ 创建表面静压报告: {name} | report-type={report_type} | field=pressure | zones={report_zones}")
      cmds = [
         f"/solve/report-files/delete {name}",
         f"/solve/report-definitions/delete {name}",
         f"/solve/report-definitions/add {name} surface report-type {report_type} field pressure surface-names {zone_expr} () quit",
         f"/solve/report-files/add {name} report-defs {name} () file-name \"{report_file}\" frequency 1 quit",
      ]
      jou = os.path.join(WORK_DIR, f"setup_{name}.jou")
      try:
         run_journal(solver_session, jou, cmds, f"创建表面静压报告 {name}")
         created.append(name)
      except Exception as e:
         print(f"   表面静压报告创建失败: {name} | {e}")
   print(f"   ▶ 表面静压报告创建结果: {created}")
   return created

# ========================================================
# 融合流程主执行：Fluent Meshing 到 Write Mesh 为止
# ========================================================


# ========================================================
# v116：独立 Solver Session 连接函数
# ========================================================

def launch_fresh_solver_from_mesh_v116(mesh_path, meshing_session=None):
   """
   v116 的唯一结构性修改。

   原来的：
      meshing_session.switch_to_solver()

   改成：
      1. 确认 .msh.h5 已写出；
      2. 关闭 Meshing Fluent，释放 Workflow/Field Mesher 内存和状态；
      3. 新启动一个 Fluent Solver；
      4. 读取刚生成的 mesh；
      5. 后续完全进入用户参考代码原有 Solver 流程。

   不修改：
      - 材料设置；
      - inlet/outlet/symmetry；
      - report zones；
      - Force Report；
      - 初始化策略；
      - iterate(100)；
      - 多速度循环；
      - 后处理。
   """
   local_mesh_path = str(mesh_path)
   fluent_mesh_path = normalize_path_for_fluent(local_mesh_path)

   if not os.path.exists(local_mesh_path):
      raise RuntimeError(
         "v116 无法启动 Solver：Meshing 输出网格不存在: " + local_mesh_path
      )

   mesh_size = os.path.getsize(local_mesh_path)
   if mesh_size <= 0:
      raise RuntimeError(
         "v116 无法启动 Solver：Meshing 输出网格为空文件: " + local_mesh_path
      )

   print("=" * 70)
   print(" v116 Meshing -> Fresh Solver")
   print("=" * 70)
   print(f"   已确认 mesh 文件: {local_mesh_path}")
   print(f"   mesh 文件大小: {mesh_size} bytes")
   print(f"   Fluent 读取路径: {fluent_mesh_path}")

   # --------------------------------------------------------
   # A. 先关闭 Meshing Session
   # --------------------------------------------------------
   if CLOSE_MESHING_BEFORE_SOLVER and meshing_session is not None:
      print("   正在关闭 Fluent Meshing，以释放 Meshing Workflow/Field Mesher 状态...")
      try:
         meshing_session.exit()
         print("   ✓ Fluent Meshing 已关闭。")
      except Exception as e:
         print(f"   Meshing 正常 exit 返回异常: {e}")
         print("   将继续尝试启动独立 Solver。")

      time.sleep(3)

   # --------------------------------------------------------
   # B. 新启动一个干净 Solver
   # --------------------------------------------------------
   print(f"   正在启动独立 Fluent Solver | cores={PROCESSOR_COUNT} ...")
   solver_session = pyfluent.launch_fluent(
      mode="solver",
      ui_mode="gui",
      precision="double",
      processor_count=PROCESSOR_COUNT,
   )

   time.sleep(float(FRESH_SOLVER_STARTUP_WAIT_SEC))
   print("   ✓ 独立 Fluent Solver 已启动。")

   # --------------------------------------------------------
   # C. 读取 mesh
   # --------------------------------------------------------
   read_ok = False
   errors = []

   # 优先 settings/file API
   try:
      solver_session.file.read(
         file_type="mesh",
         file_name=fluent_mesh_path,
      )
      read_ok = True
      print("   ✓ 已通过 solver_session.file.read() 读取 mesh。")
   except Exception as e:
      errors.append("file.read: " + str(e))

   # Fluent 2024 R1 兜底：TUI read-mesh
   if not read_ok:
      try:
         solver_session.tui.file.read_mesh(fluent_mesh_path)
         read_ok = True
         print("   ✓ 已通过 Solver TUI read-mesh 读取 mesh。")
      except Exception as e:
         errors.append("tui.file.read_mesh: " + str(e))

   # 最后兜底：journal
   if not read_ok:
      try:
         jou_path = os.path.join(
            WORK_DIR,
            "v116_read_mesh_in_fresh_solver.jou",
         )
         with open(jou_path, "w", encoding="utf-8") as f:
            f.write(f'/file/read-mesh "{fluent_mesh_path}"\n')

         solver_session.tui.file.read_journal(
            normalize_path_for_fluent(jou_path)
         )
         read_ok = True
         print("   ✓ 已通过 journal 读取 mesh。")
      except Exception as e:
         errors.append("journal: " + str(e))

   if not read_ok:
      try:
         solver_session.exit()
      except Exception:
         pass

      raise RuntimeError(
         "v116 独立 Solver 启动成功，但 mesh 读取失败。"
         + " | ".join(errors)
      )

   time.sleep(float(FRESH_SOLVER_AFTER_READ_MESH_WAIT_SEC))

   # --------------------------------------------------------
   # D. 只做连接诊断，不改变用户求解设置
   # --------------------------------------------------------
   try:
      zone_names = safe_get_solver_zone_names(solver_session)
      print(f"   Fresh Solver zone 列表 = {zone_names}")
   except Exception as e:
      print(f"   Fresh Solver zone 列表读取失败: {e}")

   print("=" * 70)
   print(" v116 独立 Solver 已准备完成，下面进入用户参考代码原求解流程")
   print("=" * 70)

   return solver_session


if __name__ == "__main__":
   run_manifest = build_initial_result_manifest()
   write_result_manifest(run_manifest)
   print("=" * 70)
   print(" UUV CFD Tool 运行参数")
   print(f" model       = {INPUT_MODEL}")
   print(f" workdir     = {WORK_DIR}")
   print(f" velocities  = {VELOCITY_LIST}")
   print(f" cores       = {PROCESSOR_COUNT}")
   print(f" iterations  = {ITERATIONS}")
   print(f" contours    = {SAVE_CONTOUR_IMAGES}")
   print(f" keep_open   = {KEEP_FLUENT_OPEN}")
   print("=" * 70)
   meshing_session = None
   solver_session = None
   try:
      meshing_session = pyfluent.launch_fluent(
         fluent_path=FLUENT_EXE_PATH,
         mode="meshing",
         ui_mode="gui",
         processor_count=PROCESSOR_COUNT,
         precision="double",
         cwd=WORK_DIR,
      )

      print("\n Fluent Meshing 已连接。等待界面加载...")
      time.sleep(8)

      workflow = meshing_session.workflow
      workflow.InitializeWorkflow(WorkflowType="Watertight Geometry")
      time.sleep(2)

      # ------------- 网格阶段 -------------
      print("\n [1/17] 导入几何...")
      apply_task_state(
         workflow.TaskObject["Import Geometry"],
         {"FileName": GEOMETRY_PATH, "LengthUnit": "mm"},
         "Import Geometry",
      )
      workflow.TaskObject["Import Geometry"].Execute()

      print("\n [2/17] 添加局部尺寸...")
      AUV_SURFACE_LABELS = read_auv_surface_labels()
      LOCAL_FACE_SIZINGS = build_local_face_sizings(AUV_SURFACE_LABELS)

      if len(LOCAL_FACE_SIZINGS) == 0:
         print("   没有生成局部尺寸规则；将使用全局面网格尺寸。请检查 geometry metrics。")

      for sizing in LOCAL_FACE_SIZINGS:
         add_face_local_sizing(
            workflow,
            sizing["name"],
            sizing["labels"],
            float(sizing["size"]),
         )

      if ENABLE_LOCAL_REFINEMENT_REGIONS:
         print("\n [2.5/17] 创建自适应 Local Refinement Regions 两级体加密缓冲...")
         build_and_apply_adaptive_refinement_regions(workflow)
      else:
         print("\n [2.5/17] v241 稳定模式：跳过 Local Refinement Regions（避免 Cortex segmentation violation）")

      print("\n [3/17] 生成面网格...")
      set_surface_mesh_controls(workflow)
      workflow.TaskObject["Generate the Surface Mesh"].Execute()

      print("\n [3.5/17] Surface Mesh 预改善...")
      improve_surface_mesh_if_available(workflow)

      print("\n [4/17] 描述几何结构...")
      describe_geometry_with_voids(workflow)

      print("\n [5/17] 更新边界条件...")
      workflow.TaskObject["Update Boundaries"].Execute()

      print("\n [6/17] 更新区域类型 fluid/dead...")
      update_regions_fluid_dead(workflow)

      print("\n [7/17] 添加边界层...")
      add_boundary_layers(workflow)

      print(f"\n [8/17] 生成体网格 ({VOL_FILL_TYPE})...")
      set_volume_mesh_controls(workflow)
      workflow.TaskObject["Generate the Volume Mesh"].Execute()

      print("\n [8.5/17] 改进网格质量...")
      improve_volume_mesh_quality_after_generation(meshing_session, workflow)

      print("\n [9/17] 导出网格...")
      # v114：Fluent TUI 必须使用 /，否则 D:\zidonghua\new2 中的 \n 会被解释为换行。
      # 时间戳精确到秒，且写入前删除同名文件，彻底避免 overwrite 交互。
      mesh_save_path = os.path.join(WORK_DIR, f"mesh_{time.strftime('%m%d_%H%M%S')}.msh.h5")
      mesh_save_path = safe_write_mesh_v241(meshing_session, mesh_save_path)
      print(f"   网格已保存并确认: {mesh_save_path}")

      # ------------- v116：Meshing -> 独立 Solver -------------
      print("\n [10/17] 启动独立 Solver 并读取 Meshing 网格...")

      if USE_FRESH_SOLVER_SESSION:
         solver_session = launch_fresh_solver_from_mesh_v116(
            mesh_path=mesh_save_path,
            meshing_session=meshing_session,
         )
         meshing_session = None
      else:
         # 保留原参考代码连接方式作为人工回退开关。
         print("   USE_FRESH_SOLVER_SESSION=False，使用原 switch_to_solver()。")
         solver_session = meshing_session.switch_to_solver()
         meshing_session = None
         time.sleep(5)

      print("   ✓ 已切换到 Fluent Solver。")
      try:
         _zones_after_switch = safe_get_solver_zone_names(solver_session)
         print(f"   Solver zone 列表 = {_zones_after_switch}")
      except Exception as _e:
         print(f"   Solver 已切换，但读取 zone 列表失败（继续后续设置）: {_e}")

      # ------------- Solver 阶段 -------------
      print("\n [11/17] 设置材料、边界条件、报告与截面 Journal...")

      velocity_inlet_zones = get_bc_names(solver_session, "velocity_inlet")
      pressure_outlet_zones = get_bc_names(solver_session, "pressure_outlet")
      wall_zones_before = get_bc_names(solver_session, "wall")
      symmetry_zones_before = get_bc_names(solver_session, "symmetry")

      all_bc_zones = []
      for group in [velocity_inlet_zones, pressure_outlet_zones, wall_zones_before, symmetry_zones_before, safe_get_solver_zone_names(solver_session)]:
         for z in group:
            if z not in all_bc_zones:
               all_bc_zones.append(z)

      inlet_zones = find_zones_by_labels(velocity_inlet_zones + all_bc_zones, ["inlet"])
      outlet_zones = find_zones_by_labels(pressure_outlet_zones + all_bc_zones, ["outlet"])
      outer_wall_zones = find_zones_by_labels(wall_zones_before + all_bc_zones, OUTER_WALL_LABELS)
      report_labels = surface_labels_from_settings()
      report_zones = find_zones_by_labels(wall_zones_before + all_bc_zones, report_labels)

      print(f"   ▶ velocity-inlet zones = {velocity_inlet_zones}")
      print(f"   ▶ pressure-outlet zones = {pressure_outlet_zones}")
      print(f"   ▶ wall zones = {wall_zones_before}")
      print(f"   ▶ inlet target zones = {inlet_zones}")
      print(f"   ▶ outlet target zones = {outlet_zones}")
      print(f"   ▶ outer wall/symmetry target zones = {outer_wall_zones}")
      print(f"   ▶ force report labels = {report_labels}")
      print(f"   ▶ force report wall zones = {report_zones}")

      # ========================================================
      # Solver 一次性基础设置：截面、材料、边界类型等
      # ========================================================
      journal_cmds = []

      for plane in PLANES_TO_CREATE:
         journal_cmds.append(f'/surface/plane-surface {plane["name"]} {plane["method"]} {plane["coord"]}')

      journal_cmds.append(f'/define/materials/copy fluid {TARGET_MATERIAL}')

      for z in outer_wall_zones:
         journal_cmds.append(f'/define/boundary-conditions/modify-zones/zone-type {z} symmetry')

      for z in inlet_zones:
         journal_cmds.append(f'/define/boundary-conditions/modify-zones/zone-type {z} velocity-inlet')

      for z in outlet_zones:
         journal_cmds.append(f'/define/boundary-conditions/modify-zones/zone-type {z} pressure-outlet')

      jou_file_path = write_and_read_journal(solver_session, journal_cmds, "setup_solver_base.jou")
      print(f"   求解器基础 journal 已执行: {jou_file_path}")

      try:
         all_bc_zones_after_setup = []
         for attr in ["wall", "symmetry", "velocity_inlet", "pressure_outlet"]:
            all_bc_zones_after_setup.extend(get_bc_names(solver_session, attr))
         report_zones_after_setup = find_zones_by_labels(all_bc_zones_after_setup, report_labels)
         if len(report_zones_after_setup) > 0:
            report_zones = report_zones_after_setup
      except Exception:
         pass

      print("\n [12/17] 将流体域挂载真实材料...")
      try:
         cell_zones = list(solver_session.setup.cell_zone_conditions.fluid.keys())
         for cz in cell_zones:
            try:
               solver_session.setup.cell_zone_conditions.fluid[cz].material = TARGET_MATERIAL
               print(f"   {cz} material = {TARGET_MATERIAL}")
            except Exception as e:
               print(f"   {cz} material 设置失败: {e}")
      except Exception as e:
         print(f"   自动赋材料失败: {e}")

      print("\n [13/17] 截面检查...")
      for plane in PLANES_TO_CREATE:
         print(f"   ▶ 已尝试创建截面: {plane['name']}")

      print("\n [13.5/17] 设置残差收敛标准...")
      try:
         set_residual_convergence_criteria(solver_session)
      except Exception as e:
         print(f"   残差收敛标准设置失败，但继续计算: {e}")

      print("\n [14/17] 创建 Velocity Magnitude 云图对象...")
      for c in CONTOURS_TO_CREATE:
         try:
            create_or_update_contour_for_picture(
               solver_session=solver_session,
               contour_name=c["name"],
               field_name=c["field"],
               surface_name=c["surface"],
            )
            print(f"   已创建/更新云图对象: {c['name']} | field={c['field']} | surface={c['surface']}")
         except Exception as e:
            print(f"   云图 {c['name']} 创建失败: {e}")

      all_force_rows = []

      print("\n [15/17] 开始多速度循环计算...")
      print(f"   ▶ 本次速度列表 VELOCITY_LIST = {VELOCITY_LIST}")

      for case_idx, velocity in enumerate(VELOCITY_LIST, start=1):
         INLET_VELOCITY = float(velocity)
         tag = speed_tag(INLET_VELOCITY)
         report_files = build_report_files_for_velocity(INLET_VELOCITY)

         print("=" * 70)
         print(f" 速度工况 {case_idx}/{len(VELOCITY_LIST)}: U = {INLET_VELOCITY} m/s | tag = {tag}")
         print("=" * 70)

         try:
            print("\n 设置入口速度...")
            velocity_ok = set_inlet_velocity_for_case(solver_session, inlet_zones, INLET_VELOCITY)
            if not velocity_ok:
               print(f"   入口速度设置失败，跳过该速度，避免继续计算出 U=0 的错误结果。速度 = {INLET_VELOCITY} m/s")
               continue

            print("\n 创建三方向力报告...")
            created_force_reports = create_force_reports(solver_session, report_zones, report_files)
            print(f"   已尝试创建三方向力报告: {created_force_reports}")
            print(f"   X 方向力报告路径: {report_files[REPORT_NAME]}")
            print(f"   Y 方向力报告路径: {report_files[REPORT_SWAY_Y_NAME]}")
            print(f"   Z 方向力报告路径: {report_files[REPORT_HEAVE_Z_NAME]}")
            print(f"   受力面: {report_zones}")

            print("\n 初始化并迭代...")
            run_ok = initialize_and_run_case(solver_session, inlet_zones, ITERATIONS)
            if not run_ok:
               print(f"   当前速度 {INLET_VELOCITY} m/s 迭代失败或 Solver 已断开，停止后续速度循环。")
               break

            print("\n 保存当前速度的 xoy/xoz 云图图片...")
            contour_paths = []
            try:
               contour_paths = save_contour_pictures(solver_session, velocity_tag=tag) or []
            except Exception as e:
               print(f"   保存云图失败，但继续后处理和下一速度: {e}")

            print("\n 保存当前速度的 case/data...")
            case_path = os.path.join(WORK_DIR, f"try_{tag}_{time.strftime('%m%d_%H%M')}.cas.h5")
            data_path = os.path.join(WORK_DIR, f"try_{tag}_{time.strftime('%m%d_%H%M')}.dat.h5")
            try:
               solver_session.file.write(file_type="case", file_name=normalize_path_for_fluent(case_path))
               solver_session.file.write(file_type="data", file_name=normalize_path_for_fluent(data_path))
            except Exception:
               try:
                  solver_session.tui.file.write_case_data(normalize_path_for_fluent(case_path.replace(".cas.h5", ".cas")))
               except Exception as e:
                  print(f"   保存失败: {e}")

            print("\n 导出当前速度 AUV 表面 total pressure solution data...")
            total_pressure_path = None
            try:
               total_pressure_path = export_total_pressure_solution_data(
                  solver_session=solver_session,
                  report_zones=report_zones,
                  velocity=INLET_VELOCITY,
               )
            except Exception as e:
               print(f"   total pressure solution data 导出失败，但继续后处理和下一速度: {e}")

            print("\n 计算当前速度的三方向总力、压差阻力和摩擦阻力...")
            drag_components = compute_drag_pressure_friction_from_forces_panel(
               solver_session=solver_session,
               report_zones=report_zones,
               report_files=report_files,
               velocity_tag=tag,
            )

            # Excel 的 X/Y/Z 统一优先取 Results -> Reports -> Forces
            # 完整矢量表的 Net Total = (Fx, Fy, Fz)。
            total_vector = drag_components.get("force_total_vector")
            if (
               isinstance(total_vector, (list, tuple))
               and len(total_vector) >= 3
            ):
               drag_x = float(total_vector[0])
               sway_y = float(total_vector[1])
               heave_z = float(total_vector[2])
               print(
                  "   Excel 三方向力采用 Forces Net Total 矢量: "
                  f"Fx={drag_x}, Fy={sway_y}, Fz={heave_z}"
               )
            else:
               print("   未解析到完整 Forces Net 矢量，回退到三个 Report Definition。")
               drag_x = extract_force_value_robust(
                  solver_session, REPORT_NAME, report_files.get(REPORT_NAME)
               )
               sway_y = extract_force_value_robust(
                  solver_session, REPORT_SWAY_Y_NAME, report_files.get(REPORT_SWAY_Y_NAME)
               )
               heave_z = extract_force_value_robust(
                  solver_session, REPORT_HEAVE_Z_NAME, report_files.get(REPORT_HEAVE_Z_NAME)
               )

            write_scalar_report_out(report_files.get(REPORT_NAME), REPORT_NAME, drag_x)
            write_scalar_report_out(report_files.get(REPORT_SWAY_Y_NAME), REPORT_SWAY_Y_NAME, sway_y)
            write_scalar_report_out(report_files.get(REPORT_HEAVE_Z_NAME), REPORT_HEAVE_Z_NAME, heave_z)

            drag_pressure = drag_components.get("drag_pressure")
            drag_friction = drag_components.get("drag_friction")

            all_force_rows.append({
               "velocity": INLET_VELOCITY,
               "drag_x": drag_x if drag_x is not None else "not found",
               "sway_y": sway_y if sway_y is not None else "not found",
               "heave_z": heave_z if heave_z is not None else "not found",
               "drag_pressure": drag_pressure if drag_pressure is not None else "not found",
               "drag_friction": drag_friction if drag_friction is not None else "not found",
            })

            case_manifest = {
               "status": "success" if all(v is not None for v in [drag_x, sway_y, heave_z]) else "partial",
               "velocity_m_s": float(INLET_VELOCITY),
               "tag": tag,
               "forces_N": {
                  "x_total": drag_x,
                  "y_total": sway_y,
                  "z_total": heave_z,
                  "x_pressure": drag_pressure,
                  "x_friction": drag_friction,
               },
               "artifacts": {
                  "case_file": os.path.abspath(case_path),
                  "data_file": os.path.abspath(data_path),
                  "surface_total_pressure": os.path.abspath(total_pressure_path) if total_pressure_path else None,
                  "contour_images": [os.path.abspath(p) for p in contour_paths],
                  "report_files": {k: os.path.abspath(str(v).replace('/', os.sep)) for k, v in report_files.items()},
               },
            }
            run_manifest["cases"].append(case_manifest)
            write_result_manifest(run_manifest)

            for _name, _path in report_files.items():
               _local = str(_path).replace("/", os.sep)
               if os.path.exists(_local):
                  print(f"   已检测到报告文件: {_local}")
               else:
                  print(f"   未检测到报告文件: {_local}")

            try:
               write_force_summary_excel(all_force_rows)
               print("   已即时刷新 Excel 汇总表。")
            except Exception as e:
               print(f"   即时刷新 Excel 汇总表失败: {e}")

         except Exception as e:
            print("=" * 70)
            print(f" 速度工况 {INLET_VELOCITY} m/s 处理失败，但继续后续速度。")
            print(str(e))
            print("=" * 70)
            if not any(float(c.get("velocity_m_s", -1)) == float(INLET_VELOCITY) for c in run_manifest.get("cases", [])):
               run_manifest["cases"].append({
                  "status": "failed",
                  "velocity_m_s": float(INLET_VELOCITY),
                  "tag": tag,
                  "error": str(e),
                  "artifacts": {},
               })
               run_manifest["errors"].append({"velocity_m_s": float(INLET_VELOCITY), "message": str(e)})
               write_result_manifest(run_manifest)
            continue

      print("\n [17/17] 写入多速度水动力结果 Excel...")
      summary_excel_path = None
      try:
         summary_excel_path = write_force_summary_excel(all_force_rows)
      except Exception as e:
         print(f"   多速度水动力结果 Excel 写入阶段失败: {e}")
         run_manifest["errors"].append({"stage": "summary_excel", "message": str(e)})

      finalize_result_manifest(run_manifest, summary_excel=summary_excel_path)

      print("=" * 70)
      print(" Fluent 自动化流程完成")
      print(f" result_manifest = {RESULT_MANIFEST_PATH}")
      print("=" * 70)

   except Exception as e:
      print("=" * 70)
      print(" Fluent 自动化流程失败")
      print(str(e))
      print("=" * 70)
      try:
         run_manifest["status"] = "failed"
         run_manifest["completed_at"] = _iso_now()
         run_manifest["errors"].append({"stage": "pipeline", "message": str(e)})
         write_result_manifest(run_manifest)
      except Exception:
         pass
      raise

   finally:
      # 全流程完成后不关闭 Fluent，保留 Solver GUI 方便检查网格、云图和报告。
      # 不主动调用 solver_session.exit() / meshing_session.exit()。
      if KEEP_FLUENT_OPEN:
         try:
            input("\n[Fluent 已保持打开] 请在 GUI 中检查网格、速度云图、force_drag 报告和结果。按回车仅结束 Python 脚本，不主动关闭 Fluent...")
         except Exception:
            pass
      else:
         # Agent/批处理模式：不等待 input，尽量正常退出 Fluent 进程。
         try:
            if solver_session is not None:
               solver_session.exit()
         except Exception:
            pass
         try:
            if meshing_session is not None:
               meshing_session.exit()
         except Exception:
            pass
      pass
