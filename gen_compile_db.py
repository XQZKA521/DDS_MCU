"""
一键生成 compile_commands.json
用法：在工程根目录下运行  python gen_compile_db.py
"""
import os, json, glob

DIR = os.path.dirname(os.path.abspath(__file__)).replace("\\", "/")
CC  = "C:/Software/Keil_v5/ARM/ARMCLANG/Bin/armclang.exe"
CFLAGS = [
    CC, "--target=arm-arm-none-eabi", "-mcpu=cortex-m0plus", "-std=c99",
    "-D__MSPM0G3519__",
    "-I.", "-IProject", "-IUser", "-IBSP", "-IBSP/delay",
    "-ISource", "-ISource/third_party/CMSIS/Core/Include",
    "-IC:/Software/Keil_v5/ARM/ARMCLANG/include",
    "-c",
]

db = []
for pat in ("User/*.c", "Project/*.c", "BSP/**/*.c"):
    for f in glob.glob(os.path.join(DIR, pat), recursive=True):
        rel = os.path.relpath(f, DIR).replace("\\", "/")
        db.append({"directory": DIR, "file": f.replace("\\", "/"),
                    "arguments": CFLAGS + [rel]})

out = os.path.join(DIR, "compile_commands.json")
with open(out, "w") as fp:
    json.dump(db, fp, indent=2)
print(f"Done – {len(db)} entries written to {out}")
