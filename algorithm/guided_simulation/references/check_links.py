# -*- coding: utf-8 -*-
"""references 目录校验: 相对链接/插图引用是否存在 + 章节文件齐全 + 文献编号使用检查."""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
MD = [f for f in sorted(os.listdir(HERE)) if f.endswith(".md")]

EXPECT = ["README.md"] + [f"{i:02d}_" for i in range(1, 11)]

link_re = re.compile(r"!?\[[^\]]*\]\(([^)]+)\)")


def main():
    errors = []
    notes = []

    # 1) 文件齐全
    for pre in EXPECT:
        if not any(f.startswith(pre) if pre.endswith("_") else f == pre for f in MD):
            errors.append(f"缺少文件: {pre}*")
    notes.append(f"md 文件数: {len(MD)}")

    # 2) 插图文件齐全
    figs = set(os.listdir(os.path.join(HERE, "figures")))
    pngs = {f for f in figs if f.endswith(".png")}
    notes.append(f"PNG 插图数: {len(pngs)}")

    # 3) 逐文件链接
    total_links = 0
    used_refs = set()
    for f in MD:
        path = os.path.join(HERE, f)
        with open(path, encoding="utf-8") as fh:
            text = fh.read()
        used_refs |= set(re.findall(r"\[R\d{2}\]", text))
        for target in link_re.findall(text):
            total_links += 1
            if target.startswith(("http://", "https://", "#", "mailto:")):
                continue
            tgt = target.split("#")[0]
            if not tgt:
                continue
            full = os.path.normpath(os.path.join(HERE, tgt))
            if not os.path.exists(full):
                errors.append(f"{f}: 失效链接 -> {target}")

    notes.append(f"内部链接/插图引用总数: {total_links}")

    # 4) 文献编号: 正文中使用的必须在 README 总表中定义
    with open(os.path.join(HERE, "README.md"), encoding="utf-8") as fh:
        readme = fh.read()
    defined = set(re.findall(r"\| (R\d{2}) \|", readme))
    undefined = sorted(used_refs - {"[" + d + "]" for d in defined})
    if undefined:
        errors.append("正文引用了 README 未定义的文献: " + ", ".join(undefined))
    notes.append(f"README 定义文献 {len(defined)} 条, 正文引用 {len(used_refs)} 条")

    # 5) 每章都有「动手实验」和「本章文献」
    for f in MD:
        if f == "README.md":
            continue
        with open(os.path.join(HERE, f), encoding="utf-8") as fh:
            text = fh.read()
        if "动手实验" not in text:
            errors.append(f"{f}: 缺少「动手实验」小节")
        if "本章文献" not in text:
            errors.append(f"{f}: 缺少「本章文献」小节")

    print("== 校验结果 ==")
    for n in notes:
        print("  ·", n)
    if errors:
        for e in errors:
            print("  ✗", e)
        print(f"FAILED ({len(errors)})")
        return 1
    print("  全部通过 OK")
    return 0


sys.exit(main())
