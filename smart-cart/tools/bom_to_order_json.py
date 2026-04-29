#!/usr/bin/env python3
import argparse
import json
import re
import sys
from collections import Counter
from pathlib import Path
from zipfile import ZipFile
from xml.etree import ElementTree as ET

NS = {"x": "http://schemas.openxmlformats.org/spreadsheetml/2006/main"}


def column_index(cell_ref: str) -> int:
    letters = "".join(ch for ch in cell_ref if ch.isalpha())
    index = 0
    for ch in letters:
        index = index * 26 + ord(ch.upper()) - ord("A") + 1
    return index - 1


def normalize_header(value: str) -> str:
    return re.sub(r"[^a-z0-9]+", "", value.lower())


def read_shared_strings(book: ZipFile) -> list[str]:
    try:
        root = ET.fromstring(book.read("xl/sharedStrings.xml"))
    except KeyError:
        return []

    values = []
    for si in root.findall("x:si", NS):
        values.append("".join(t.text or "" for t in si.findall(".//x:t", NS)))
    return values


def cell_value(cell: ET.Element, shared_strings: list[str]) -> str:
    cell_type = cell.get("t")
    if cell_type == "inlineStr":
        return "".join(t.text or "" for t in cell.findall(".//x:t", NS)).strip()

    value_node = cell.find("x:v", NS)
    if value_node is None or value_node.text is None:
        return ""

    raw = value_node.text.strip()
    if cell_type == "s":
        try:
            return shared_strings[int(raw)].strip()
        except (IndexError, ValueError):
            return raw
    return raw


def read_rows(path: Path) -> list[list[str]]:
    with ZipFile(path) as book:
        shared_strings = read_shared_strings(book)
        root = ET.fromstring(book.read("xl/worksheets/sheet1.xml"))

        rows = []
        for row in root.findall(".//x:sheetData/x:row", NS):
            values: list[str] = []
            for cell in row.findall("x:c", NS):
                index = column_index(cell.get("r", "A1"))
                while len(values) <= index:
                    values.append("")
                values[index] = cell_value(cell, shared_strings)
            rows.append(values)
        return rows


def designator_count(value: str) -> int:
    total = 0
    for token in re.split(r"[,;\s]+", value.strip()):
        if not token:
            continue

        range_match = re.fullmatch(r"([A-Za-z]+)(\d+)-([A-Za-z]+)?(\d+)", token)
        if range_match:
            prefix_a, start, prefix_b, end = range_match.groups()
            if prefix_b is None or prefix_b == prefix_a:
                total += abs(int(end) - int(start)) + 1
            else:
                total += 1
            continue

        if re.fullmatch(r"[A-Za-z]+\d+", token):
            total += 1
            continue

        total += 1
    return total


def usage_priorities(quantity_by_part: Counter[str]) -> dict[str, int]:
    total = sum(max(qty, 1) for qty in quantity_by_part.values())
    cumulative = 0
    result: dict[str, int] = {}

    ordered = sorted(
        quantity_by_part.items(),
        key=lambda item: (-item[1], item[0]),
    )

    for part_number, quantity in ordered:
        share_before = cumulative / total if total else 1.0
        if share_before < 0.50:
            priority = 1
        elif share_before < 0.85:
            priority = 2
        else:
            priority = 3
        result[part_number] = priority
        cumulative += max(quantity, 1)

    return result


def build_order(input_path: Path,
                order_id: str,
                title: str,
                material_type: str) -> dict:
    rows = read_rows(input_path)
    if not rows:
        raise RuntimeError("BOM is empty")

    header = {normalize_header(value): index for index, value in enumerate(rows[0])}
    part_index = header.get("partnumber")
    designator_index = header.get("designator")
    quantity_index = header.get("quantity")

    if part_index is None:
        raise RuntimeError("Column PartNumber is required")

    quantity_by_part: Counter[str] = Counter()
    for row in rows[1:]:
        part_number = row[part_index].strip() if part_index < len(row) else ""
        if not part_number:
            continue

        quantity = 0
        if quantity_index is not None and quantity_index < len(row) and row[quantity_index].strip():
            quantity = int(float(row[quantity_index].strip()))
        elif designator_index is not None and designator_index < len(row):
            quantity = designator_count(row[designator_index])

        quantity_by_part[part_number] += max(quantity, 1)

    priorities = usage_priorities(quantity_by_part)
    ordered_parts = sorted(
        quantity_by_part.items(),
        key=lambda item: (priorities[item[0]], -item[1], item[0]),
    )

    items = []
    for slot_index, (part_number, quantity) in enumerate(ordered_parts, start=1):
        items.append({
            "barcode": part_number,
            "part_number": part_number,
            "material_type": material_type,
            "required_quantity": quantity,
            "usage_priority": priorities[part_number],
            "target_slot": slot_index,
        })

    return {
        "order_id": order_id,
        "title": title,
        "priority": "normal",
        "duration_minutes": 0,
        "items": items,
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Convert BOM xlsx with PartNumber to SmartCart order JSON."
    )
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--order-id", default=None)
    parser.add_argument("--title", default=None)
    parser.add_argument("--material-type", default="reel")
    args = parser.parse_args()

    order_id = args.order_id or args.input.stem
    title = args.title or args.input.stem
    order = build_order(args.input, order_id, title, args.material_type)

    args.output.write_text(
        json.dumps(order, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"Wrote {args.output} with {len(order['items'])} items")
    return 0


if __name__ == "__main__":
    sys.exit(main())
