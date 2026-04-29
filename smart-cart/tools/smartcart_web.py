#!/usr/bin/env python3
import argparse
import html
import json
import sqlite3
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional
from urllib.parse import urlparse


HTML_PAGE = r"""<!doctype html>
<html lang="ru">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>SmartCart</title>
  <style>
    :root {
      --bg: #eef5f0;
      --panel: #ffffff;
      --text: #102217;
      --muted: #5b6c60;
      --green: #2f8f57;
      --green-dark: #183b28;
      --line: #c8d8cc;
      --free: #e6ece8;
      --occupied: #d7e9f7;
      --reserved: #f6e7b6;
      --error: #f2c3c3;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: var(--bg);
      color: var(--text);
    }
    header {
      background: var(--green-dark);
      color: white;
      padding: 18px 22px;
      display: flex;
      justify-content: space-between;
      gap: 20px;
      align-items: center;
    }
    h1, h2, h3 { margin: 0; letter-spacing: 0; }
    h1 { font-size: 26px; }
    h2 { font-size: 18px; margin-bottom: 10px; }
    h3 { font-size: 15px; }
    .status-line {
      color: #cfe4d4;
      font-size: 14px;
      text-align: right;
    }
    main {
      padding: 16px;
      display: grid;
      grid-template-columns: minmax(360px, 1.2fr) minmax(360px, 0.8fr);
      gap: 14px;
    }
    section {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 14px;
    }
    .overview {
      display: grid;
      grid-template-columns: repeat(4, minmax(120px, 1fr));
      gap: 10px;
      grid-column: 1 / -1;
    }
    .modules-section {
      grid-column: 1 / -1;
    }
    .metric {
      background: #f7faf8;
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 12px;
    }
    .metric span {
      display: block;
      color: var(--muted);
      font-size: 13px;
      margin-bottom: 5px;
    }
    .metric strong {
      font-size: 18px;
      line-height: 1.25;
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(6, minmax(92px, 1fr));
      gap: 8px;
    }
    .slot {
      min-height: 92px;
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 8px;
      display: flex;
      flex-direction: column;
      justify-content: space-between;
      overflow: hidden;
    }
    .slot.free { background: var(--free); }
    .slot.occupied { background: var(--occupied); }
    .slot.reserved { background: var(--reserved); }
    .slot.error { background: var(--error); }
    .slot-number {
      font-weight: 800;
      font-size: 18px;
    }
    .slot-state {
      color: var(--muted);
      font-size: 12px;
      text-transform: uppercase;
    }
    .slot-barcode {
      font-size: 12px;
      overflow-wrap: anywhere;
    }
    table {
      width: 100%;
      border-collapse: collapse;
      font-size: 14px;
    }
    th, td {
      text-align: left;
      padding: 8px;
      border-bottom: 1px solid #e1e9e3;
      vertical-align: top;
    }
    th {
      color: var(--muted);
      font-size: 12px;
      text-transform: uppercase;
    }
    .pill {
      display: inline-block;
      padding: 3px 8px;
      border-radius: 999px;
      font-weight: 700;
      font-size: 12px;
      border: 1px solid var(--line);
      background: #f7faf8;
    }
    .p1 { color: #8f2f2f; border-color: #e0b5b5; background: #fff3f3; }
    .p2 { color: #8a641f; border-color: #e8d199; background: #fff8e6; }
    .p3 { color: #2f6f49; border-color: #b8dac5; background: #effaf3; }
    .events {
      display: flex;
      flex-direction: column;
      gap: 8px;
      max-height: 420px;
      overflow: auto;
    }
    .modules {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(170px, 1fr));
      gap: 8px;
    }
    .module-card {
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 10px;
      background: #f7faf8;
    }
    .module-card.online {
      border-color: #91caa5;
      background: #eefaf2;
    }
    .module-card.offline {
      opacity: 0.72;
    }
    .module-card strong {
      display: block;
      margin-bottom: 4px;
      overflow-wrap: anywhere;
    }
    .module-card span {
      display: block;
      color: var(--muted);
      font-size: 13px;
    }
    .event {
      border-bottom: 1px solid #e1e9e3;
      padding-bottom: 7px;
    }
    .event code {
      font-weight: 800;
      color: var(--green-dark);
      background: transparent;
    }
    .event small {
      color: var(--muted);
      display: block;
      margin-bottom: 2px;
    }
    .empty {
      color: var(--muted);
      padding: 10px 0;
    }
    @media (max-width: 980px) {
      main { grid-template-columns: 1fr; }
      .overview { grid-template-columns: repeat(2, minmax(120px, 1fr)); }
      .grid { grid-template-columns: repeat(3, minmax(88px, 1fr)); }
    }
  </style>
</head>
<body>
  <header>
    <div>
      <h1>SmartCart</h1>
      <div id="subtitle">загрузка...</div>
    </div>
    <div class="status-line">
      <div id="dbPath"></div>
      <div id="updatedAt"></div>
    </div>
  </header>
  <main>
    <div class="overview">
      <div class="metric"><span>Модуль</span><strong id="module">-</strong></div>
      <div class="metric"><span>Маршрут</span><strong id="workflow">-</strong></div>
      <div class="metric"><span>Заказ</span><strong id="order">-</strong></div>
      <div class="metric"><span>Прогресс</span><strong id="progress">-</strong></div>
    </div>

    <section class="modules-section">
      <h2>Модули</h2>
      <div id="modules" class="modules"></div>
    </section>

    <section>
      <h2>Слоты</h2>
      <div id="slots" class="grid"></div>
    </section>

    <section>
      <h2>Материалы заказа</h2>
      <div id="items"></div>
    </section>

    <section>
      <h2>Активные катушки</h2>
      <div id="reels"></div>
    </section>

    <section>
      <h2>Последние события</h2>
      <div id="events" class="events"></div>
    </section>
  </main>
  <script>
    const stateClass = (state) => {
      const value = String(state || '').toLowerCase();
      if (value.includes('occupied')) return 'occupied';
      if (value.includes('reserved')) return 'reserved';
      if (value.includes('error')) return 'error';
      return 'free';
    };
    const esc = (value) => String(value ?? '').replace(/[&<>"']/g, (ch) => ({
      '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#039;'
    }[ch]));

    async function load() {
      const response = await fetch('/api/state', { cache: 'no-store' });
      const data = await response.json();
      if (!data.ok) {
        document.getElementById('subtitle').textContent = data.error || 'ошибка';
        return;
      }

      document.getElementById('subtitle').textContent = 'панель наблюдения';
      document.getElementById('dbPath').textContent = data.db_path;
      document.getElementById('updatedAt').textContent = 'обновлено: ' + data.generated_at;
      document.getElementById('module').textContent =
        data.module ? `${data.module.serial} / ${data.module.status}` : 'нет данных';
      document.getElementById('workflow').textContent =
        data.workflow ? data.workflow.state : 'нет данных';
      document.getElementById('order').textContent =
        data.order ? `${data.order.external_order_id} · ${data.order.title}` : 'нет заказа';
      document.getElementById('progress').textContent =
        `${data.stats.done}/${data.stats.total} · ошибок ${data.stats.wrong_slot}`;

      document.getElementById('modules').innerHTML = data.modules.length ? data.modules.map((module) => `
        <div class="module-card ${String(module.status || '').toLowerCase()}">
          <strong>${esc(module.serial)}</strong>
          <span>${esc(module.kind || 'UNKNOWN')} · ${esc(module.status)}</span>
          <span>${module.slot_count} слотов</span>
        </div>
      `).join('') : '<div class="empty">Модули не зарегистрированы</div>';

      document.getElementById('slots').innerHTML = data.slots.map((slot) => `
        <div class="slot ${stateClass(slot.state)}">
          <div>
            <div class="slot-number">${slot.slot_index}</div>
            <div class="slot-state">${esc(slot.state)}</div>
          </div>
          <div class="slot-barcode">${esc(slot.barcode || '')}</div>
        </div>
      `).join('');

      document.getElementById('items').innerHTML = data.items.length ? `
        <table>
          <thead><tr><th>Приоритет</th><th>PartNumber</th><th>Кол-во</th><th>Слот</th><th>Статус</th></tr></thead>
          <tbody>${data.items.map((item) => `
            <tr>
              <td><span class="pill p${item.usage_priority}">P${item.usage_priority}</span></td>
              <td>${esc(item.part_number || item.barcode)}</td>
              <td>${item.required_quantity}</td>
              <td>${item.target_slot}${item.current_slot ? ` / факт ${item.current_slot}` : ''}</td>
              <td>${esc(item.status)}</td>
            </tr>
          `).join('')}</tbody>
        </table>
      ` : '<div class="empty">Заказ не загружен</div>';

      document.getElementById('reels').innerHTML = data.reels.length ? `
        <table>
          <thead><tr><th>Слот</th><th>Штрихкод</th><th>Размещено</th></tr></thead>
          <tbody>${data.reels.map((reel) => `
            <tr><td>${reel.slot_index}</td><td>${esc(reel.barcode)}</td><td>${esc(reel.placed_at)}</td></tr>
          `).join('')}</tbody>
        </table>
      ` : '<div class="empty">Активных катушек нет</div>';

      document.getElementById('events').innerHTML = data.events.map((event) => `
        <div class="event">
          <small>#${event.id} · ${esc(event.ts)} · ${esc(event.level)}</small>
          <code>${esc(event.code)}</code>
          <div>${esc(event.message)}</div>
        </div>
      `).join('');
    }

    load();
    setInterval(load, 1500);
  </script>
</body>
</html>
"""


def row_to_dict(cursor: sqlite3.Cursor, row: tuple) -> dict:
    return {description[0]: row[index] for index, description in enumerate(cursor.description)}


def query_all(conn: sqlite3.Connection, sql: str, params: tuple = ()) -> list[dict]:
    cursor = conn.execute(sql, params)
    return [row_to_dict(cursor, row) for row in cursor.fetchall()]


def query_one(conn: sqlite3.Connection,
              sql: str,
              params: tuple = ()) -> Optional[dict]:
    cursor = conn.execute(sql, params)
    row = cursor.fetchone()
    return row_to_dict(cursor, row) if row else None


def table_columns(conn: sqlite3.Connection, table: str) -> set[str]:
    if not table_exists(conn, table):
        return set()
    return {row["name"] for row in query_all(conn, f"pragma table_info({table})")}


def table_exists(conn: sqlite3.Connection, table: str) -> bool:
    row = query_one(
        conn,
        """
        select 1 as found
        from sqlite_master
        where type = 'table' and name = ?
        limit 1
        """,
        (table,),
    )
    return row is not None


def make_state(db_path: Path) -> dict:
    if not db_path.exists():
        return {"ok": False, "error": f"База не найдена: {db_path}"}

    conn = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True, timeout=1.0)
    conn.row_factory = sqlite3.Row
    try:
        workflow = None
        if table_exists(conn, "cart_workflow"):
            workflow = query_one(
                conn,
                """
                select module_id, current_order_id, state, updated_at
                from cart_workflow
                order by module_id desc
                limit 1
                """,
            )

        module_id = int(workflow["module_id"]) if workflow else 1
        module = None
        modules: list[dict] = []
        if table_exists(conn, "modules"):
            module_columns = table_columns(conn, "modules")
            kind_expr = "kind" if "kind" in module_columns else "'UNKNOWN' as kind"
            modules = query_all(
                conn,
                f"""
                select id, serial, slot_count, {kind_expr}, status
                from modules
                order by id
                """,
            )
            module = query_one(
                conn,
                "select id, serial, slot_count, firmware, status from modules where id = ?",
                (module_id,),
            )
        order = None
        items: list[dict] = []
        if (
            workflow
            and workflow.get("current_order_id") is not None
            and table_exists(conn, "orders")
            and table_exists(conn, "order_items")
        ):
            order = query_one(
                conn,
                """
                select id, module_id, external_order_id, title, priority,
                       duration_minutes, status, created_at, updated_at
                from orders
                where id = ?
                """,
                (workflow["current_order_id"],),
            )
            item_columns = table_columns(conn, "order_items")
            part_expr = "part_number" if "part_number" in item_columns else "'' as part_number"
            qty_expr = (
                "required_quantity"
                if "required_quantity" in item_columns
                else "1 as required_quantity"
            )
            priority_expr = (
                "usage_priority"
                if "usage_priority" in item_columns
                else "3 as usage_priority"
            )
            items = query_all(
                conn,
                f"""
                select id, order_id, barcode, {part_expr}, material_type,
                       {qty_expr}, {priority_expr}, target_slot,
                       current_slot, status, updated_at
                from order_items
                where order_id = ?
                order by usage_priority, target_slot, id
                """,
                (workflow["current_order_id"],),
            )

        reels: list[dict] = []
        if table_exists(conn, "reels"):
            reels = query_all(
                conn,
                """
                select id, barcode, module_id, slot_index, placed_at
                from reels
                where module_id = ? and removed_at is null
                order by slot_index
                """,
                (module_id,),
            )
        barcode_by_slot = {int(reel["slot_index"]): reel["barcode"] for reel in reels}

        slots: list[dict] = []
        if table_exists(conn, "slot_states"):
            slots = query_all(
                conn,
                """
                select module_id, slot_index, state, updated_at
                from slot_states
                where module_id = ?
                order by slot_index
                """,
                (module_id,),
            )
        for slot in slots:
            slot["barcode"] = barcode_by_slot.get(int(slot["slot_index"]), "")

        events: list[dict] = []
        if table_exists(conn, "event_log"):
            events = query_all(
                conn,
                """
                select id, ts, level, code, coalesce(message, '') as message
                from event_log
                order by id desc
                limit 30
                """,
            )

        done_statuses = {"PLACED", "ISSUED", "RETURNED"}
        stats = {
            "total": len(items),
            "done": sum(1 for item in items if item["status"] in done_statuses),
            "wrong_slot": sum(1 for item in items if item["status"] == "WRONG_SLOT"),
        }

        return {
            "ok": True,
            "db_path": str(db_path),
            "generated_at": query_one(conn, "select datetime('now') as now")["now"],
            "module": module,
            "modules": modules,
            "workflow": workflow,
            "order": order,
            "items": items,
            "slots": slots,
            "reels": reels,
            "events": events,
            "stats": stats,
        }
    finally:
        conn.close()


class SmartCartHandler(BaseHTTPRequestHandler):
    db_path: Path = Path("smartcart.db")

    def log_message(self, fmt: str, *args) -> None:
        sys.stderr.write("[%s] %s\n" % (self.log_date_time_string(), fmt % args))

    def send_text(self, status: int, body: str, content_type: str) -> None:
        payload = body.encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", f"{content_type}; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/":
            self.send_text(200, HTML_PAGE, "text/html")
            return

        if parsed.path == "/api/state":
            try:
                body = json.dumps(make_state(self.db_path), ensure_ascii=False)
                self.send_text(200, body, "application/json")
            except Exception as exc:
                body = json.dumps(
                    {"ok": False, "error": str(exc)},
                    ensure_ascii=False,
                )
                self.send_text(500, body, "application/json")
            return

        self.send_text(404, "Not found", "text/plain")


def main() -> int:
    parser = argparse.ArgumentParser(description="Read-only SmartCart web dashboard.")
    parser.add_argument("--db", default="./smartcart.db", help="Path to smartcart.db")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()

    SmartCartHandler.db_path = Path(args.db).expanduser().resolve()
    server = ThreadingHTTPServer((args.host, args.port), SmartCartHandler)
    print(f"SmartCart web dashboard: http://{args.host}:{args.port}")
    print(f"DB: {SmartCartHandler.db_path}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
