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
  <title>SmartCart Operator</title>
  <style>
    :root {
      --bg: #07150d;
      --panel: #f5faf6;
      --panel-soft: #e9f3ed;
      --text: #102217;
      --muted: #587062;
      --green: #2f8f57;
      --green-2: #42b86e;
      --green-dark: #102217;
      --line: #c8d8cc;
      --free: #e4ebe7;
      --occupied: #d7e9f7;
      --reserved: #f7e8b8;
      --error: #f4c3c3;
      --warn: #fff3c9;
      --shadow: 0 18px 46px rgba(0, 0, 0, 0.22);
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: var(--bg);
      color: var(--text);
    }
    button, input { font: inherit; }
    .shell {
      min-height: 100vh;
      display: grid;
      grid-template-rows: auto 1fr;
      background:
        radial-gradient(circle at 18% 12%, rgba(66, 184, 110, 0.22), transparent 30%),
        linear-gradient(135deg, #07150d 0%, #102217 100%);
    }
    .topbar {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 18px;
      padding: 14px 22px;
      color: #fff;
    }
    .brand {
      display: flex;
      align-items: center;
      gap: 12px;
      min-width: 0;
    }
    .brand-mark {
      width: 46px;
      height: 46px;
      border-radius: 8px;
      display: grid;
      place-items: center;
      background: var(--green);
      color: white;
      font-weight: 900;
      font-size: 18px;
    }
    .brand h1 {
      margin: 0;
      font-size: 25px;
      line-height: 1;
    }
    .brand p {
      margin: 4px 0 0;
      color: #c8ddd0;
      font-size: 14px;
    }
    .top-status {
      display: flex;
      align-items: center;
      justify-content: flex-end;
      gap: 10px;
      flex-wrap: wrap;
      text-align: right;
    }
    .top-pill {
      border: 1px solid rgba(255, 255, 255, 0.18);
      border-radius: 8px;
      padding: 8px 11px;
      background: rgba(255, 255, 255, 0.08);
      color: #eaf7ee;
      font-weight: 800;
      font-size: 13px;
    }
    .content {
      padding: 0 18px 18px;
      display: grid;
      grid-template-columns: minmax(0, 1.35fr) minmax(340px, 0.65fr);
      gap: 14px;
    }
    .stage {
      grid-column: 1 / -1;
      min-height: 176px;
      border-radius: 8px;
      padding: 22px;
      background: linear-gradient(135deg, #f6fbf7 0%, #e5f3ea 100%);
      box-shadow: var(--shadow);
      border: 1px solid rgba(255, 255, 255, 0.6);
      display: grid;
      grid-template-columns: minmax(0, 1fr) auto;
      gap: 20px;
      align-items: center;
    }
    .stage.alert {
      background: linear-gradient(135deg, #fff7d5 0%, #ffe8ad 100%);
    }
    .stage.danger {
      background: linear-gradient(135deg, #ffe1e1 0%, #f5bcbc 100%);
    }
    .eyebrow {
      color: var(--muted);
      font-size: 14px;
      font-weight: 900;
      text-transform: uppercase;
    }
    .stage h2 {
      margin: 6px 0 8px;
      font-size: clamp(34px, 5vw, 58px);
      line-height: 1.02;
      letter-spacing: 0;
    }
    .stage p {
      margin: 0;
      color: #2d4936;
      font-size: clamp(20px, 2.2vw, 30px);
      line-height: 1.25;
      font-weight: 800;
    }
    .stage-meta {
      display: grid;
      gap: 8px;
      min-width: 230px;
    }
    .meta-card {
      background: rgba(255, 255, 255, 0.62);
      border: 1px solid rgba(16, 34, 23, 0.12);
      border-radius: 8px;
      padding: 10px 12px;
    }
    .meta-card span {
      display: block;
      color: var(--muted);
      font-size: 12px;
      font-weight: 800;
      text-transform: uppercase;
    }
    .meta-card strong {
      display: block;
      margin-top: 3px;
      font-size: 18px;
      overflow-wrap: anywhere;
    }
    .panel {
      background: var(--panel);
      border: 1px solid rgba(255, 255, 255, 0.68);
      border-radius: 8px;
      padding: 14px;
      box-shadow: 0 10px 28px rgba(0, 0, 0, 0.14);
      min-width: 0;
    }
    .panel h3 {
      margin: 0 0 10px;
      font-size: 17px;
    }
    .left-stack, .right-stack {
      display: grid;
      gap: 14px;
      align-content: start;
      min-width: 0;
    }
    .route {
      display: grid;
      grid-template-columns: repeat(6, minmax(96px, 1fr));
      gap: 8px;
    }
    .route-step {
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 10px;
      min-height: 82px;
      background: #f8fbf9;
      color: var(--muted);
      display: grid;
      align-content: space-between;
      gap: 6px;
    }
    .route-step.done {
      background: #e6f4eb;
      color: #254d34;
    }
    .route-step.active {
      background: var(--green);
      color: #fff;
      border-color: var(--green);
    }
    .route-index {
      width: 26px;
      height: 26px;
      border-radius: 8px;
      display: grid;
      place-items: center;
      background: rgba(16, 34, 23, 0.08);
      font-weight: 900;
    }
    .route-step.active .route-index {
      background: rgba(255, 255, 255, 0.22);
    }
    .route-label {
      font-weight: 900;
      font-size: 13px;
      line-height: 1.15;
    }
    .slots-grid {
      display: grid;
      grid-template-columns: repeat(8, minmax(72px, 1fr));
      gap: 8px;
    }
    .slot {
      min-height: 82px;
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 8px;
      display: grid;
      align-content: space-between;
      overflow: hidden;
    }
    .slot.free { background: var(--free); }
    .slot.occupied { background: var(--occupied); }
    .slot.reserved { background: var(--reserved); }
    .slot.error { background: var(--error); border-color: #d37d7d; }
    .slot-number {
      font-size: 24px;
      line-height: 1;
      font-weight: 950;
    }
    .slot-state {
      color: var(--muted);
      font-size: 12px;
      font-weight: 900;
      text-transform: uppercase;
    }
    .slot-barcode {
      font-size: 11px;
      line-height: 1.15;
      overflow-wrap: anywhere;
      color: #223a2b;
    }
    .items {
      display: grid;
      gap: 8px;
      max-height: 430px;
      overflow: auto;
      padding-right: 3px;
    }
    .item {
      display: grid;
      grid-template-columns: auto 1fr auto;
      gap: 10px;
      align-items: center;
      border: 1px solid var(--line);
      border-radius: 8px;
      background: #fff;
      padding: 10px;
    }
    .item.placed, .item.issued, .item.returned {
      background: #eefaf2;
      border-color: #afd9bd;
    }
    .item.wrong_slot {
      background: #fff0f0;
      border-color: #e2a4a4;
    }
    .priority {
      width: 44px;
      height: 44px;
      border-radius: 8px;
      display: grid;
      place-items: center;
      font-weight: 950;
      background: #edf3ef;
      color: #27523a;
      border: 1px solid var(--line);
    }
    .priority.p1 { background: #ffe5e5; color: #8f2f2f; border-color: #e5b0b0; }
    .priority.p2 { background: #fff2ce; color: #84601d; border-color: #e8cf8c; }
    .item-title {
      font-weight: 900;
      overflow-wrap: anywhere;
    }
    .item-sub {
      color: var(--muted);
      font-size: 13px;
      margin-top: 2px;
    }
    .status-badge {
      border-radius: 8px;
      padding: 7px 9px;
      background: #edf3ef;
      color: #2d4936;
      font-weight: 900;
      font-size: 12px;
      white-space: nowrap;
    }
    .modules {
      display: grid;
      gap: 8px;
    }
    .module-card {
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 10px;
      background: #fff;
      display: grid;
      gap: 4px;
    }
    .module-card.active {
      border-color: var(--green);
      box-shadow: inset 0 0 0 2px rgba(47, 143, 87, 0.18);
    }
    .module-card.online { background: #eefaf2; }
    .module-card.offline { background: #f2f5f3; opacity: 0.78; }
    .module-card strong { overflow-wrap: anywhere; }
    .module-card span {
      color: var(--muted);
      font-size: 13px;
      font-weight: 800;
    }
    .comms {
      display: grid;
      gap: 8px;
    }
    .comm-row {
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 10px;
      background: #fff;
    }
    .comm-row span {
      display: block;
      color: var(--muted);
      font-size: 12px;
      font-weight: 900;
      text-transform: uppercase;
    }
    .comm-row strong {
      display: block;
      margin-top: 3px;
      font-size: 14px;
      overflow-wrap: anywhere;
    }
    .events {
      display: grid;
      gap: 8px;
      max-height: 360px;
      overflow: auto;
    }
    .event {
      border-bottom: 1px solid #dce8e0;
      padding-bottom: 8px;
    }
    .event small {
      color: var(--muted);
      display: block;
      margin-bottom: 2px;
    }
    .event code {
      color: var(--green-dark);
      font-weight: 950;
      background: transparent;
    }
    .empty {
      color: var(--muted);
      font-weight: 800;
      padding: 8px 0;
    }
    @media (max-width: 1180px) {
      .content { grid-template-columns: 1fr; }
      .slots-grid { grid-template-columns: repeat(6, minmax(72px, 1fr)); }
      .route { grid-template-columns: repeat(3, minmax(96px, 1fr)); }
      .stage { grid-template-columns: 1fr; }
      .stage-meta { grid-template-columns: repeat(3, 1fr); }
    }
    @media (max-width: 720px) {
      .topbar { align-items: flex-start; flex-direction: column; }
      .content { padding: 0 10px 10px; }
      .stage { padding: 16px; }
      .stage h2 { font-size: 34px; }
      .stage p { font-size: 20px; }
      .stage-meta { grid-template-columns: 1fr; }
      .slots-grid { grid-template-columns: repeat(3, minmax(72px, 1fr)); }
      .route { grid-template-columns: repeat(2, minmax(96px, 1fr)); }
    }
  </style>
</head>
<body>
  <div class="shell">
    <header class="topbar">
      <div class="brand">
        <div class="brand-mark">SC</div>
        <div>
          <h1>SmartCart</h1>
          <p id="subtitle">рабочий экран тележки</p>
        </div>
      </div>
      <div class="top-status">
        <div id="healthPill" class="top-pill">связь: проверка</div>
        <div id="clockPill" class="top-pill">--:--</div>
      </div>
    </header>

    <main class="content">
      <section id="stage" class="stage">
        <div>
          <div class="eyebrow">Текущий этап</div>
          <h2 id="stageTitle">Загрузка...</h2>
          <p id="stageInstruction">Получаю состояние тележки.</p>
        </div>
        <div class="stage-meta">
          <div class="meta-card"><span>Заказ</span><strong id="orderMeta">нет данных</strong></div>
          <div class="meta-card"><span>Активный модуль</span><strong id="moduleMeta">нет данных</strong></div>
          <div class="meta-card"><span>Прогресс</span><strong id="progressMeta">0/0</strong></div>
        </div>
      </section>

      <div class="left-stack">
        <section class="panel">
          <h3>Маршрут тележки</h3>
          <div id="route" class="route"></div>
        </section>

        <section class="panel">
          <h3>Карта слотов активного модуля</h3>
          <div id="slots" class="slots-grid"></div>
        </section>

        <section class="panel">
          <h3>Материалы заказа</h3>
          <div id="items" class="items"></div>
        </section>
      </div>

      <div class="right-stack">
        <section class="panel">
          <h3>Модули</h3>
          <div id="modules" class="modules"></div>
        </section>

        <section class="panel">
          <h3>Связь и последние данные</h3>
          <div id="comms" class="comms"></div>
        </section>

        <section class="panel">
          <h3>Последние события</h3>
          <div id="events" class="events"></div>
        </section>
      </div>
    </main>

  </div>

  <script>
    const ROUTE = [
      { key: 'FREE', label: 'Свободна' },
      { key: 'ORDER_LOADED', label: 'Заказ' },
      { key: 'LOADING_FEEDERS', label: 'Загрузка питателей' },
      { key: 'PICKING_MATERIALS', label: 'Подбор' },
      { key: 'READY_FOR_FEEDER_PREP', label: 'К питателям' },
      { key: 'FEEDER_PREP', label: 'Подготовка' },
      { key: 'READY_FOR_LINE', label: 'К линии' },
      { key: 'ISSUING_TO_LINE', label: 'Выдача' },
      { key: 'ORDER_COMPLETED', label: 'Завершён' },
      { key: 'LEFTOVERS_DETECTED', label: 'Остатки' },
      { key: 'RETURNING_LEFTOVERS', label: 'Возврат' }
    ];

    const LABELS = {
      FREE: ['Тележка свободна', 'Остатков нет. Выберите заказ в основном приложении.'],
      ORDER_LOADED: ['Заказ загружен', 'Выберите состояние тележки: пустая или уже была в работе.'],
      LOADING_FEEDERS: ['Загрузка питателей', 'Загрузите питатели в модуль и подтвердите завершение.'],
      PICKING_MATERIALS: ['Подбор материалов', 'Сканируйте катушку и кладите её в подсказанный слот.'],
      READY_FOR_FEEDER_PREP: ['Доставка к питателям', 'Перевезите тележку в зону подготовки питателей.'],
      FEEDER_PREP: ['Подготовка питателей', 'Проверьте установку катушек в питатели и отметьте готовность.'],
      READY_FOR_LINE: ['Доставка на линию', 'Перевезите тележку к линии SMT.'],
      ISSUING_TO_LINE: ['Выдача на линию', 'Сканируйте выдаваемые материалы и питатели.'],
      ORDER_COMPLETED: ['Заказ завершён', 'Проверьте отчёт и наличие остатков.'],
      LEFTOVERS_DETECTED: ['Обнаружены остатки', 'Нужно выполнить возврат материалов на склад.'],
      RETURNING_LEFTOVERS: ['Возврат остатков', 'Сканируйте остатки и возвращайте их на склад.']
    };

    const STATUS_LABEL = {
      PENDING: 'ожидает',
      PLACED: 'размещён',
      ISSUED: 'выдан',
      RETURNED: 'возвращён',
      MISSING: 'нет',
      WRONG_SLOT: 'неверный слот'
    };

    const SLOT_LABEL = {
      FREE: 'свободно',
      OCCUPIED: 'занято',
      RESERVED: 'резерв',
      ERROR: 'ошибка'
    };

    const esc = (value) => String(value ?? '').replace(/[&<>"']/g, (ch) => ({
      '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#039;'
    }[ch]));

    const lower = (value) => String(value || '').toLowerCase();
    const stateClass = (state) => {
      const value = lower(state);
      if (value.includes('occupied')) return 'occupied';
      if (value.includes('reserved')) return 'reserved';
      if (value.includes('error')) return 'error';
      return 'free';
    };

    function routeIndex(stateKey) {
      const index = ROUTE.findIndex((step) => step.key === stateKey);
      return index >= 0 ? index : 0;
    }

    function firstByStatus(items, statuses) {
      return items.find((item) => statuses.includes(String(item.status || '').toUpperCase()));
    }

    function latestEvent(events, matcher) {
      return events.find((event) => matcher(String(event.code || ''), String(event.message || '')));
    }

    function screenFor(data) {
      const modules = data.modules || [];
      const hasModules = modules.length > 0;
      const onlineModules = modules.filter((module) => module.status === 'ONLINE');
      const stateKey = data.workflow ? data.workflow.state : 'FREE';
      const items = data.items || [];
      const nextPending = firstByStatus(items, ['PENDING']);
      const wrong = firstByStatus(items, ['WRONG_SLOT']);
      const leftovers = data.reels || [];

      if (hasModules && onlineModules.length === 0) {
        return {
          title: 'Вставьте модуль',
          instruction: 'RFID-метка модуля не найдена. Установите модуль в тележку.',
          className: 'danger'
        };
      }

      if (wrong) {
        return {
          title: 'Неверный слот',
          instruction: `${wrong.part_number || wrong.barcode}: нужен слот ${wrong.target_slot}. Исправьте размещение.`,
          className: 'danger'
        };
      }

      if (stateKey === 'PICKING_MATERIALS' && nextPending) {
        return {
          title: 'Подбор материалов',
          instruction: `Следующий материал: ${nextPending.part_number || nextPending.barcode}. Целевой слот ${nextPending.target_slot}.`,
          className: ''
        };
      }

      const nextIssued = firstByStatus(items, ['PLACED', 'PENDING']);
      if (stateKey === 'ISSUING_TO_LINE' && nextIssued) {
        return {
          title: 'Выдача на линию',
          instruction: `Выдать: ${nextIssued.part_number || nextIssued.barcode}. Сканируйте перед передачей.`,
          className: ''
        };
      }

      if ((stateKey === 'LEFTOVERS_DETECTED' || stateKey === 'RETURNING_LEFTOVERS') && leftovers.length) {
        return {
          title: 'Возврат остатков',
          instruction: `Осталось вернуть ${leftovers.length}. Сканируйте катушку или слот перед возвратом.`,
          className: 'alert'
        };
      }

      const fallback = LABELS[stateKey] || LABELS.FREE;
      return { title: fallback[0], instruction: fallback[1], className: '' };
    }

    function renderRoute(stateKey) {
      const active = routeIndex(stateKey);
      document.getElementById('route').innerHTML = ROUTE.map((step, index) => {
        const state = index < active ? 'done' : (index === active ? 'active' : '');
        return `
          <div class="route-step ${state}">
            <div class="route-index">${index + 1}</div>
            <div class="route-label">${esc(step.label)}</div>
          </div>
        `;
      }).join('');
    }

    function renderModules(data) {
      const activeId = data.module ? data.module.id : null;
      const modules = data.modules || [];
      document.getElementById('modules').innerHTML = modules.length ? modules.map((module) => `
        <div class="module-card ${lower(module.status)} ${module.id === activeId ? 'active' : ''}">
          <strong>${esc(module.serial)}</strong>
          <span>${esc(module.kind || 'UNKNOWN')} · ${esc(module.status)}</span>
          <span>${module.slot_count} слотов${module.id === activeId ? ' · активный' : ''}</span>
        </div>
      `).join('') : '<div class="empty">Модули не зарегистрированы</div>';
    }

    function renderSlots(slots) {
      document.getElementById('slots').innerHTML = slots.length ? slots.map((slot) => `
        <div class="slot ${stateClass(slot.state)}">
          <div>
            <div class="slot-number">${slot.slot_index}</div>
            <div class="slot-state">${esc(SLOT_LABEL[slot.state] || slot.state)}</div>
          </div>
          <div class="slot-barcode">${esc(slot.barcode || '')}</div>
        </div>
      `).join('') : '<div class="empty">Слоты пока не инициализированы</div>';
    }

    function renderItems(items) {
      document.getElementById('items').innerHTML = items.length ? items.map((item) => {
        const status = String(item.status || '').toUpperCase();
        return `
          <div class="item ${lower(status)}">
            <div class="priority p${item.usage_priority}">P${item.usage_priority}</div>
            <div>
              <div class="item-title">${esc(item.part_number || item.barcode)}</div>
              <div class="item-sub">
                ${esc(item.material_type)} · ${item.required_quantity} шт · слот ${item.target_slot}
                ${item.current_slot ? ` · факт ${item.current_slot}` : ''}
              </div>
            </div>
            <div class="status-badge">${esc(STATUS_LABEL[status] || status)}</div>
          </div>
        `;
      }).join('') : '<div class="empty">Заказ не загружен</div>';
    }

    function renderComms(data) {
      const events = data.events || [];
      const switchEvent = latestEvent(events, (code) => code === 'SwitchChanged');
      const rfidEvent = latestEvent(events, (code) => code.startsWith('Rfid'));
      const lastEvent = events[0];
      const onlineCount = (data.modules || []).filter((module) => module.status === 'ONLINE').length;
      const totalCount = (data.modules || []).length;

      document.getElementById('comms').innerHTML = `
        <div class="comm-row"><span>Модули онлайн</span><strong>${onlineCount}/${totalCount}</strong></div>
        <div class="comm-row"><span>Последний RFID</span><strong>${rfidEvent ? esc(rfidEvent.message || rfidEvent.code) : 'нет событий'}</strong></div>
        <div class="comm-row"><span>Последний слот</span><strong>${switchEvent ? esc(switchEvent.message || switchEvent.code) : 'нет событий'}</strong></div>
        <div class="comm-row"><span>Последнее обновление</span><strong>${lastEvent ? esc(lastEvent.ts) : esc(data.generated_at)}</strong></div>
      `;
    }

    function renderEvents(events) {
      document.getElementById('events').innerHTML = events.length ? events.slice(0, 10).map((event) => `
        <div class="event">
          <small>#${event.id} · ${esc(event.ts)} · ${esc(event.level)}</small>
          <code>${esc(event.code)}</code>
          <div>${esc(event.message)}</div>
        </div>
      `).join('') : '<div class="empty">Событий пока нет</div>';
    }

    async function load() {
      const response = await fetch('/api/state', { cache: 'no-store' });
      const data = await response.json();
      if (!data.ok) {
        document.getElementById('stage').className = 'stage danger';
        document.getElementById('stageTitle').textContent = 'Ошибка данных';
        document.getElementById('stageInstruction').textContent = data.error || 'Не удалось прочитать состояние.';
        document.getElementById('healthPill').textContent = 'связь: ошибка';
        return;
      }

      const stateKey = data.workflow ? data.workflow.state : 'FREE';
      const screen = screenFor(data);
      const total = data.stats ? data.stats.total : 0;
      const done = data.stats ? data.stats.done : 0;
      const moduleName = data.module ? `${data.module.serial} / ${data.module.status}` : 'нет модуля';
      const orderName = data.order ? `${data.order.external_order_id} · ${data.order.title}` : 'нет заказа';

      document.getElementById('stage').className = `stage ${screen.className}`;
      document.getElementById('stageTitle').textContent = screen.title;
      document.getElementById('stageInstruction').textContent = screen.instruction;
      document.getElementById('orderMeta').textContent = orderName;
      document.getElementById('moduleMeta').textContent = moduleName;
      document.getElementById('progressMeta').textContent = `${done}/${total}`;
      document.getElementById('healthPill').textContent = 'связь: web ok';
      document.getElementById('clockPill').textContent = data.generated_at;
      document.getElementById('subtitle').textContent = `маршрут: ${stateKey}`;

      renderRoute(stateKey);
      renderModules(data);
      renderSlots(data.slots || []);
      renderItems(data.items || []);
      renderComms(data);
      renderEvents(data.events || []);
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
                f"""
                select id, serial, slot_count, firmware, {kind_expr}, status
                from modules
                where id = ?
                """,
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
    parser = argparse.ArgumentParser(description="SmartCart operator web screen.")
    parser.add_argument("--db", default="./smartcart.db", help="Path to smartcart.db")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()

    SmartCartHandler.db_path = Path(args.db).expanduser().resolve()
    server = ThreadingHTTPServer((args.host, args.port), SmartCartHandler)
    print(f"SmartCart operator web screen: http://{args.host}:{args.port}")
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
