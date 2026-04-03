// ===== src/presentation/qt/viewmodels/WorkerViewModel.cpp =====
// Исправлено:
//   - include path относительный
//   - slotsUpdated эмитирует QVector<SlotCellData>
//   - onOperationStateChanged: убраны кириллица-мусор строки в сравнениях
//   - onStateChanged: все строки через QString::fromUtf8
#include "WorkerViewModel.hpp"

#include <QDateTime>
#include <QStringList>
#include <algorithm>
#include <string>
#include <unordered_map>

using namespace smartcart::application;
using namespace smartcart::domain;

WorkerViewModel::WorkerViewModel(
    ports::IModuleRepository&    moduleRepo,
    ports::IReelRepository&      reelRepo,
    ports::IOperationRepository& opRepo,
    ports::IOrderRepository&     orderRepo,
    ports::IWorkflowRepository&  workflowRepo,
    services::OrderImportService& orderImportSvc,
    services::WorkflowService& workflowSvc,
    AppStateMachine&             stateMachine,
    QObject*                     parent)
    : QObject(parent)
    , moduleRepo_(moduleRepo)
    , reelRepo_(reelRepo)
    , opRepo_(opRepo)
    , orderRepo_(orderRepo)
    , workflowRepo_(workflowRepo)
    , orderImportSvc_(orderImportSvc)
    , workflowSvc_(workflowSvc)
    , stateMachine_(stateMachine)
{
    connect(&stateMachine_, &AppStateMachine::stateChanged,
            this, &WorkerViewModel::onStateChanged);
    connect(&stateMachine_, &AppStateMachine::slotHighlighted,
            this, &WorkerViewModel::onSlotHighlighted);
    connect(&stateMachine_, &AppStateMachine::operationStarted,
            this, &WorkerViewModel::onOperationStarted);
    connect(&stateMachine_, &AppStateMachine::operationFinished,
            this, &WorkerViewModel::onOperationFinished);
    connect(&stateMachine_, &AppStateMachine::errorOccurred,
            this, &WorkerViewModel::onError);

    reload();
}

QString WorkerViewModel::stateLabel() const {
    switch (stateMachine_.currentState()) {
        case AppState::Idle:         return QString::fromUtf8("Ожидание");
        case AppState::Initializing: return QString::fromUtf8("Инициализация...");
        case AppState::Ready:        return QString::fromUtf8("Готов");
        case AppState::Operating:    return QString::fromUtf8("Выполняется операция");
        case AppState::Error:        return QString::fromUtf8("Ошибка");
    }
    return QString::fromUtf8("Неизвестно");
}

void WorkerViewModel::submitBarcode(const QString& barcode) {
    const auto normalized = barcode.trimmed();
    if (normalized.isEmpty()) {
        emit errorOccurred(QString::fromUtf8("Пустой штрихкод — игнорируется"));
        return;
    }
    if (!ensureActiveModuleOnline(QString::fromUtf8("Сканирование недоступно"))) {
        return;
    }

    const auto workflow = workflowRepo_.get();
    switch (workflow.state) {
        case CartWorkflowState::IssuingToLine:
            markItemIssued(normalized);
            return;
        case CartWorkflowState::LeftoversDetected:
        case CartWorkflowState::ReturningLeftovers:
            markLeftoverReturned(normalized);
            return;
        default:
            onBarcodeScanned(normalized);
            return;
    }
}

void WorkerViewModel::onBarcodeScanned(const QString& barcode) {
    if (barcode.trimmed().isEmpty()) {
        emit errorOccurred(QString::fromUtf8("Пустой штрихкод — игнорируется"));
        return;
    }
    if (!ensureActiveModuleOnline(QString::fromUtf8("Подбор недоступен"))) {
        return;
    }
    stateMachine_.scanBarcode(barcode.trimmed());
    rebuildWorkflowSummary();
}

void WorkerViewModel::onSlotPhysicalChange(int slotIndex, bool occupied) {
    SlotCellData* item = findSlot(slotIndex);
    if (!item) return;
    item->state = occupied ? SlotState::Occupied : SlotState::Free;
    if (!occupied) item->barcode.clear();
    emit slotsUpdated(slots_);
}

void WorkerViewModel::cancelCurrentOperation() {
    stateMachine_.cancelCurrentOperation();
}

void WorkerViewModel::importOrderFromFile(const QString& path) {
    if (path.trimmed().isEmpty()) {
        return;
    }
    if (!ensureActiveModuleOnline(QString::fromUtf8("Загрузка заказа недоступна"))) {
        return;
    }

    const auto result = orderImportSvc_.importFromFile(path.toStdString());
    if (!result) {
        emit errorOccurred(QString::fromStdString(result.message));
        reload();
        return;
    }

    emit operationStateChanged(QString::fromUtf8("Заказ загружен"),
                               QString::fromStdString(result.message));
    reload();
}

void WorkerViewModel::markCartArrivedToFeederPrep() {
    if (!ensureActiveModuleOnline(QString::fromUtf8("Маршрут недоступен"))) {
        return;
    }
    handleWorkflowResult(workflowSvc_.markCartArrivedToFeederPrep());
}

void WorkerViewModel::startFeederPrep() {
    if (!ensureActiveModuleOnline(QString::fromUtf8("Подготовка недоступна"))) {
        return;
    }
    handleWorkflowResult(workflowSvc_.startFeederPrep());
}

void WorkerViewModel::markFeederPrepCompleted() {
    if (!ensureActiveModuleOnline(QString::fromUtf8("Подготовка недоступна"))) {
        return;
    }
    handleWorkflowResult(workflowSvc_.markFeederPrepCompleted());
}

void WorkerViewModel::markCartArrivedToLine() {
    if (!ensureActiveModuleOnline(QString::fromUtf8("Маршрут недоступен"))) {
        return;
    }
    handleWorkflowResult(workflowSvc_.markCartArrivedToLine());
}

void WorkerViewModel::startIssuingToLine() {
    if (!ensureActiveModuleOnline(QString::fromUtf8("Выдача недоступна"))) {
        return;
    }
    handleWorkflowResult(workflowSvc_.startIssuingToLine());
}

void WorkerViewModel::markItemIssued(const QString& barcode) {
    const auto normalized = barcode.trimmed();
    if (normalized.isEmpty()) {
        emit errorOccurred(QString::fromUtf8("Введите штрихкод для выдачи"));
        return;
    }
    if (!ensureActiveModuleOnline(QString::fromUtf8("Выдача недоступна"))) {
        return;
    }
    handleWorkflowResult(workflowSvc_.markItemIssued(normalized.toStdString()));
}

void WorkerViewModel::completeIssuing() {
    if (!ensureActiveModuleOnline(QString::fromUtf8("Выдача недоступна"))) {
        return;
    }
    handleWorkflowResult(workflowSvc_.completeIssuing());
}

void WorkerViewModel::inspectLeftovers() {
    if (!ensureActiveModuleOnline(QString::fromUtf8("Проверка остатков недоступна"))) {
        return;
    }
    handleWorkflowResult(workflowSvc_.inspectLeftoversAfterOrderCompleted());
}

void WorkerViewModel::startReturningLeftovers() {
    if (!ensureActiveModuleOnline(QString::fromUtf8("Возврат недоступен"))) {
        return;
    }
    handleWorkflowResult(workflowSvc_.startReturningLeftovers());
}

void WorkerViewModel::markLeftoverReturned(const QString& barcodeOrSlot) {
    const auto normalized = barcodeOrSlot.trimmed();
    if (normalized.isEmpty()) {
        emit errorOccurred(QString::fromUtf8("Введите штрихкод или номер слота остатка"));
        return;
    }
    if (!ensureActiveModuleOnline(QString::fromUtf8("Возврат недоступен"))) {
        return;
    }

    bool isSlot = false;
    const int slotIndex = normalized.toInt(&isSlot);
    if (isSlot && slotIndex > 0) {
        handleWorkflowResult(workflowSvc_.markLeftoverReturnedBySlot(slotIndex));
        return;
    }

    handleWorkflowResult(
        workflowSvc_.markLeftoverReturnedByBarcode(normalized.toStdString()));
}

void WorkerViewModel::reload() {
    rebuildSlots();
    rebuildWorkflowSummary();
    rebuildStm32Status();
    rebuildModuleStatus();
}

void WorkerViewModel::onStateChanged(AppState newState) {
    QString label, message;
    switch (newState) {
        case AppState::Idle:
            label   = QString::fromUtf8("Ожидание");
            message = QString::fromUtf8("Система не инициализирована");
            break;
        case AppState::Initializing:
            label   = QString::fromUtf8("Инициализация");
            message = QString::fromUtf8("Подключение к модулю...");
            break;
        case AppState::Ready:
            label   = QString::fromUtf8("Готов");
            message = QString::fromUtf8("Отсканируйте штрихкод катушки");
            rebuildSlots();
            break;
        case AppState::Operating:
            label   = QString::fromUtf8("Операция");
            message = QString::fromUtf8("Следуйте световым указателям");
            break;
        case AppState::Error:
            label   = QString::fromUtf8("Ошибка");
            message = QString::fromUtf8("Требуется вмешательство оператора");
            break;
    }
    emit operationStateChanged(label, message);
}

void WorkerViewModel::onSlotHighlighted(int slotIndex, QColor color) {
    SlotCellData* item = findSlot(slotIndex);
    if (!item) return;
    const bool isOff     = (color == QColor(0, 0, 0));
    item->highlighted    = !isOff;
    item->color          = isOff ? QColor(128, 128, 128) : color;
    emit slotsUpdated(slots_);
}

void WorkerViewModel::onOperationStarted(int operationId, OperationType) {
    QString message = QString::fromUtf8("Скан принят. Нажмите целевую ячейку");
    const auto op = opRepo_.getById(operationId);
    if (op.has_value() && op->slotIndex > 0) {
        message = QString::fromUtf8("Скан принят. Положите в слот %1")
                      .arg(op->slotIndex);
    }
    emit operationStateChanged(QString::fromUtf8("Подбор материалов"), message);
    rebuildWorkflowSummary();
}

void WorkerViewModel::onOperationFinished(int, OperationStatus status) {
    QString label, message;
    switch (status) {
        case OperationStatus::Completed:
            label   = QString::fromUtf8("Готов");
            message = QString::fromUtf8("Операция выполнена успешно");
            break;
        case OperationStatus::Cancelled:
            label   = QString::fromUtf8("Отменено");
            message = QString::fromUtf8("Операция отменена");
            break;
        case OperationStatus::Failed:
            label   = QString::fromUtf8("Ошибка");
            message = QString::fromUtf8("Операция завершилась с ошибкой");
            break;
        default:
            label   = QString::fromUtf8("Готов");
            message = "";
            break;
    }
    rebuildSlots();
    emit operationStateChanged(label, message);
}

void WorkerViewModel::onError(ErrorCode code, QString message) {
    emit errorOccurred(errorMessage(code) + ": " + message);
}

SlotCellData* WorkerViewModel::findSlot(int slotIndex) {
    for (auto& item : slots_)
        if (item.slotIndex == slotIndex) return &item;
    return nullptr;
}

void WorkerViewModel::rebuildSlots() {
    const auto workflow = workflowRepo_.get();
    const auto domainSlots = reelRepo_.getSlotStates(workflow.moduleId);
    const auto activeReels = reelRepo_.getActiveByModule(workflow.moduleId);

    std::unordered_map<int, QString> barcodeBySlot;
    for (const auto& reel : activeReels)
        barcodeBySlot[reel.slotIndex] = QString::fromStdString(reel.barcode);

    slots_.clear();
    slots_.reserve(static_cast<int>(domainSlots.size()));

    for (const auto& slot : domainSlots) {
        SlotCellData item;
        item.slotIndex = slot.slotIndex;
        item.state     = slot.state;
        item.barcode   = barcodeBySlot.count(slot.slotIndex)
                         ? barcodeBySlot.at(slot.slotIndex)
                         : "";
        switch (slot.state) {
            case SlotState::Free:     item.color = QColor(211, 218, 212); break;
            case SlotState::Occupied: item.color = QColor(78,  143,  97); break;
            case SlotState::Reserved: item.color = QColor(216, 183,  94); break;
            case SlotState::Error:    item.color = QColor(190,  85,  74); break;
        }
        slots_.append(item);
    }

    emit slotsUpdated(slots_);
}

void WorkerViewModel::rebuildWorkflowSummary() {
    const auto workflow = workflowRepo_.get();
    QString orderText = QString::fromUtf8("Заказ не загружен");
    QString checklistText = QString::fromUtf8("Загрузите JSON заказа");
    QString progressText = QString::fromUtf8("Материалы: 0/0");
    const bool showLeftovers =
        workflow.state == CartWorkflowState::LeftoversDetected ||
        workflow.state == CartWorkflowState::ReturningLeftovers;
    const auto leftovers = reelRepo_.getActiveByModule(workflow.moduleId);
    const bool showStartPage =
        workflow.state == CartWorkflowState::Free &&
        !workflow.currentOrderId.has_value() &&
        leftovers.empty();

    if (!workflow.currentOrderId.has_value()) {
        if (!leftovers.empty()) {
            QStringList lines;
            for (const auto& reel : leftovers) {
                lines << QString::fromUtf8("%1 → слот %2 → вернуть на склад")
                             .arg(QString::fromStdString(reel.barcode))
                             .arg(reel.slotIndex);
            }

            orderText = QString::fromUtf8("В тележке есть остатки");
            checklistText = lines.join('\n');
            progressText = QString::fromUtf8("Остатки: %1")
                               .arg(static_cast<int>(leftovers.size()));
        }
    } else {
        const auto order = orderRepo_.getOrderById(*workflow.currentOrderId);
        if (order.has_value()) {
            orderText =
                QString::fromUtf8(
                    "<b>%1</b> — %2<br>"
                    "<span style=\"color:%3; font-weight:700;\">● %4</span>"
                    " · %5")
                    .arg(QString::fromStdString(order->externalOrderId).toHtmlEscaped(),
                         QString::fromStdString(order->title).toHtmlEscaped(),
                         priorityColor(order->priority),
                         priorityLabel(order->priority),
                         orderTimeText(*order).toHtmlEscaped());

            const auto items = orderRepo_.getItems(order->id);
            QStringList lines;
            int placedOrIssued = 0;
            int issued = 0;
            int wrongSlot = 0;
            int pending = 0;
            for (const auto& item : items) {
                switch (item.status) {
                    case OrderItemStatus::Placed:
                        ++placedOrIssued;
                        break;
                    case OrderItemStatus::Issued:
                        ++placedOrIssued;
                        ++issued;
                        break;
                    case OrderItemStatus::WrongSlot:
                        ++wrongSlot;
                        break;
                    case OrderItemStatus::Pending:
                    case OrderItemStatus::Missing:
                    case OrderItemStatus::Returned:
                        ++pending;
                        break;
                }

                QString line = QString::fromUtf8("%1 → слот %2 [%3]")
                    .arg(QString::fromStdString(item.barcode))
                    .arg(item.targetSlot)
                    .arg(itemStatusLabel(item.status));
                if (item.currentSlot.has_value()) {
                    line += QString::fromUtf8(" факт: %1").arg(*item.currentSlot);
                }
                lines << line;
            }
            checklistText = lines.isEmpty()
                ? QString::fromUtf8("В заказе нет позиций")
                : lines.join('\n');

            progressText =
                QString::fromUtf8("Материалы: %1/%2 размещены · выдано: %3 · ошибок: %4 · ожидает: %5")
                    .arg(placedOrIssued)
                    .arg(static_cast<int>(items.size()))
                    .arg(issued)
                    .arg(wrongSlot)
                    .arg(pending);

            if (showLeftovers) {
                QStringList leftoverLines;
                for (const auto& reel : leftovers) {
                    leftoverLines << QString::fromUtf8("%1 → слот %2 → вернуть на склад")
                                         .arg(QString::fromStdString(reel.barcode))
                                         .arg(reel.slotIndex);
                }

                checklistText = leftoverLines.isEmpty()
                    ? QString::fromUtf8("Остатков нет")
                    : QString::fromUtf8("Остатки к возврату:\n") +
                          leftoverLines.join('\n');
                progressText = QString::fromUtf8("Остатки: %1")
                                   .arg(static_cast<int>(leftovers.size()));
            }
        }
    }

    emit workflowUpdated(workflowLabel(workflow.state),
                         orderText,
                         checklistText,
                         progressText,
                         showStartPage);
    emit workflowControlsUpdated(
        QString::fromStdString(std::string(toString(workflow.state))));
}

void WorkerViewModel::rebuildStm32Status() {
    const auto status = stateMachine_.stm32ConnectionStatus();

    const QString uartText = status.uartOpen
        ? QString::fromUtf8("UART открыт")
        : QString::fromUtf8("UART закрыт");
    const QString pollingText = status.pollingRunning
        ? QString::fromUtf8("обмен активен")
        : QString::fromUtf8("обмен ожидает старта");

    const QString eventAt = status.lastEventAt.empty()
        ? QString::fromUtf8("нет времени")
        : QString::fromStdString(status.lastEventAt);
    const QString snapshotAt = status.lastSnapshotAt.empty()
        ? QString::fromUtf8("нет времени")
        : QString::fromStdString(status.lastSnapshotAt);

    emit stm32StatusUpdated(
        QString::fromUtf8("%1 · %2\nEVT: %3 (%4)\nsnapshot: %5 (%6)")
            .arg(uartText,
                 pollingText,
                 QString::fromStdString(status.lastEvent),
                 eventAt,
                 QString::fromStdString(status.lastSnapshot),
                 snapshotAt)
    );
}

void WorkerViewModel::rebuildModuleStatus() {
    const auto workflow = workflowRepo_.get();
    const auto module = moduleRepo_.getById(workflow.moduleId);

    if (!module.has_value()) {
        emit activeModuleUpdated(
            QString::fromUtf8("Активный модуль: #%1 · запись не найдена")
                .arg(workflow.moduleId));
        emit activeModuleAvailabilityChanged(false);
        return;
    }

    QString statusText;
    QString statusColor;
    switch (module->status) {
        case ModuleStatus::Online:
            statusText = QString::fromUtf8("онлайн");
            statusColor = QStringLiteral("#8DE4AD");
            break;
        case ModuleStatus::Offline:
            statusText = QString::fromUtf8("офлайн");
            statusColor = QStringLiteral("#C6CFCA");
            break;
        case ModuleStatus::Maint:
            statusText = QString::fromUtf8("обслуживание");
            statusColor = QStringLiteral("#F0C66B");
            break;
    }

    emit activeModuleUpdated(
        QString::fromUtf8(
            "Активный модуль: <b>#%1</b> · %2 · "
            "<span style=\"color:%3; font-weight:700;\">● %4</span>")
            .arg(module->id)
            .arg(QString::fromStdString(module->serial).toHtmlEscaped())
            .arg(statusColor)
            .arg(statusText));
    emit activeModuleAvailabilityChanged(module->status == ModuleStatus::Online);
}

bool WorkerViewModel::isActiveModuleOnline() const {
    const auto workflow = workflowRepo_.get();
    const auto module = moduleRepo_.getById(workflow.moduleId);
    return module.has_value() && module->status == ModuleStatus::Online;
}

bool WorkerViewModel::ensureActiveModuleOnline(const QString& actionLabel) {
    if (isActiveModuleOnline()) {
        return true;
    }

    emit operationStateChanged(
        QString::fromUtf8("Модуль недоступен"),
        actionLabel + QString::fromUtf8(". Вставьте модуль и дождитесь перехода в онлайн."));
    emit activeModuleAvailabilityChanged(false);
    return false;
}

void WorkerViewModel::handleWorkflowResult(
    const services::WorkflowActionResult& result)
{
    if (!result) {
        emit errorOccurred(QString::fromStdString(result.message));
        reload();
        return;
    }

    emit operationStateChanged(QString::fromUtf8("Маршрут"),
                               QString::fromStdString(result.message));
    reload();
}

QString WorkerViewModel::errorMessage(ErrorCode code) {
    switch (code) {
        case ErrorCode::None:                    return QString::fromUtf8("Нет ошибки");
        case ErrorCode::InvalidConfig:           return QString::fromUtf8("Неверная конфигурация");
        case ErrorCode::InvalidBarcode:          return QString::fromUtf8("Некорректный штрихкод");
        case ErrorCode::SlotNotFound:            return QString::fromUtf8("Слот не найден");
        case ErrorCode::SlotOccupied:            return QString::fromUtf8("Слот занят");
        case ErrorCode::NoFreeSlot:              return QString::fromUtf8("Нет свободных слотов");
        case ErrorCode::ReelNotFound:            return QString::fromUtf8("Катушка не найдена");
        case ErrorCode::WrongSlotInteraction:    return QString::fromUtf8("Неверный слот");
        case ErrorCode::Stm32CommunicationError: return QString::fromUtf8("Ошибка связи с модулем");
        case ErrorCode::RfidReadError:           return QString::fromUtf8("Ошибка чтения RFID");
        case ErrorCode::ScannerReadError:        return QString::fromUtf8("Ошибка сканера");
        case ErrorCode::PersistenceError:        return QString::fromUtf8("Ошибка базы данных");
        case ErrorCode::RecoveryFailed:          return QString::fromUtf8("Ошибка восстановления");
        case ErrorCode::Timeout:                 return QString::fromUtf8("Таймаут операции");
        case ErrorCode::Unknown:                 return QString::fromUtf8("Неизвестная ошибка");
    }
    return QString::fromUtf8("Неизвестная ошибка");
}

QString WorkerViewModel::workflowLabel(CartWorkflowState state) {
    switch (state) {
        case CartWorkflowState::Free:               return QString::fromUtf8("Тележка свободна");
        case CartWorkflowState::OrderLoaded:        return QString::fromUtf8("Заказ загружен");
        case CartWorkflowState::PickingMaterials:   return QString::fromUtf8("Подбор материалов");
        case CartWorkflowState::ReadyForFeederPrep: return QString::fromUtf8("Готова к подготовке питателей");
        case CartWorkflowState::FeederPrep:         return QString::fromUtf8("Подготовка питателей");
        case CartWorkflowState::ReadyForLine:       return QString::fromUtf8("Готова к передаче на линию");
        case CartWorkflowState::IssuingToLine:      return QString::fromUtf8("Выдача на линию");
        case CartWorkflowState::OrderCompleted:     return QString::fromUtf8("Заказ завершён");
        case CartWorkflowState::LeftoversDetected:  return QString::fromUtf8("Обнаружены остатки");
        case CartWorkflowState::ReturningLeftovers: return QString::fromUtf8("Возврат остатков");
    }
    return QString::fromUtf8("Неизвестно");
}

QString WorkerViewModel::itemStatusLabel(OrderItemStatus status) {
    switch (status) {
        case OrderItemStatus::Pending:   return QString::fromUtf8("ожидает");
        case OrderItemStatus::Placed:    return QString::fromUtf8("размещён");
        case OrderItemStatus::Issued:    return QString::fromUtf8("выдан");
        case OrderItemStatus::Returned:  return QString::fromUtf8("возвращён");
        case OrderItemStatus::Missing:   return QString::fromUtf8("не найден");
        case OrderItemStatus::WrongSlot: return QString::fromUtf8("неверный слот");
    }
    return QString::fromUtf8("неизвестно");
}

QString WorkerViewModel::priorityColor(const std::string& priority) {
    const QString value = QString::fromStdString(priority).trimmed().toLower();
    if (value == QStringLiteral("high") ||
        value == QStringLiteral("urgent") ||
        value == QStringLiteral("critical") ||
        value == QString::fromUtf8("высокий") ||
        value == QString::fromUtf8("срочно"))
    {
        return QStringLiteral("#B85C5C");
    }

    if (value == QStringLiteral("low") ||
        value == QString::fromUtf8("низкий"))
    {
        return QStringLiteral("#6E8F78");
    }

    return QStringLiteral("#C58A36");
}

QString WorkerViewModel::priorityLabel(const std::string& priority) {
    const QString value = QString::fromStdString(priority).trimmed();
    if (value.isEmpty()) {
        return QString::fromUtf8("приоритет не задан");
    }

    const QString lower = value.toLower();
    if (lower == QStringLiteral("high") ||
        lower == QStringLiteral("urgent") ||
        lower == QStringLiteral("critical"))
    {
        return QString::fromUtf8("высокий приоритет");
    }
    if (lower == QStringLiteral("normal") ||
        lower == QStringLiteral("medium"))
    {
        return QString::fromUtf8("обычный приоритет");
    }
    if (lower == QStringLiteral("low")) {
        return QString::fromUtf8("низкий приоритет");
    }

    return value.toHtmlEscaped();
}

QString WorkerViewModel::orderTimeText(const OrderInfo& order) {
    if (order.durationMinutes <= 0) {
        return QString::fromUtf8("время не задано");
    }

    QDateTime createdAt =
        QDateTime::fromString(QString::fromStdString(order.createdAt),
                              QStringLiteral("yyyy-MM-dd HH:mm:ss"));
    if (!createdAt.isValid()) {
        return QString::fromUtf8("выделено: %1 мин").arg(order.durationMinutes);
    }

    const qint64 elapsedSeconds = createdAt.secsTo(QDateTime::currentDateTime());
    const int elapsedMinutes =
        static_cast<int>(std::max<qint64>(0, elapsedSeconds) / 60);
    const int remainingMinutes = order.durationMinutes - elapsedMinutes;

    if (remainingMinutes >= 0) {
        return QString::fromUtf8("выделено: %1 мин, осталось: %2 мин")
            .arg(order.durationMinutes)
            .arg(remainingMinutes);
    }

    return QString::fromUtf8("выделено: %1 мин, просрочено на %2 мин")
        .arg(order.durationMinutes)
        .arg(-remainingMinutes);
}
