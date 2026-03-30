// ===== src/presentation/qt/viewmodels/WorkerViewModel.cpp =====
// Исправлено:
//   - include path относительный
//   - slotsUpdated эмитирует QVector<SlotCellData>
//   - onOperationStateChanged: убраны кириллица-мусор строки в сравнениях
//   - onStateChanged: все строки через QString::fromUtf8
#include "WorkerViewModel.hpp"

#include <unordered_map>

using namespace smartcart::application;
using namespace smartcart::domain;

WorkerViewModel::WorkerViewModel(
    ports::IReelRepository&      reelRepo,
    ports::IOperationRepository& opRepo,
    AppStateMachine&             stateMachine,
    QObject*                     parent)
    : QObject(parent)
    , reelRepo_(reelRepo)
    , opRepo_(opRepo)
    , stateMachine_(stateMachine)
{
    connect(&stateMachine_, &AppStateMachine::stateChanged,
            this, &WorkerViewModel::onStateChanged);
    connect(&stateMachine_, &AppStateMachine::slotHighlighted,
            this, &WorkerViewModel::onSlotHighlighted);
    connect(&stateMachine_, &AppStateMachine::operationFinished,
            this, &WorkerViewModel::onOperationFinished);
    connect(&stateMachine_, &AppStateMachine::errorOccurred,
            this, &WorkerViewModel::onError);

    rebuildSlots();
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

void WorkerViewModel::onBarcodeScanned(const QString& barcode) {
    if (barcode.trimmed().isEmpty()) {
        emit errorOccurred(QString::fromUtf8("Пустой штрихкод — игнорируется"));
        return;
    }
    stateMachine_.scanBarcode(barcode.trimmed());
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

void WorkerViewModel::reload() { rebuildSlots(); }

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
    const auto domainSlots = reelRepo_.getSlotStates(1);
    const auto activeReels = reelRepo_.getActiveByModule(1);

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
            case SlotState::Free:     item.color = QColor(80,  80,  80);  break;
            case SlotState::Occupied: item.color = QColor(30,  80,  200); break;
            case SlotState::Reserved: item.color = QColor(200, 160,   0); break;
            case SlotState::Error:    item.color = QColor(200,  30,  30); break;
        }
        slots_.append(item);
    }

    emit slotsUpdated(slots_);
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
